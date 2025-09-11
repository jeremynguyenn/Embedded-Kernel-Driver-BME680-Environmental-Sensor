#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <pthread.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/netlink.h>
#include <linux/seccomp.h>
#include <linux/filter.h>
#include <linux/prctl.h>
#include <string.h>
#include <math.h>
#include "bme680.h"

#define SHM_NAME "/bme680_shm"
#define PIPE_BUF_SIZE 1024
#define MAX_READS 100
#define NETLINK_USER 31
#define IAQ_GOOD 0
#define IAQ_MODERATE 1
#define IAQ_POOR 2
#define GAS_THRESHOLD 100000

volatile sig_atomic_t keep_running = 1;
pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t data_cond = PTHREAD_COND_INITIALIZER;
struct bme680_fifo_data *shared_data;
int shm_fd;
int pipe_fds[2];
int nl_sock;

int bme680_calculate_iaq(float humidity, u32 gas_resistance) {
    float iaq = 25.0 * log(gas_resistance / 100000.0) - humidity + 50.0;
    if (iaq > 100) return IAQ_GOOD;
    if (iaq > 50) return IAQ_MODERATE;
    return IAQ_POOR;
}

void *netlink_listener(void *arg) {
    struct sockaddr_nl src_addr = {0};
    struct nlmsghdr *nlh = NULL;
    char buf[4096];

    nl_sock = socket(PF_NETLINK, SOCK_RAW, NETLINK_USER);
    if (nl_sock < 0) {
        perror("Netlink socket failed");
        pthread_exit(NULL);
    }

    memset(&src_addr, 0, sizeof(src_addr));
    src_addr.nl_family = AF_NETLINK;
    src_addr.nl_pid = getpid();
    src_addr.nl_groups = 1;
    if (bind(nl_sock, (struct sockaddr *)&src_addr, sizeof(src_addr)) < 0) {
        perror("Netlink bind failed");
        close(nl_sock);
        pthread_exit(NULL);
    }

    while (keep_running) {
        int len = recv(nl_sock, buf, sizeof(buf), 0);
        if (len < 0) continue;
        nlh = (struct nlmsghdr *)buf;
        if (nlh->nlmsg_type == NLMSG_DONE) break;
        printf("Alert from kernel: %s\n", (char *)NLMSG_DATA(nlh));
    }
    close(nl_sock);
    pthread_exit(NULL);
}

void setup_seccomp_filter() {
    struct sock_filter filter[] = {
        BPF_STMT(BPF_LD + BPF_W + BPF_ABS, 0),
        BPF_JUMP(BPF_JMP + BPF_JEQ + BPF_K, __NR_ioctl, 0, 1),
        BPF_STMT(BPF_RET + BPF_K, SECCOMP_RET_ALLOW),
        BPF_STMT(BPF_RET + BPF_K, SECCOMP_RET_KILL),
    };
    struct sock_fprog prog = {
        .len = (unsigned short)(sizeof(filter) / sizeof(filter[0])),
        .filter = filter,
    };
    if (prctl(PR_SET_SECCOMP, 2, &prog) < 0) {
        perror("Seccomp setup failed");
    }
}

void signal_handler(int sig) {
    keep_running = 0;
    pthread_cond_broadcast(&data_cond);
    if (sig == SIGINT) printf("\nReceived SIGINT, cleaning up...\n");
    signal(sig, SIG_DFL);
}

void *read_thread(void *arg) {
    int interval_ms = *(int *)arg;
    int fd = open("/dev/bme680", O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        perror("Failed to open /dev/bme680");
        pthread_exit(NULL);
    }

    struct bme680_gas_config gas_config = {
        .heater_temp = 320, .heater_dur = 150, .preheat_curr_ma = 10
    };
    ioctl(fd, BME680_IOC_SET_GAS_CONFIG, &gas_config);

    struct bme680_fifo_data data;
    while (keep_running) {
        int retries = 5;
        while (retries--) {
            if (ioctl(fd, BME680_IOC_READ_FIFO, &data) == 0) break;
            if (errno == EAGAIN) { usleep(100000); continue; }
            perror("Failed to read FIFO");
            close(fd);
            pthread_exit(NULL);
        }

        int iaq = bme680_calculate_iaq(data.humidity / 1000.0, data.gas_resistance);
        data.iaq_index = iaq;
        printf("Temp: %.2f °C, Press: %.2f hPa, Hum: %.2f %%, Gas: %u Ohms, IAQ: %d\n",
               data.temperature / 1000000.0, data.pressure / 100000.0, data.humidity / 1000.0,
               data.gas_resistance, iaq);

        pthread_mutex_lock(&data_mutex);
        memcpy(shared_data, &data, sizeof(struct bme680_fifo_data));
        pthread_cond_signal(&data_cond);
        pthread_mutex_unlock(&data_mutex);

        usleep(interval_ms * 1000);
    }
    close(fd);
    pthread_exit(NULL);
}

void *process_thread(void *arg) {
    struct bme680_fifo_data local_data;
    while (keep_running) {
        pthread_mutex_lock(&data_mutex);
        pthread_cond_wait(&data_cond, &data_mutex);
        if (!keep_running) { pthread_mutex_unlock(&data_mutex); break; }
        memcpy(&local_data, shared_data, sizeof(struct bme680_fifo_data));
        pthread_mutex_unlock(&data_mutex);

        char buf[PIPE_BUF_SIZE];
        snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f,%u,%lld,%u\n",
                 local_data.temperature / 1000000.0, local_data.pressure / 100000.0,
                 local_data.humidity / 1000.0, local_data.gas_resistance, local_data.timestamp,
                 local_data.iaq_index);
        write(pipe_fds[1], buf, strlen(buf));
    }
    pthread_exit(NULL);
}

int main(int argc, char *argv[]) {
    int opt, interval_ms = 500, num_reads = 0, use_sysfs = 0;

    setup_seccomp_filter();

    struct sigaction sa = {0};
    sa.sa_handler = signal_handler;
    sigaction(SIGINT, &sa, NULL);

    shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, sizeof(struct bme680_fifo_data));
    shared_data = mmap(NULL, sizeof(struct bme680_fifo_data), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    memset(shared_data, 0, sizeof(struct bme680_fifo_data));

    while ((opt = getopt(argc, argv, "i:n:s")) != -1) {
        switch (opt) {
            case 'i': interval_ms = atoi(optarg); break;
            case 'n': num_reads = atoi(optarg); break;
            case 's': use_sysfs = 1; break;
        }
    }

    pipe(pipe_fds);
    pid_t pid = fork();
    if (pid == 0) {
        close(pipe_fds[1]);
        dup2(pipe_fds[0], STDIN_FILENO);
        close(pipe_fds[0]);
        execlp("cat", "cat", NULL);
        exit(1);
    } else {
        close(pipe_fds[0]);
    }

    pthread_t read_tid, process_tid, nl_tid;
    pthread_create(&read_tid, NULL, read_thread, &interval_ms);
    pthread_create(&process_tid, NULL, process_thread, NULL);
    pthread_create(&nl_tid, NULL, netlink_listener, NULL);

    struct bme680_fifo_data *reads = malloc(MAX_READS * sizeof(struct bme680_fifo_data));
    memset(reads, 0, MAX_READS * sizeof(struct bme680_fifo_data));
    int read_count = 0;
    loff_t seek_pos = 0;

    while (keep_running && (num_reads == 0 || read_count < num_reads)) {
        pthread_mutex_lock(&data_mutex);
        if (shared_data->timestamp != 0) {
            if (seek_pos >= 0 && seek_pos < read_count) {
                reads[(int)seek_pos] = *shared_data;
                seek_pos++;
            } else {
                if (read_count >= MAX_READS) {
                    reads = realloc(reads, MAX_READS * 2 * sizeof(struct bme680_fifo_data));
                    memset(reads + MAX_READS, 0, MAX_READS * sizeof(struct bme680_fifo_data));
                    MAX_READS *= 2;
                }
                reads[read_count++] = *shared_data;
            }
            shared_data->timestamp = 0;
        }
        pthread_mutex_unlock(&data_mutex);
        sleep(1);
    }

    keep_running = 0;
    pthread_join(read_tid, NULL);
    pthread_join(process_tid, NULL);
    pthread_join(nl_tid, NULL);
    kill(pid, SIGTERM);
    waitpid(pid, NULL, 0);
    close(pipe_fds[1]);
    free(reads);
    munmap(shared_data, sizeof(struct bme680_fifo_data));
    shm_unlink(SHM_NAME);
    close(shm_fd);
    pthread_mutex_destroy(&data_mutex);
    pthread_cond_destroy(&data_cond);

    if (use_sysfs) {
        int sysfs_interval = interval_ms;
        while (keep_running && (num_reads == 0 || num_reads-- > 0)) {
            FILE *temp_file = fopen("/sys/bus/iio/devices/iio:device0/in_temp_input", "r");
            FILE *press_file = fopen("/sys/bus/iio/devices/iio:device0/in_pressure_input", "r");
            FILE *humid_file = fopen("/sys/bus/iio/devices/iio:device0/in_humidityrelative_input", "r");
            FILE *gas_file = fopen("/sys/bus/iio/devices/iio:device0/in_resistance_input", "r");
            if (!temp_file || !press_file || !humid_file || !gas_file) {
                perror("Failed to open sysfs files");
                if (temp_file) fclose(temp_file);
                if (press_file) fclose(press_file);
                if (humid_file) fclose(humid_file);
                if (gas_file) fclose(gas_file);
                break;
            }

            float temp, press, humid, gas;
            fscanf(temp_file, "%f", &temp);
            fscanf(press_file, "%f", &press);
            fscanf(humid_file, "%f", &humid);
            fscanf(gas_file, "%f", &gas);
            printf("SysFS - Temperature: %.2f °C\n", temp / 1000.0);
            printf("SysFS - Pressure: %.2f hPa\n", press * 10.0);
            printf("SysFS - Humidity: %.2f %%\n", humid / 1000.0);
            printf("SysFS - Gas Resistance: %.0f Ohms\n", gas);
            fclose(temp_file); fclose(press_file); fclose(humid_file); fclose(gas_file);
            usleep(sysfs_interval * 1000);
        }
    }

    printf("Application terminated normally.\n");
    return 0;
}