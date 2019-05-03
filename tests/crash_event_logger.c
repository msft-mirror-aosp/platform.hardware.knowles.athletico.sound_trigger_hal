/*
 * Copyright (C) 2018 Knowles Electronics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define LOG_TAG "ia_crash_event_logger"
#include <cutils/log.h>
#include <cutils/uevent.h>

#define UEVENT_MSG_LEN          (1024)
#define BUF_SIZE                (4096)
#define CRASH_LOGGER_DEV        "/dev/crashdump"
#define REGDUMP_LOGGER_DEV	"/dev/regdump"
#define CRASH_DUMP_FILE_PREFIX  "/data/data/dump_crash_"
#define REG_ACCESS_FILE_PREFIX	"/data/data/dump_reg_access_history_"
#define BIN_EXTN                ".bin"
#define TXT_EXTN                ".txt"
#define MAX_FILENAME_LEN        512
#define MAX_TIMESTR_LEN         64

int g_exit_socket[2];

void sigint_handler(int sig __unused) {
    ALOGE("Interrupted, setting the exit condition");
    if (g_exit_socket[0] >= 0)
        write(g_exit_socket[0], "T", 1);
}

char *crash_dump_split_file_names[] =
    {"/data/data/dump_debug_CM4_",
    "/data/data/dump_debug_HMD_",
    "/data/data/dump_debug_DMX_",
    "/data/data/dump_crash_CM4_",
    "/data/data/dump_crash_HMD_",
    "/data/data/dump_crash_DMX_"
    };

int split_bin(unsigned char *buf, int len, const char* time_stamp)
{
    unsigned int header, size, tot_len, flen;
    int fcount = 0;
    unsigned char *ptr;
    FILE *fp;
    char file_name[MAX_FILENAME_LEN];
    int no_crashdump_files = sizeof(crash_dump_split_file_names) /
            sizeof(crash_dump_split_file_names[0]);

    tot_len = 0;
    while ((tot_len < len) && (fcount++ < no_crashdump_files))
    {
        header = buf[tot_len] ;

        size = buf[tot_len+8] |
                buf[tot_len+9]  << 8  |
                buf[tot_len+10] << 16 |
                buf[tot_len+11] << 24 ;

        tot_len += 12;

        strcpy(file_name, crash_dump_split_file_names[header & 0xf]);
        strcat(file_name, time_stamp);
        strcat(file_name, BIN_EXTN);

        fp = fopen (file_name, "w+");

        ptr = buf + tot_len;
        flen = fwrite(ptr , 1, size, fp);
        tot_len += size;
        fclose(fp);
        ALOGI("Crash logs saved to %s", file_name);
    }
    return 0;
}

int split_crash_dump_file (const char* crash_dump_filename,
                        const char* time_stamp)
{
    int fd, fil_len;
    FILE *fp;
    struct stat st;
    unsigned char *buf;
    int len,ret ;

    fp = fopen(crash_dump_filename, "r");
    if (!fp)
    {
        ALOGE("File open error %s \n", crash_dump_filename);
        return -1;
    }

    fd = fileno(fp);
    fstat(fd, &st);
    fil_len = st.st_size;
    buf = (unsigned char*) malloc(fil_len);

    if (NULL == buf) {
        ALOGE("Failed to allocate buffer exiting");
        ret = -1;
        goto exit;
    }

    len = fread(buf,1,  fil_len, fp);
    if (len <=0) {
        ALOGE("file reading error %s\n", crash_dump_filename);
        ret = -1;
        goto exit;
    }
    ret = split_bin(buf, len, time_stamp);

exit:
    if (fp)
        fclose (fp);
    if (buf)
        free(buf);
    return ret;
}

void dump_crash_log() {
    void *buf = NULL;
    int inp_fp = -1, out_fp = -1;
    int bytes_read = 0;
    int err = 0;
    time_t t;
    struct tm *tm;
    char file_name[MAX_FILENAME_LEN];
    char curr_time[MAX_TIMESTR_LEN];

    buf = malloc(BUF_SIZE);
    if (NULL == buf) {
        ALOGE("Failed to allocate buffer exiting");
        err = -1;
        goto exit;
    }

    inp_fp = open(CRASH_LOGGER_DEV, O_RDONLY);
    if (inp_fp == -1) {
        ALOGE("Failed to open %s with error %d(%s)",
                CRASH_LOGGER_DEV, errno, strerror(errno));
        goto exit;
    }

    strcpy(file_name, CRASH_DUMP_FILE_PREFIX);
    t = time(NULL);
    tm = localtime(&t);
    strftime(curr_time, 64, "%F_%H_%M_%S", tm);
    strcat(file_name, curr_time);
    strcat(file_name, BIN_EXTN);

    out_fp = open(file_name, O_WRONLY | O_CREAT, 0644);
    if (out_fp == -1) {
        ALOGE("Failed to open %s for writing", file_name);
        goto exit;
    }

    do {
        bytes_read = read(inp_fp, buf, BUF_SIZE);
        if (bytes_read > 0)
            write(out_fp, buf, bytes_read);
    } while (bytes_read > 0);

    ALOGI("Crash logs has been dumped to %s", file_name);
    close(out_fp);
    out_fp = -1;
    close(inp_fp);
    inp_fp = -1;
    free(buf);
    buf = NULL;
    split_crash_dump_file(file_name, curr_time);

exit:
    if (out_fp != -1) {
        close(out_fp);
    }

    if (inp_fp != -1) {
        close(inp_fp);
    }

    if (buf) {
        free(buf);
    }
}

void dump_reg_access_hist_log() {
    void *buf = NULL;
    int inp_fp = -1, out_fp = -1;
    int bytes_read = 0;
    int err = 0;
    time_t t;
    struct tm *tm;
    char file_name[MAX_FILENAME_LEN];
    char curr_time[MAX_TIMESTR_LEN];

    buf = malloc(BUF_SIZE);
    if (!buf) {
        ALOGE("Failed to allocate buffer exiting");
        err = -1;
        goto exit;
    }

    inp_fp = open(REGDUMP_LOGGER_DEV, O_RDONLY);
    if (inp_fp == -1) {
        ALOGE("Failed to open %s with error %d(%s)",
                REGDUMP_LOGGER_DEV, errno, strerror(errno));
        goto exit;
    }

    strcpy(file_name, REG_ACCESS_FILE_PREFIX);
    t = time(NULL);
    tm = localtime(&t);
    strftime(curr_time, 64, "%F_%H_%M_%S", tm);
    strcat(file_name, curr_time);
    strcat(file_name, TXT_EXTN);

    out_fp = open(file_name, O_WRONLY | O_CREAT, 0644);
    if (out_fp == -1) {
        ALOGE("Failed to open %s for writing", file_name);
        goto exit;
    }

    do {
        bytes_read = read(inp_fp, buf, BUF_SIZE);
        if (bytes_read > 0)
            write(out_fp, buf, bytes_read);
    } while (bytes_read > 0);

    ALOGI("Register access history has been dumped to %s", file_name);

exit:
    if (out_fp != -1) {
        close(out_fp);
    }

    if (inp_fp != -1) {
        close(inp_fp);
    }

    if (buf) {
        free(buf);
    }
}

int main(int argc, char** argv) {
    int err = 0;
    int timeout = -1; // Wait for event indefinitely
    struct pollfd fds[2];
    char msg[UEVENT_MSG_LEN];
    int i, n;

    signal(SIGINT, sigint_handler);

    if ( (argc == 2) && !strcmp(argv[1], "-f")) {
        ALOGD("Read to get the crash logs");
        dump_reg_access_hist_log();
        dump_crash_log();
        return 0;
    }

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, g_exit_socket) == -1) {
        ALOGE("%s: Failed to create termination socket", __func__);
        err = -1;
        goto exit;
    }


    fds[0].events = POLLIN;
    fds[0].fd = uevent_open_socket(64*1024, true);
    if (fds[0].fd == -1) {
        ALOGE("Error opening socket for hotplug uevent errno %d(%s)",
                errno, strerror(errno));
        goto exit;
    }
    fds[1].events = POLLIN;
    fds[1].fd = g_exit_socket[1];

    while (1) {
        err = poll (fds, 2, timeout);

        if (fds[0].revents & POLLIN) {
            n = uevent_kernel_multicast_recv(fds[0].fd, msg, UEVENT_MSG_LEN);
            if (n <= 0) {
                continue;
            }

            for (i = 0; i < n;) {
                if (strstr(msg + i, "IAXXX_CRASH_EVENT")) {
                    ALOGD("IAXXX_CRASH_EVENT received trying to get the crash logs");
                    dump_reg_access_hist_log();
                    dump_crash_log();
                }

                i += strlen(msg + i) + 1;
            }
        } else if (fds[1].revents & POLLIN) {
            read(fds[1].fd, &n, sizeof(n)); /* clear the socket */
            ALOGE("Interrupt received, exiting");
            break;
        } else {
            ALOGI("Message ignored");
        }
    }

exit:
    if (g_exit_socket[0] >= 0) {
        close(g_exit_socket[0]);
    }

    if (g_exit_socket[1] >= 0) {
        close(g_exit_socket[1]);
    }

    return err;
}
