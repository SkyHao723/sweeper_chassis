/*
 * writets.c —— 用一个极小的 LD_PRELOAD 钩子, 记录厂商节点把 11 字节命令帧
 *              写到串口的精确时刻 (CLOCK_REALTIME, µs 级) 和帧内容。
 *
 * 为什么需要它: 主机上没装 strace (也不想装)。钩 write() 比 ptrace 开销更小,
 * 而且时间戳由本进程自己打, 不受调度噪声影响。
 *
 * 为什么要记内容: 厂商源码里 Stm32_Serial.write(Send_Data.tx, 11) 有 9 处调用点,
 *   其中 L98-109 的 LED 设置帧同样以 0x7B 开头、同样 11 字节 —— 只按
 *   "11 字节 + 0x7B" 筛会把 LED 帧当成命令帧, 配对就错位了。记下 payload 后,
 *   分析脚本可以从 [3..4]/[7..8] 解出 (vx,wz) 按内容配对, 精确区分。
 *
 * 编译:
 *   gcc -shared -fPIC -O2 -o /tmp/writets.so /tmp/writets.c -ldl
 * 运行:
 *   LD_PRELOAD=/tmp/writets.so WRITETS_LOG=/tmp/writets.log <厂商节点> --ros-args ...
 * 输出 (每行):
 *   <epoch秒>.<微秒> <fd> <字节数> <payload十六进制,小写>
 *   例: 1698451234.567890 3 11 7b0000 0058 0000 0000 09 7d
 *
 * 判据: write(fd, buf, 11) 且 buf[0] == 0x7B
 *       (帧头 0x7B, [1]AutoRecharge, [2]SecurityPLY, [3..4]vx*1000 大端,
 *        [5..6]vy*1000, [7..8]wz*1000, [9]BCC, [10]0x7D)
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <unistd.h>

static ssize_t (*real_write)(int, const void *, size_t) = NULL;
static ssize_t (*real_writev)(int, const struct iovec *, int) = NULL;
static FILE *logfp = NULL;
static __thread int busy = 0;

static void init_hook(void)
{
    if (!real_write)
        real_write = (ssize_t (*)(int, const void *, size_t))dlsym(RTLD_NEXT, "write");
    if (!real_writev)
        real_writev = (ssize_t (*)(int, const struct iovec *, int))dlsym(RTLD_NEXT, "writev");
    if (!logfp) {
        const char *p = getenv("WRITETS_LOG");
        logfp = fopen(p ? p : "/tmp/writets.log", "a");
        if (logfp)
            setvbuf(logfp, NULL, _IOLBF, 0);
    }
}

static void note(int fd, const unsigned char *p, size_t count)
{
    struct timeval tv;
    size_t i;
    gettimeofday(&tv, NULL);
    fprintf(logfp, "%ld.%06ld %d %zu ",
            (long)tv.tv_sec, (long)tv.tv_usec, fd, count);
    for (i = 0; i < count && i < 32; i++)
        fprintf(logfp, "%02x", p[i]);
    fprintf(logfp, "\n");
    fflush(logfp);
}

ssize_t write(int fd, const void *buf, size_t count)
{
    init_hook();
    if (logfp && !busy && count == 11 && buf &&
        ((const unsigned char *)buf)[0] == 0x7B) {
        busy = 1;
        note(fd, (const unsigned char *)buf, count);
        busy = 0;
    }
    return real_write(fd, buf, count);
}

ssize_t writev(int fd, const struct iovec *iov, int iovcnt)
{
    init_hook();
    if (logfp && !busy && iov && iovcnt > 0) {
        size_t total = 0;
        unsigned char flat[11];
        size_t n = 0;
        int i;
        for (i = 0; i < iovcnt; i++)
            total += iov[i].iov_len;
        if (total == 11) {
            for (i = 0; i < iovcnt && n < total; i++) {
                size_t k = iov[i].iov_len;
                if (k > total - n)
                    k = total - n;
                memcpy(flat + n, iov[i].iov_base, k);
                n += k;
            }
            if (n == 11 && flat[0] == 0x7B) {
                busy = 1;
                note(fd, flat, 11);
                busy = 0;
            }
        }
    }
    return real_writev(fd, iov, iovcnt);
}
