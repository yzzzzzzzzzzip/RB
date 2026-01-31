#ifndef __BSP_DEBUG_UART_H
#define __BSP_DEBUG_UART_H

#include "hal_data.h"
#include "stdio.h"
#include <assert.h>  // 补充assert头文件
#include <stdint.h>  // 补充标准类型头文件

// 1. 补充struct stat结构体定义（解决st->st_mode报错）
#ifndef _STRUCT_STAT_DEFINED
#define _STRUCT_STAT_DEFINED
struct stat {
    uint32_t st_mode;  // 仅保留用到的字段，兼容编译
};
#endif

// 2. 补充POSIX常量定义（解决EBADF/STDIN_FILENO等未定义）
#ifndef EBADF
#define EBADF       9   // 无效文件描述符（POSIX标准错误码）
#endif

#ifndef S_IFMT
#define S_IFMT      0170000 // 文件类型掩码
#define S_IFREG     0100000 // 普通文件
#define S_IFCHR     0020000 // 字符设备（串口）
#define S_IRUSR     00400   // 所有者读
#define S_IWUSR     00200   // 所有者写
#define S_IXUSR     00100   // 所有者执行
#endif

#ifndef STDIN_FILENO
#define STDIN_FILENO  0  // 标准输入
#define STDOUT_FILENO 1  // 标准输出
#define STDERR_FILENO 2  // 标准错误
#endif

// 3. 补充errno定义（Keil默认无全局errno，模拟实现）
#ifndef errno
extern int errno;
#endif

// 4. 修正函数声明（与实现一致：UART4而非UART0）
void Debug_UART_Init(void);

// 5. 声明POSIX兼容函数（防止编译警告）
int _isatty(int fd);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _fstat(int fd, struct stat *st);

// 6. 声明全局标志位（解决未定义问题）
extern volatile bool uart_send_complete_flag;
extern volatile bool uart_receive_complete_flag;

#endif // __BSP_DEBUG_UART_H
