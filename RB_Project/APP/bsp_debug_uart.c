#include "bsp_debug_uart.h"

// 1. 定义全局errno（模拟POSIX错误码存储）
int errno;

// 2. 定义全局标志位（解决未定义问题）
volatile bool uart_send_complete_flag = false;
volatile bool uart_receive_complete_flag = false;

// 3. POSIX兼容函数实现（修复EBADF/struct stat报错）
__attribute__((weak)) int _isatty(int fd)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 1;

    errno = EBADF;
    return 0;
}

__attribute__((weak)) int _close(int fd)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 0;

    errno = EBADF;
    return -1;
}

__attribute__((weak)) int _lseek(int fd, int ptr, int dir)
{
    (void) fd;
    (void) ptr;
    (void) dir;

    errno = EBADF;
    return -1;
}

__attribute__((weak)) int _fstat(int fd, struct stat *st)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    {
        st->st_mode = S_IFCHR;  // 串口是字符设备
        return 0;
    }

    errno = EBADF;
    return -1;  // 修正返回值：失败应返回-1而非0
}

// 4. 调试串口UART初始化（修复函数名/控制器名称）
void Debug_UART_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    err = R_SCI_UART_Open(&Debug_UART_ctrl, &Debug_UART_cfg);
    assert(FSP_SUCCESS == err);  // 已补充assert.h，可正常使用
}

// 5. 串口中断回调（保留你的回显逻辑）
void debug_uart_callback(uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            // 串口回显：接收到的数据发回去
            R_SCI_UART_Write(&Debug_UART_ctrl, (uint8_t *)&(p_args->data), 1);
            uart_receive_complete_flag = true;  // 标记接收完成
            break;
        }
        case UART_EVENT_TX_COMPLETE:
        {
            uart_send_complete_flag = true;  // 标记发送完成
            break;
        }
        default:
            break;
    }
}

// 6. printf/scanf重定向（修复g_uart0_ctrl → g_uart4_ctrl）
#if defined __GNUC__ && !defined __clang__
// GCC编译器重定向
int _write(int fd, char *pBuffer, int size)
{
    (void) fd;
    R_SCI_UART_Write(&Debug_UART_ctrl, (uint8_t*)pBuffer, (uint32_t)size);
    while (uart_send_complete_flag == false);
    uart_send_complete_flag = false;

    return size;
}

int _read(int fd, char *pBuffer, int size)
{
    (void) fd;

    R_SCI_UART_Read(&Debug_UART_ctrl, (uint8_t*)pBuffer, (uint32_t)size);
    while (uart_receive_complete_flag == false);
    uart_receive_complete_flag = false;

    // 回显接收到的数据
    R_SCI_UART_Write(&Debug_UART_ctrl, (uint8_t*)pBuffer, (uint32_t)size);

    return size;
}

#else
// ARMCLANG编译器重定向（fputc）
int fputc(int ch, FILE *f)
{
    (void)f;
    R_SCI_UART_Write(&Debug_UART_ctrl, (uint8_t *)&ch, 1);
    while(uart_send_complete_flag == false);
    uart_send_complete_flag = false;

    return ch;
}
#endif
