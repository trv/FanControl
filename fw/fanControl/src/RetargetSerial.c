
#include "RetargetSerial.h"

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>

#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32F0xx_usart.h"


/*  ----------------------------------------------------------------------------
    Static Assertions
    ------------------------------------------------------------------------- */


/*  ----------------------------------------------------------------------------
    Private Constant Definitions
    ------------------------------------------------------------------------- */


/*  ----------------------------------------------------------------------------
    Private Data Structure Declarations
    ------------------------------------------------------------------------- */


/*  ----------------------------------------------------------------------------
    Private Function Declarations
    ------------------------------------------------------------------------- */


/*  ----------------------------------------------------------------------------
    Private Data Definitions
    ------------------------------------------------------------------------- */


/*  ----------------------------------------------------------------------------
    API Function Definitions
    ------------------------------------------------------------------------- */
void RetargetSerial_Init()
{
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    // USART1:
    // UART_TX: PA9
    // UART_RX: PA10

    USART_InitTypeDef   initUSART = {
        .USART_BaudRate = 115200,
        .USART_WordLength = USART_WordLength_8b,
        .USART_StopBits = USART_StopBits_1,
        .USART_Parity = USART_Parity_No,
        .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None
    };
    GPIO_InitTypeDef    initGPIO = {
        .GPIO_Mode  = GPIO_Mode_AF,
        .GPIO_Speed = GPIO_Speed_Level_1,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd  = GPIO_PuPd_UP
    };

    // enable peripheral clocks and select USART1 alternate function mode
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    // configure UART pins (PA9 & PA10)
    initGPIO.GPIO_Pin = GPIO_Pin_9,
    GPIO_Init(GPIOA, &initGPIO);
    initGPIO.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &initGPIO);

    // select UART alternate function
    GPIO_PinAFConfig(GPIOA, 9, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, 10, GPIO_AF_1);

    USART_Init(USART1, &initUSART);
    USART_Cmd(USART1, ENABLE);
}

/*  ----------------------------------------------------------------------------
    Private Function Definitions
    ------------------------------------------------------------------------- */

int _close(int);
int _fstat(int, struct stat*);
int _isatty(int);
off_t _lseek(int, off_t, int);
int _read(int, char*, size_t);
int _write(int, const char*, size_t);


int _read(int __fd, char *__buf, size_t __nbyte)
{
    (void) __fd;
    uint8_t c;
    if (__nbyte > 0 && SET == USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) {
        c = USART_ReceiveData(USART1);
        __buf[0] = c;
        return 1;
    } else {
        errno = EAGAIN;
        return -1;
    }
}

int _write(int __fd, const char *__buf, size_t __nbyte)
{
    uint32_t err = 0;
    (void) __fd;
    for (size_t i=0; i < __nbyte; i++) {
        while (RESET == USART_GetFlagStatus(USART1, USART_FLAG_TXE));
        USART_SendData(USART1, __buf[i]);
    }
    (void) err;
    return __nbyte;
}

/* Stub other functions from stdio.h */

int _close(int __fildes)
{
    (void) __fildes;
    return 0;
}

int _fstat(int fildes, struct stat *buf)
{
    (void) fildes;
    (void) *buf;
    return 0;
}

int _isatty(int fildes)
{
    (void) fildes;
    return 1;
}

off_t _lseek(int fildes, off_t offset, int whence)
{
    (void) fildes;
    (void) offset;
    (void) whence;
    return 0;
}
