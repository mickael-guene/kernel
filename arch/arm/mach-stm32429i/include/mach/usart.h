#ifndef _MACH_STM32429I_UART_H_
#define _MACH_STM32429I_UART_H_

void stm32429i_uart_init(void);

/* registers */
#define USART_SR    0x00
#define USART_DR    0x04
#define USART_BRR   0x08
#define USART_CR1   0x0C
#define USART_CR2   0x10
#define USART_CR3   0x14
#define USART_GTPR  0x18

/* bit mask */
#define USART_SR_RXNE       (1<<5)
#define USART_SR_TXE        (1<<7)

#define USART_CR1_RE        (1<<2)
#define USART_CR1_TE        (1<<3)
#define USART_CR1_RXNEIE    (1<<5)
#define USART_CR1_TXEIE     (1<<7)
#define USART_CR1_UE        (1<<13)

#endif

