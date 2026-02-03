#include "sam3xa.h"
#include "uart.h"

static inline void uart0_putc(char c)
{
    while ((USART0->US_CSR & US_CSR_TXRDY) == 0u) {}
    USART0->US_THR = (uint32_t)c;
}

static inline void uart0_puts(const char *s)
{
    while (*s) {
        uart0_putc(*s++);
    }
}

/* u32 -> ASCII (десятичное) без sprintf.
   Возвращает количество выведенных символов. */
static inline uint32_t uart0_put_u32(uint32_t v)
{
    char buf[10];                // максимум 4294967295 (10 цифр)
    uint32_t n = 0;

    if (v == 0u) {
        uart0_putc('0');
        return 1u;
    }

    while (v != 0u) {
        uint32_t q = v / 10u;
        uint32_t r = v - q * 10u;
        buf[n++] = (char)('0' + r);
        v = q;
    }

    while (n) {
        uart0_putc(buf[--n]);
    }
    return 0u; // значение не нужно, оставлено для совместимости/расширения
}

void uart0_send_params(const params_t *params)
{
    if (params == 0) {
        return;
    }

    uart0_puts("UDC=");
    uart0_put_u32(params->u_dc);
    uart0_puts("\r\n");
}
