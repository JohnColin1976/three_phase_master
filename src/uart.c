#include "sam3xa.h"
#include "uart.h"

static uint8_t uart0_parse_com_n_gen(const char *line, uint32_t *out)
{
    const char *p = line;
    uint64_t v = 0u;
    uint8_t seen = 0u;

    if (line == 0 || out == 0) {
        return 0u;
    }

    if (p[0] != 'C' || p[1] != 'O' || p[2] != 'M' || p[3] != '_' ||
        p[4] != 'N' || p[5] != '_' || p[6] != 'G' || p[7] != 'E' ||
        p[8] != 'N' || p[9] != '=')
    {
        return 0u;
    }

    p += 10;
    while (*p) {
        if (*p >= '0' && *p <= '9') {
            seen = 1u;
            v = v * 10u + (uint64_t)(*p - '0');
            if (v > 0xFFFFFFFFu) {
                v = 0xFFFFFFFFu;
                break;
            }
        } else {
            return 0u;
        }
        p++;
    }

    if (seen == 0u) {
        return 0u;
    }

    *out = (uint32_t)v;
    return 1u;
}

static inline void uart0_putc(char c)
{
    while ((USART0->US_CSR & US_CSR_TXRDY) == 0u) {}
    USART0->US_THR = (uint32_t)c;
}

static inline void uart1_putc(char c)
{
    while ((USART1->US_CSR & US_CSR_TXRDY) == 0u) {}
    USART1->US_THR = (uint32_t)c;
}

static inline void uart0_puts(const char *s)
{
    while (*s) {
        uart0_putc(*s++);
    }
}

static inline void uart1_puts(const char *s)
{
    while (*s) {
        uart1_putc(*s++);
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

uint8_t uart0_poll_com_n_gen(void)
{
    static char line[32];
    static uint8_t idx = 0u;
    uint8_t updated = 0u;

    while ((USART0->US_CSR & US_CSR_RXRDY) != 0u) {
        char c = (char)(USART0->US_RHR & 0xFFu);

        if (c == '\r' || c == '\n' || c == ';') {
            if (idx > 0u) {
                line[idx] = '\0';
                if (uart0_parse_com_n_gen(line, &g_params_in.com_n_gen)) {
                    updated = 1u;
                    uart0_puts("MASTER_OK\r\n");
                    uart1_puts(line);
                    uart1_puts("\r\n");
                } else {
                    uart0_puts("MASTER_ERR\r\n");
                }
                idx = 0u;
            }
            continue;
        }

        if (idx < (uint8_t)(sizeof(line) - 1u)) {
            line[idx++] = c;
        } else {
            idx = 0u; // переполнение буфера — сброс строки
        }
    }

    return updated;
}

void uart1_forward_rx_to_uart0(void)
{
    while ((USART1->US_CSR & US_CSR_RXRDY) != 0u) {
        char c = (char)(USART1->US_RHR & 0xFFu);
        uart0_putc(c);
    }
}
