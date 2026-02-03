#include "sam3xa.h"
#include "init.h"

/* *****************************************************************
    INIT BLOCK
***************************************************************** */

void uart_init(void) {

    /* 2. Настроить PA10 (RXD0) и PA11 (TXD0) как периферию A */
    PIOA->PIO_PDR   = (1u << 10) | (1u << 11); // отключить PIO
    PIOA->PIO_ABSR &= ~((1u << 10) | (1u << 11)); // Peripheral A

    /* 3. Сброс и отключение USART */
    USART0->US_CR = US_CR_RSTRX | US_CR_RSTTX |
                    US_CR_RXDIS | US_CR_TXDIS;

    /* 4. Режим: асинхронный, 8N1, без паритета */
    USART0->US_MR =
        US_MR_USART_MODE_NORMAL |
        US_MR_USCLKS_MCK |
        US_MR_CHRL_8_BIT |
        US_MR_PAR_NO |
        US_MR_NBSTOP_1_BIT;

    /* 5. Baudrate = MCK / (16 * baud)
       MCK = 84 MHz → 115200 → CD = 45 */
    USART0->US_BRGR = 45;

    /* 6. Включить приём и передачу */
    USART0->US_CR = US_CR_RXEN | US_CR_TXEN;
}

void uart1_init(void)
{
    /* 2) PA12(TXD1) + PA13(RXD1) -> Peripheral A */
    PIOA->PIO_PDR   = (1u << 12) | (1u << 13);   // отключить PIO, отдать периферии
    PIOA->PIO_ABSR &= ~((1u << 12) | (1u << 13));/* 0 = Peripheral A */

    /* (опционально) подтяжка на RX */
    PIOA->PIO_PUER  = (1u << 13);

    /* 3) Сброс и отключение TX/RX */
    USART1->US_CR = US_CR_RSTRX | US_CR_RSTTX |
                    US_CR_RXDIS | US_CR_TXDIS;

    /* 4) Режим: async, MCK, 8N1 */
    USART1->US_MR =
        US_MR_USART_MODE_NORMAL |
        US_MR_USCLKS_MCK |
        US_MR_CHRL_8_BIT |
        US_MR_PAR_NO |
        US_MR_NBSTOP_1_BIT;

    /* 5) Baud = MCK/(16*CD). MCK=84MHz -> 115200 => CD ~ 45.57 -> 45 */
    USART1->US_BRGR = 45u;

    /* 6) Включить TX/RX */
    USART1->US_CR = US_CR_RXEN | US_CR_TXEN;
}

// Инициализация PB27
void gpio_init_out(void) {
  TEST_PIO->PIO_PER  = TEST_MASK;
  TEST_PIO->PIO_OER  = TEST_MASK;
  TEST_PIO->PIO_CODR = TEST_MASK;
}

// Инициализация PB26 для вывода сигнала синхронизации
void sync_out_init(void) {
  PMC->PMC_PCER0 = (1u << ID_PIOB);
  SYNC_OUT_PIO->PIO_PER = SYNC_OUT_MASK;
  SYNC_OUT_PIO->PIO_OER = SYNC_OUT_MASK;
  SYNC_OUT_PIO->PIO_CODR = SYNC_OUT_MASK; // low
}

void dacc_init(void) {
  // Включить тактирование DACC (ID_DACC в PMC_PCER1, т.к. >31)
  PMC->PMC_PCER1 = (1u << (ID_DACC - 32));

  // (Опционально) снять защиту записи DACC, если включена
  // В SAM3X у DACC есть WPMR. Для начала можно просто отключить WP:
  DACC->DACC_WPMR = 0x44414300; // "DAC", WPEN=0

  // Сброс
  DACC->DACC_CR = DACC_CR_SWRST;

  // Режим:
  // - TRGEN_DIS: обновляем вручную
  // - WORD_HALF: 12-bit
  // - TAG_EN: удобно писать в один регистр и выбирать канал
  DACC->DACC_MR =
      DACC_MR_TRGEN_DIS |
      DACC_MR_WORD_HALF |
      DACC_MR_TAG_EN |
      DACC_MR_STARTUP_8;

  // Разрешить каналы
  DACC->DACC_CHER = DACC_CHER_CH0 | DACC_CHER_CH1;
}
