#ifndef INIT_H
#define INIT_H

#include "sam3xa.h"   // чтобы PIOB / Pio были видимы везде, где включили init.h

// Конфигурация под сигнальный PB27 - для вывода на светодиод
// событий, в том числе и за счет различной последовательности
// морганий
#define TEST_PIO      PIOB
#define TEST_PIN      27u
#define TEST_MASK     (1u << TEST_PIN)

// Конфиг пинов SYNC_OUT (пример: PB26) + флаг SYNC_PIO
#define SYNC_OUT_PIO      PIOB
#define SYNC_OUT_PIN      26u
#define SYNC_OUT_MASK     (1u << SYNC_OUT_PIN)

// Сдвиги фаз на 120° и 240° в 32-битной фазе (2^32 * 120/360, 2^32 * 240/360):
#define PHASE_120 0x55555555u  // +120°
#define PHASE_240 0xAAAAAAAau  // +240°


void uart_init(void);
void uart1_init(void);
void gpio_init_out(void);
void sync_out_init(void);
void dacc_init(void);

#endif /* INIT_H */
