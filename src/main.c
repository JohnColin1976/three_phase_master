#include "sam3xa.h"

#include "sin_lut_1024.h"

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

// Настройка сигнала синуса
static volatile uint32_t phase = 0;
static volatile uint32_t phase_inc = 0;   // задаём частоту
static volatile uint16_t amp = 2047;      // 0..2047 (масштаб амплитуды)

// Настройка синхронизации
static volatile uint32_t sync_div = 100; // 100kHz / 100 = 1kHz
static volatile uint32_t sync_cnt = 0;

// Сдвиги фаз на 120° и 240° в 32-битной фазе:
#define PHASE_120 0x55555555u
#define PHASE_240 0xAAAAAAAau


// Генерация синусоидальных значений
static inline uint16_t synth_u12(uint32_t ph) {
  // 1024 точки => 10 бит индекса из старших бит фазы
  uint32_t idx = ph >> 22; // 32 - 10
  int32_t s = (int32_t)sinLUT_1024[idx] - 2048;
  s = (s * (int32_t)amp) >> 11;
  return (uint16_t)(s + 2048);
}

static inline void dds_set_freq_hz(uint32_t f_hz) {
  // phase_inc = f_out * 2^32 / f_update
  // f_update = 100000 (у нас RC=420 на MCK/2=42MHz)
  const uint32_t f_update = 100000u;
  phase_inc = (uint32_t)(((uint64_t)f_hz << 32) / f_update);
}


// Инициализация PB27
static inline void gpio_init_out(void) {
  TEST_PIO->PIO_PER  = TEST_MASK;
  TEST_PIO->PIO_OER  = TEST_MASK;
  TEST_PIO->PIO_CODR = TEST_MASK;
}

// Инициализация PB26 для вывода сигнала синхронизации
static inline void sync_out_init(void) {
  PMC->PMC_PCER0 = (1u << ID_PIOB);
  SYNC_OUT_PIO->PIO_PER = SYNC_OUT_MASK;
  SYNC_OUT_PIO->PIO_OER = SYNC_OUT_MASK;
  SYNC_OUT_PIO->PIO_CODR = SYNC_OUT_MASK; // low
}

// Формирование импульса синхронизации
static inline void sync_pulse(void) {
  SYNC_OUT_PIO->PIO_SODR = SYNC_OUT_MASK;
  __NOP(); __NOP(); __NOP(); __NOP(); __NOP();  // подберёшь длительность
  SYNC_OUT_PIO->PIO_CODR = SYNC_OUT_MASK;
}

//Переключение PB27
static inline void toggle_pin(void) {
  if (TEST_PIO->PIO_ODSR & TEST_MASK) TEST_PIO->PIO_CODR = TEST_MASK;
  else                                TEST_PIO->PIO_SODR = TEST_MASK;
}

// Все что ниже нужно для синхронизации
static volatile uint8_t sync_hi = 0;

static inline void sync_set_hi(void){ PIOB->PIO_SODR = (1u<<26); }
static inline void sync_set_lo(void){ PIOB->PIO_CODR = (1u<<26); }



void TC0_Handler(void) __attribute__((used));
void TC0_Handler(void) {
  TcChannel *tc = &TC0->TC_CHANNEL[0];
  uint32_t sr = tc->TC_SR;           // ACK

  if (sr & TC_SR_CPCS) {
    // Синхронизация
    if (sync_hi) { sync_hi = 0; sync_set_lo(); }

    uint32_t prev = phase; // Сохраняем предыдущую фазу
    phase += phase_inc; // Вычисляем новую фазу

    uint16_t va = synth_u12(phase); // Получаем точку для фазы А
    uint16_t vb = synth_u12(phase + PHASE_120); // Получаем точку для фазы Б

    // DAC0 = A
    while ((DACC->DACC_ISR & DACC_ISR_TXRDY) == 0) {}
    DACC->DACC_CDR = (0u << 12) | (va & 0x0FFF);

    // DAC1 = B (+120°)
    while ((DACC->DACC_ISR & DACC_ISR_TXRDY) == 0) {}
    DACC->DACC_CDR = (1u << 12) | (vb & 0x0FFF);

    // Синхронизация
    if (phase < prev) {          // overflow => начало периода
      sync_set_hi();
      sync_hi = 1;               // держим 1 тик (10 мкс)
    }
  }
}

static inline void dacc_init(void) {
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

static inline void dacc_write_ch(uint8_t ch, uint16_t v12) {
  while ((DACC->DACC_ISR & DACC_ISR_TXRDY) == 0) { }
  DACC->DACC_CDR = ((uint32_t)ch << 12) | (v12 & 0x0FFF);
}



int main(void) {
  // 1) снять защиту PMC и включить тактирование
  PMC->PMC_WPMR = 0x504D4300;                  // WPKEY="PMC", WPEN=0
  PMC->PMC_PCER0 = (1u << ID_PIOB) | (1u << ID_TC0);

  SystemCoreClockUpdate();

  sync_out_init();

  dacc_init();

  amp = 1800;          // амплитуда (подбери)
  dds_set_freq_hz(50); // старт: 50 Гц для проверки

  // 2) gpio
  gpio_init_out();

  // 3) снять защиту TC0
  TC0->TC_WPMR = 0x54430000;                   // WPKEY="TC", WPEN=0

  TcChannel *tc = &TC0->TC_CHANNEL[0];

  // 4) стоп + сброс IRQ/статуса
  tc->TC_CCR = TC_CCR_CLKDIS;
  tc->TC_IDR = 0xFFFFFFFFu;
  (void)tc->TC_SR;

  // 5) Waveform, UP to RC, clock = MCK/2
  tc->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |
               TC_CMR_WAVE |
               TC_CMR_WAVSEL_UP_RC;

  // 84MHz/2 = 42MHz; 42MHz/420 = 100kHz
  tc->TC_RC  = 420;

  /* 7) Разрешить IRQ по RC compare */
  tc->TC_IER = TC_IER_CPCS;
  

  /* 8) NVIC */
  NVIC_DisableIRQ(TC0_IRQn);
  NVIC_ClearPendingIRQ(TC0_IRQn);
  NVIC_SetPriority(TC0_IRQn, 0);   // высокий приоритет
  NVIC_EnableIRQ(TC0_IRQn);



  // 6) старт
  tc->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

  //volatile uint32_t cv_test;

  while (1) {
    //cv_test = TC0->TC_CHANNEL[0].TC_CV;
    //__WFI();
  }
}
