// startup_sam3x8e.c — SES-compatible startup for ATSAM3X8E (Cortex-M3)
//
// Features:
// - Full vector table (core + SAM3X IRQ0..44) with weak aliases (override in user code)
// - .data copy from Flash to RAM, .bss zero
// - SystemInit() call
// - SCB->VTOR points to vector table (stable across toolchains/IDEs)
// - __libc_init_array() (newlib init like SES)
// - Diagnostic Default_Handler + HardFault_Handler (no naked asm => no linker Thumb issues)
//
// Notes:
// - Device handler names must be ..._Handler (SAM3X CMSIS convention), e.g. TC0_Handler.
// - Do NOT read peripheral status regs like TC_SR in Watch window (ACK side effects).

#include <stdint.h>
#include "sam3xa.h"   // CMSIS for SAM3X: SCB, IRQn, SystemInit(), etc.

extern uint32_t _estack;

// Linker-provided section symbols
extern uint32_t _sidata;  // start of init values for .data in FLASH
extern uint32_t _sdata;   // start of .data in RAM
extern uint32_t _edata;   // end of .data in RAM
extern uint32_t _sbss;    // start of .bss in RAM
extern uint32_t _ebss;    // end of .bss in RAM

int main(void);
extern void __libc_init_array(void);

// Forward declarations
void Reset_Handler(void);
void Default_Handler(void);
void HardFault_Handler(void);

// ---------------------------
// Core exception handlers (weak)
// ---------------------------
void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)    __attribute__((weak, alias("Default_Handler")));

// ---------------------------
// Device IRQ handlers (weak aliases like SES)
// ---------------------------
void SUPC_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void RSTC_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void RTC_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void RTT_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void WDT_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void PMC_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void EFC0_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void EFC1_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void UART_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SMC_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void SDRAMC_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void PIOA_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void PIOB_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void PIOC_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void PIOD_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void PIOE_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void PIOF_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void USART0_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void USART1_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void USART2_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void USART3_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void HSMCI_Handler(void)    __attribute__((weak, alias("Default_Handler")));
void TWI0_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void TWI1_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SPI0_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SPI1_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SSC_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void TC0_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void TC1_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void TC2_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void TC3_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void TC4_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void TC5_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void TC6_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void TC7_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void TC8_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void PWM_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void ADC_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void DACC_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void DMAC_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void UOTGHS_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void TRNG_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void EMAC_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void CAN0_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void CAN1_Handler(void)     __attribute__((weak, alias("Default_Handler")));

// ---------------------------
// Vector table
// ---------------------------
__attribute__((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
  (void (*)(void))(&_estack), // 0: Initial Stack Pointer
  Reset_Handler,              // 1: Reset
  NMI_Handler,                // 2: NMI
  HardFault_Handler,          // 3: HardFault
  MemManage_Handler,          // 4: MemManage
  BusFault_Handler,           // 5: BusFault
  UsageFault_Handler,         // 6: UsageFault
  0, 0, 0, 0,                 // 7..10: Reserved
  SVC_Handler,                // 11: SVCall
  DebugMon_Handler,           // 12: Debug Monitor
  0,                          // 13: Reserved
  PendSV_Handler,             // 14: PendSV
  SysTick_Handler,            // 15: SysTick

  // SAM3X IRQ0..44 (per datasheet / CMSIS device header order)
  SUPC_Handler,     // 0
  RSTC_Handler,     // 1
  RTC_Handler,      // 2
  RTT_Handler,      // 3
  WDT_Handler,      // 4
  PMC_Handler,      // 5
  EFC0_Handler,     // 6
  EFC1_Handler,     // 7
  UART_Handler,     // 8
  SMC_Handler,      // 9
  SDRAMC_Handler,   // 10
  PIOA_Handler,     // 11
  PIOB_Handler,     // 12
  PIOC_Handler,     // 13
  PIOD_Handler,     // 14
  PIOE_Handler,     // 15
  PIOF_Handler,     // 16
  USART0_Handler,   // 17
  USART1_Handler,   // 18
  USART2_Handler,   // 19
  USART3_Handler,   // 20
  HSMCI_Handler,    // 21
  TWI0_Handler,     // 22
  TWI1_Handler,     // 23
  SPI0_Handler,     // 24
  SPI1_Handler,     // 25
  SSC_Handler,      // 26
  TC0_Handler,      // 27  (ID_TC0 = 27)
  TC1_Handler,      // 28
  TC2_Handler,      // 29
  TC3_Handler,      // 30
  TC4_Handler,      // 31
  TC5_Handler,      // 32
  TC6_Handler,      // 33
  TC7_Handler,      // 34
  TC8_Handler,      // 35
  PWM_Handler,      // 36
  ADC_Handler,      // 37
  DACC_Handler,     // 38
  DMAC_Handler,     // 39
  UOTGHS_Handler,   // 40
  TRNG_Handler,     // 41
  EMAC_Handler,     // 42
  CAN0_Handler,     // 43
  CAN1_Handler      // 44
};

// ---------------------------
// Handlers
// ---------------------------
void Default_Handler(void) {
  // Core/system status (helpful if you get here by wrong IRQ vector)
  volatile uint32_t icsr  = SCB->ICSR;   // VECTACTIVE/VECTPENDING
  volatile uint32_t cfsr  = SCB->CFSR;
  volatile uint32_t hfsr  = SCB->HFSR;
  volatile uint32_t mmfar = SCB->MMFAR;
  volatile uint32_t bfar  = SCB->BFAR;
  (void)icsr; (void)cfsr; (void)hfsr; (void)mmfar; (void)bfar;
  while (1) {}
}

void HardFault_Handler(void) {
  // Keep it pure-C to avoid Thumb/relocation issues across toolchains.
  // If you need stacked registers later, we can add a small .S helper.
  volatile uint32_t icsr  = SCB->ICSR;
  volatile uint32_t cfsr  = SCB->CFSR;
  volatile uint32_t hfsr  = SCB->HFSR;
  volatile uint32_t mmfar = SCB->MMFAR;
  volatile uint32_t bfar  = SCB->BFAR;
  (void)icsr; (void)cfsr; (void)hfsr; (void)mmfar; (void)bfar;
  while (1) {}
}

// ---------------------------
// Reset handler (SES-like init sequence)
// ---------------------------
void Reset_Handler(void) {
  // 1) Copy .data from Flash to RAM
  uint32_t *src = &_sidata;
  uint32_t *dst = &_sdata;
  while (dst < &_edata) {
    *dst++ = *src++;
  }

  // 2) Zero .bss
  dst = &_sbss;
  while (dst < &_ebss) {
    *dst++ = 0;
  }

  // 3) CMSIS system init (clocks)
  SystemInit();

  // 4) Vector table base (important if any remap/boot ROM differences)
  SCB->VTOR = (uint32_t)g_pfnVectors;

  // 5) C runtime init (newlib) — constructors/init arrays like SES
  __libc_init_array();

  // 6) main
  (void)main();

  // If main returns
  while (1) {}
}
