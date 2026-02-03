#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>

typedef struct {
  uint32_t u_dc;        // Напряжение шины DC
  uint32_t u_gen_phA;   // Действующее значение напряжения на фазе А генератора
  uint32_t u_gen_phB;   // Действующее значение напряжения на фазе B генератора
  uint32_t u_gen_phC;   // Действующее значение напряжения на фазе С генератора
  uint32_t n_gen;       // Частота вращения ротора генератора
} params_t;

extern params_t g_params;

#endif /* PARAM_H */