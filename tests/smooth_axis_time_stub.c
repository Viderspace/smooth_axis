//
// Created by Jonatan Vider on 30/11/2025.
//

// tests/smooth_axis_time_stub.c
#include <stdint.h>
#include "smooth_axis.h"

// Minimal stub for tests: AUTO_DT isn't used here, but the
// symbol must exist for the linker. We just return 0.
uint32_t smooth_axis_now_ms(void) {
  return 0;
}