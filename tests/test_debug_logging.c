////
//// Created by Jonatan Vider on 07/12/2025.
////
//
//#include <stdio.h>
//
//// Enable debug logging for this test
//#define SMOOTH_AXIS_DEBUG_ENABLE 1
//#include "smooth_axis.h"
//
//uint32_t test_timer(void) {
//    static uint32_t time_ms = 0;
//    time_ms += 16;  // Simulate 16ms per call
//    return time_ms;
//}
//
//void test_debug_logging(void) {
//    printf("\n=== Testing Debug Logging ===\n");
//    printf("(You should see debug messages below if logging is enabled)\n\n");
//
//    smooth_axis_config_t cfg;
//    smooth_axis_t axis;
//
//    // Should log init
//    smooth_axis_default_config_auto_dt(&cfg, 1023, 0.25f, test_timer);
//    smooth_axis_init(&axis, &cfg);
//
//    // Should log first sample
//    smooth_axis_update_auto_dt(&axis, 512);
//
//    // Run warmup (should log completion after ~4096 cycles)
//    printf("\nRunning warmup cycles...\n");
//    for (int i = 0; i < 4100; i++) {
//        smooth_axis_update_auto_dt(&axis, 512);
//    }
//
//    // Add some noise to trigger noise logging
//    printf("\nAdding noise...\n");
//    for (int i = 0; i < 100; i++) {
//        uint16_t noisy = 512 + (i % 2 ? 50 : -50);
//        smooth_axis_update_auto_dt(&axis, noisy);
//    }
//
//    // Large movement to trigger new value
//    printf("\nLarge movement...\n");
//    for (int i = 0; i < 100; i++) {
//        smooth_axis_update_auto_dt(&axis, 900);
//        if (smooth_axis_has_new_value(&axis)) {
//            // Should log new value
//        }
//    }
//
//    printf("\n=== Debug Logging Test Complete ===\n");
//}
//
//
//// note: All logging compiles to nothing unless      SMOOTH_AXIS_DEBUG_ENABLE=1
//int main(void) {
//    test_debug_logging();
//    return 0;
//}