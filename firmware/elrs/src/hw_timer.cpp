#include "hw_timer.h"
#include <Arduino.h>
#include "driver/timer.h"

#define TIMER_GROUP 0
#define TIMER_IDX   0

static void (*s_callback)() = nullptr;
static hw_timer_t* s_timer = nullptr;
static uint32_t s_intervalUs = ELRS_PACKET_INTERVAL_US;

void IRAM_ATTR HwTimerIsr() {
    if (s_callback) s_callback();
}

void HwTimerInit(void (*callback)()) {
    s_callback = callback;
    s_timer = timerBegin(TIMER_IDX, 80, true);  // 80 MHz / 80 = 1 us per tick
    timerAttachInterrupt(s_timer, &HwTimerIsr, true);
    timerAlarmWrite(s_timer, s_intervalUs, true);
}

void HwTimerStart() {
    if (s_timer) timerAlarmEnable(s_timer);
}

void HwTimerStop() {
    if (s_timer) timerAlarmDisable(s_timer);
}

void HwTimerSetIntervalUs(uint32_t us) {
    s_intervalUs = us;
    if (s_timer) {
        timerAlarmDisable(s_timer);
        timerAlarmWrite(s_timer, us, true);
        timerAlarmEnable(s_timer);
    }
}
