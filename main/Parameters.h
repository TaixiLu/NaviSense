#pragma once

// Super parameters
#define SPD_SENSITIVITY_RATIO 5
#define SENSITIVITY_MAX_DIST 400 // cm
#define SENSITIVITY_MIN_DIST 100 // cm
#define ALARM_MAX_DIST (SENSITIVITY_MIN_DIST +   \
                        parameters.sensitivity * \
                            (SENSITIVITY_MAX_DIST - SENSITIVITY_MIN_DIST)) // cm
#define ALARM_MIN_DIST 30                                                  // cm

#define BEEPER_MIN_PWM 80
#define BEEPER_MAX_PWM 255

#define BEEPER_ON_TIME 100       // ms, time for each beep
#define BEEPER_OFF_MAX_TIME 1000 // ms, max time for gape between each beep
#define BEEPER_OFF_MIN_TIME 100  // ms, min time for gape between each beep

typedef struct
{
    float sensitivity = 50; // 0~100
    bool Alarm_sw = 1;
} Parameters;