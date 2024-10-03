#pragma once

// Super parameters
#define FULLSTOP_SPD_THRESHHOLD_MIN 0.005
#define FULLSTOP_SPD_THRESHHOLD_MAX 0.05
#define FULLSTOP_SPD_THRESHHOLD (FULLSTOP_SPD_THRESHHOLD_MIN +  \
                                 (1 - parameters.sensitivity) * \
                                     (FULLSTOP_SPD_THRESHHOLD_MAX - FULLSTOP_SPD_THRESHHOLD_MIN))
#define MAXSPD_THRESHHOLD 0.1
#define SENSITIVITY_MAX_DIST 5000 // mm
#define SENSITIVITY_MIN_DIST 300  // mm
#define ALARM_MAX_DIST (SENSITIVITY_MIN_DIST + curr_sensitivity * \
                                                   (SENSITIVITY_MAX_DIST - SENSITIVITY_MIN_DIST))
#define ALARM_MIN_DIST 300 // mm

#define BEEPER_MIN_PWM 80
#define BEEPER_MAX_PWM 255

#define BEEPER_ON_TIME 70        // ms, time for each beep
#define BEEPER_OFF_MAX_TIME 1000 // ms, max time for gape between each beep
#define BEEPER_OFF_MIN_TIME 70   // ms, min time for gape between each beep

typedef struct
{
    float sensitivity = 1; // 0~1
    bool Alarm_sw = 1;
} Parameters;

// ALSO See Speed_estimator.h