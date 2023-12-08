/* 
 * File:   config.g
 * Author: Paulo A P Hayashida, Rennan
 * Comments:
 * Revision history: 
 * Inserido define para o vetor cont, para facilitar a leitura
 */

#include "pwm_lib.h"
#include "Sensor_DS2.h"


#define sensor_press_1                   A0
#define sensor_press_2                   A1
#define sensor_press_3                   A2
#define sensor_press_4                   A3

#define ESC_C1_V1 2
#define ESC_C1_V2 3

#define ESC_C2_V1 11
#define ESC_C2_V2 12

#define ABS_C1_V1 7
#define ABS_C1_V2 4

#define ABS_C2_V1 6
#define ABS_C2_V2 5

#define ABS_C3_V1 8
#define ABS_C3_V2 13

#define ABS_C4_V1 9
#define ABS_C4_V2 10

#define DC_ABS_ISOLA 220
#define DC_ABS_ISOLA_NULL 0
#define DC_ABS_ISOLA_FULL 255
#define DC_ABS_ALIVIO 200
#define DC_ABS_ALIVIO_NULL 0

#define DC_ESC_HOLD 255
#define DC_ESC_ON 255
#define DC_ESC_OFF 0

#define DC_PUMP_ON 70 // [0...100]
#define DC_PUMP_OFF 0


#define PUMP                             42
#define PUMP_PWM_PERIOD_PIN_42           5000                         /* 20.000 kHz = 50 usecs em centenas de usecs (1e-8 secs) */
#define PUMP_PWM_DUTY_TO_PERIOD_COEF     (PUMP_PWM_PERIOD_PIN_42/100) /* Coeficiente para converter um Duty Cycle de [0..100] para o período equivalente */


 
