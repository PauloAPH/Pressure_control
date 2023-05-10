#include "config.h"

void controle_ESC_circuito_1(int pwm_esc){
  analogWrite(ESC_C1_V1,pwm_esc);
  analogWrite(ESC_C1_V2,pwm_esc);
}

void controle_ESC_circuito_2(int pwm_esc){
  digitalWrite(ESC_C2_V1,pwm_esc);
  digitalWrite(ESC_C2_V2,pwm_esc);
}

void controle_roda_1(boolean enable_control, unsigned int p_1, unsigned int p_ref) {
  if (enable_control == 1) {
    if (p_1 > p_ref) {
      analogWrite(ABS_C1_V1, 255);
      analogWrite(ABS_C1_V2, 0);
    }
    else if (p_1 > p_ref * 0.95 && p_1 < p_ref * 1.05) {
      analogWrite(ABS_C1_V1, DC_ABS_ISOLA_FULL);
      analogWrite(ABS_C1_V2, DC_ABS_ALIVIO_NULL);
    }
    else {
      analogWrite(ABS_C1_V1, 0);
      analogWrite(ABS_C1_V2, 0);
    }
  }
  else {
    analogWrite(ABS_C1_V1, DC_ABS_ISOLA_NULL);
    analogWrite(ABS_C1_V2, DC_ABS_ALIVIO_NULL);
  }
}

void controle_roda_2(boolean enable_control, float p_2, float p_ref) {
  if (enable_control == 1) {
    if (p_2 > p_ref) {
      analogWrite(ABS_C2_V1, 255);
      analogWrite(ABS_C2_V2, 0);
    }
    else if (p_2 > p_ref * 0.95 && p_2 < p_ref * 1.05) {
      analogWrite(ABS_C2_V1, DC_ABS_ISOLA_FULL);
      analogWrite(ABS_C2_V2, DC_ABS_ALIVIO_NULL);
    }
    else {
      analogWrite(ABS_C2_V1, 0);
      analogWrite(ABS_C2_V2, 0);
    }
  }
  else {
    analogWrite(ABS_C2_V1, DC_ABS_ISOLA_NULL);
    analogWrite(ABS_C2_V2, DC_ABS_ALIVIO_NULL);
  }
}

void controle_roda_3(boolean enable_control, float p_3, float p_ref) {
  if (enable_control == 1) {
    if (p_3 > p_ref) {
      analogWrite(ABS_C3_V1, 255);
      analogWrite(ABS_C3_V2, 0);
    }
    else if ( p_3 > (p_ref * 0.95) && p_3 < (p_ref * 1.05)) {
      analogWrite(ABS_C3_V1, DC_ABS_ISOLA_FULL);
      analogWrite(ABS_C3_V2, DC_ABS_ALIVIO_NULL);
    }
    else {
      analogWrite(ABS_C3_V1, 0);
      analogWrite(ABS_C3_V2, 0);
    }
  }
  else {
    analogWrite(ABS_C3_V1, DC_ABS_ISOLA_NULL);
    analogWrite(ABS_C3_V2, DC_ABS_ALIVIO_NULL);
  }
}

void controle_roda_4(boolean enable_control, float p_4, float p_ref) {
  if (enable_control == 1) {
    if (p_4 > p_ref) {
      analogWrite(ABS_C4_V1, 255);
      analogWrite(ABS_C4_V2, 0);
    }
    else if (p_4 > p_ref * 0.9 && p_4 < p_ref * 1.1) {
      analogWrite(ABS_C4_V1, DC_ABS_ISOLA_FULL);
      analogWrite(ABS_C4_V2, DC_ABS_ALIVIO_NULL);
    }
    else {
      analogWrite(ABS_C4_V1, 0);
      analogWrite(ABS_C4_V2, 0);
    }
  }
  else {
    analogWrite(ABS_C4_V1, DC_ABS_ISOLA_NULL);
    analogWrite(ABS_C4_V2, DC_ABS_ALIVIO_NULL);
  }
}
