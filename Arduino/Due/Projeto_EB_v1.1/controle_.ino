#include "config.h"

void controle_ESC_circuito_1(int pwm_esc){
  analogWrite(ESC_C1_V1,pwm_esc);
  analogWrite(ESC_C1_V2,pwm_esc);
}

void controle_ESC_circuito_2(int pwm_esc){
  digitalWrite(ESC_C2_V1,pwm_esc);
  digitalWrite(ESC_C2_V2,pwm_esc);
}

float controle_roda_1(boolean enable_control, float p_1, float p_ref) {
  float erro = 0;
  if (enable_control == 1) {
    erro = p_ref - p_1;
    if (erro < -15) {
      analogWrite(ABS_C1_V1, DC_ABS_ISOLA_FULL);
      analogWrite(ABS_C1_V2, DC_ABS_ALIVIO);
    }
    else if (erro < 5 && erro > - 5) {
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
  return erro;
}

void controle_roda_2(boolean enable_control, float p_2, float p_ref) {
  if (enable_control == 1) {
    if (p_2 > p_ref* 1.02) {
      analogWrite(ABS_C2_V1, 255);
      analogWrite(ABS_C2_V2, DC_ABS_ALIVIO);
    }
    else if (p_2 > p_ref * 0.98 && p_2 < p_ref * 1.02) {
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
    if (p_3 > p_ref* 1.02) {
      analogWrite(ABS_C3_V1, 255);
      analogWrite(ABS_C3_V2, DC_ABS_ALIVIO);
    }
    else if ( p_3 > (p_ref * 0.98) && p_3 < (p_ref * 1.02)) {
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
    if (p_4 > p_ref* 1.02) {
      analogWrite(ABS_C4_V1, 255);
      analogWrite(ABS_C4_V2, DC_ABS_ALIVIO);
    }
    else if (p_4 > p_ref * 0.98 && p_4 < p_ref * 1.02) {
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
