#include "config.h"

void gotFrameMB0(CAN_FRAME *frame) {
  uint16_t  pressaoReferenciaRoda1Raw = frame->data.s0;
  uint16_t  pressaoReferenciaRoda2Raw = frame->data.s1;
  uint16_t  pressaoReferenciaRoda3Raw = frame->data.s2;
  uint16_t  pressaoReferenciaRoda4Raw = frame->data.s3;


  float pressaoRefenciaRoda1 = pressaoReferenciaRoda1Raw * FACTOR_PRESSURE;
  float pressaoRefenciaRoda2 = pressaoReferenciaRoda2Raw * FACTOR_PRESSURE;
  float pressaoRefenciaRoda3 = pressaoReferenciaRoda3Raw * FACTOR_PRESSURE;
  float pressaoRefenciaRoda4 = pressaoReferenciaRoda4Raw * FACTOR_PRESSURE;
  
}