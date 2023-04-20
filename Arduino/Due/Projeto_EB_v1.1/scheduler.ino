/*
   File:   scheduler.ino
   Author: Gr Eletr Automotiva

   Created on 24 de Abril de 2022, 19:09
*/

#include "pwm_lib.h"
#include "config.h"
#include "scheduler.h"
#include "Sensor_DS2.h"


void task_telemetry(void) {
  switch (state) {
    case 0:
      Serial.println("Iniciar teste?");
      while (Serial.available() == 0);
      x = Serial.parseInt();
      if (x == 1) {
        state = 1;
      }
      else {
        state = 0;
        run_test = 0;
      }
      break;
    case 1:
      Serial.println("Entre com a pressao");
      while (Serial.available() == 0);
      p_ref = Serial.parseInt();
      if ((signed int)p_ref != 0) {
        state = 2;
      }
      break;

    case 2:
      Serial.println("Entre com o tempo");
      while (Serial.available() == 0);
      t_test = Serial.parseInt();
      if ((signed int)t_test != 0) {
        Serial.println("Teste em andamento");
        run_test = 1;
        state = 3;
      }
      break;

    case 3:
      break;

    case 4:
      Serial.println("Imprimir resultado?");
      while (Serial.available() == 0);
      y = Serial.parseInt();
      if (y == 1) {
        y = 0;
        x = 0;
        state = 5;
        p_ref_print = (int)p_ref;
        p_ref = -1;
      }
      break;

    case 5:
      Serial.println("Imprimindo");
      for (int i = 0; i < t_test; i++) {
        Serial.println(v_p_1[i], DEC);
        delay(0.1);
        Serial.println(v_p_2[i], DEC);
        delay(0.1);
        Serial.println(v_p_3[i], DEC);
        delay(0.1);
        Serial.println(v_p_4[i], DEC);
        delay(0.1);
        Serial.println(p_ref_print, DEC);
        delay(0.1);
      }
      t_test = 0;
      state = 0;
      break;
  }
}

void task_controller(void) {
  if(state == 3){
    if (counter < t_test && run_test == 1) {
      p_1 = 333 * ((analogRead(sensor_press_1) * (3.3 / (1280.0))) - 0.12);
      p_2 = 333 * ((analogRead(sensor_press_2) * (3.3 / (1280.0))) - 0.12);
      p_3 = 333 * ((analogRead(sensor_press_3) * (3.3 / (1280.0))) - 0.12);
      p_4 = 333 * ((analogRead(sensor_press_4) * (3.3 / (1280.0))) - 0.12);

      v_p_1[counter] = counter;
      //(int)p_1;
      v_p_2[counter] = (int)p_2;
      v_p_3[counter] = (int)p_3;
      v_p_4[counter] = (int)p_4;

      controle_ESC_circuito_1(DC_ESC_ON);
      controle_ESC_circuito_2(DC_ESC_ON);

      controle_roda_1(1, p_1, p_ref);
      controle_roda_2(1, p_2, p_ref);
      controle_roda_3(1, p_3, p_ref);
      controle_roda_4(1, p_4, p_ref);

      pwm_pump.set_duty(DC_PUMP_ON * PUMP_PWM_DUTY_TO_PERIOD_COEF);
      counter++;
//      Serial.println("counter");
//      Serial.println(counter, DEC);
    }
    else {
      pwm_pump.set_duty(DC_PUMP_OFF * PUMP_PWM_DUTY_TO_PERIOD_COEF);

      controle_ESC_circuito_1(DC_ESC_OFF);
      controle_ESC_circuito_2(DC_ESC_OFF);

      controle_roda_1(0, p_1, p_ref);
      controle_roda_2(0, p_2, p_ref);
      controle_roda_3(0, p_3, p_ref);
      controle_roda_4(0, p_4, p_ref);

      if (counter >= t_test) {
        sended = 1;
        counter = 0;
        state = 4;
      }
      run_test = 0;
    }
  }
}
