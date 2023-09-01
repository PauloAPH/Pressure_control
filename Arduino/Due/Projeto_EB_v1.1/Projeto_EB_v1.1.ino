/*/*
   File:   Projeto_EB.ino
   Author: Paulo A P Hayashida, Rennan, Rodrigo França
   Created on 18 de Março de 2022, 15:22
*/
#include "config.h"
#include "variant.h"
#include <due_can.h>

/* Configurações do PWM de HW para a bomba */
using namespace arduino_due::pwm_lib;
pwm<pwm_pin::PWMH1_PA19> pwm_pump; /* Pino PA19 é mapeado ao pin 42 no DUE, canal 1 de PWM */

#define SERIAL_TASK_US 1000000
#define CONTROLER_TASK_US 10000

unsigned char state = 0;
signed int pump_pwm_duty_cycle;

extern pwm<pwm_pin::PWMH1_PA19> pwm_pump;

float p_1 = 0;
float p_2 = 0;
float p_3 = 0;
float p_4 = 0;

bool serial_task = 0;
bool controler_task = 0;

long count_serial = 0;
long count_controler = 0;

int v_p_1[500];
int v_p_2[500];
int v_p_3[500];
int v_p_4[500];

int sended = 0;
int t_test = 300;
int run_test = 0;
int next_msg = 0;
int imprime = 0;
int p_ref_print = 0;
float p_ref = 0;
int x = 0;
int y = 0;
int counter = 0;

void setup() {

  /* Abre a comunicação Serial */
  Serial.begin(9600);

  /* Configuração dos pinos */
  pinMode(43, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(ESC_C1_V1, OUTPUT);
  pinMode(ESC_C1_V2, OUTPUT);
  pinMode(ESC_C2_V1, OUTPUT);
  pinMode(ESC_C2_V2, OUTPUT);
  pinMode(ABS_C1_V1, OUTPUT);
  pinMode(ABS_C1_V2, OUTPUT);
  pinMode(ABS_C2_V1, OUTPUT);
  pinMode(ABS_C2_V2, OUTPUT);
  pinMode(ABS_C3_V1, OUTPUT);
  pinMode(ABS_C3_V2, OUTPUT);
  pinMode(ABS_C4_V1, OUTPUT);
  pinMode(ABS_C4_V2, OUTPUT);
  pinMode(PUMP, OUTPUT);
  //
  /* Configura resolução do ADC */
  analogReadResolution(8);
  
  /* Inicia o PWM de HW */
  pwm_pump.start(PUMP_PWM_PERIOD_PIN_42, 5000);
  tc_setup();
  pwm_pump.set_duty(DC_PUMP_OFF * PUMP_PWM_DUTY_TO_PERIOD_COEF);


  /* Configuração do CAN */
  Can0.begin(CAN_BPS_1000K);
  
  //Only recive the ID 100
  Can0.setRXFilter(0, 0x100, 0x7FF, false);
  Can0.setCallback(0, gotFrameMB0);

}


void tc_setup() {

  PMC->PMC_PCER1 |= PMC_PCER1_PID35;                      // TC8 power ON : Timer Counter 2 channel 2 IS TC8 - see page 38

  TC2->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1  // MCK/2 = 42 M Hz, clk on rising edge
                              | TC_CMR_WAVE               // Waveform mode
                              | TC_CMR_WAVSEL_UP_RC;      // UP mode with automatic trigger on RC Compare

  TC2->TC_CHANNEL[2].TC_RC = 42;  //<*********************  Frequency = (Mck/2)/TC_RC  Hz

  TC2->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;                // Interrupt on RC compare match
  NVIC_EnableIRQ(TC8_IRQn);
  TC2->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Software trigger TC2 counter and enable
}

void TC8_Handler() {

  static uint32_t Count;
  static int ledBlinkController = 0;
  static int ledBlinkSerial = 0;

  count_serial++;
  count_controler++;

  TC2->TC_CHANNEL[2].TC_SR;                               // Read and clear status register

  if (count_controler == CONTROLER_TASK_US){
    
    if (ledBlinkController == 1) {
      digitalWrite(43, HIGH);
      ledBlinkController = 0;
    }
    else {
      digitalWrite(43, LOW);
      ledBlinkController = 1;
    }
    count_controler = 0;
    
    if (state == 3){
      controler_task = 1;
    }
  }
  
  if (count_serial == SERIAL_TASK_US){
    if (ledBlinkSerial== 1) {
      digitalWrite(45, HIGH);
      ledBlinkSerial = 0;
    }
    else {
      digitalWrite(45, LOW);
      ledBlinkSerial = 1;
    }
    count_serial = 0;
    serial_task = 1;
  }
}




void loop() {
  if (serial_task == 1) {
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
        t_test = t_test * 100;
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
    serial_task = 0;
  }

  if (controler_task == 1) {
    if (counter < t_test && run_test == 1) {
      p_1 = 333 * ((analogRead(sensor_press_1) * (3.3 / (1280.0))) - 0.12);
      p_2 = 333 * ((analogRead(sensor_press_2) * (3.3 / (1280.0))) - 0.12);
      p_3 = 333 * ((analogRead(sensor_press_3) * (3.3 / (1280.0))) - 0.12);
      p_4 = 333 * ((analogRead(sensor_press_4) * (3.3 / (1280.0))) - 0.12);

      v_p_1[counter] = (int)p_1;
      v_p_2[counter] = (int)p_2;
      v_p_3[counter] = (int)p_3;
      v_p_4[counter] = (int)p_4;

      controle_ESC_circuito_1(DC_ESC_ON);
      controle_ESC_circuito_2(HIGH);

      controle_roda_1(1, p_1, p_ref);
      controle_roda_2(1, p_2, p_ref);
      controle_roda_3(1, p_3, p_ref);
      controle_roda_4(1, p_4, p_ref);

      pwm_pump.set_duty(DC_PUMP_ON * PUMP_PWM_DUTY_TO_PERIOD_COEF);
      counter++;
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
    controler_task = 0;
  }
}