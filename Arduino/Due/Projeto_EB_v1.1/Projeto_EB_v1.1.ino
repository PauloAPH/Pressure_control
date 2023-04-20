
/*/*
   File:   Projeto_EB.ino
   Author: Paulo A P Hayashida, Rennan, Rodrigo França
   Created on 18 de Março de 2022, 15:22
*/

#include "scheduler.h"
#include "config.h"

/* Configurações do PWM de HW para a bomba */
using namespace arduino_due::pwm_lib;

pwm<pwm_pin::PWMH1_PA19> pwm_pump; /* Pino PA19 é mapeado ao pin 42 no DUE, canal 1 de PWM */

#define TASK_TELEMETRY_TIMEMS            2010
#define TASK_CONTROLLER_TIMEMS           10

void task_telemetry(void);
void task_controller(void);

/* Tasks para o sistema */
Task tTelemetry (TASK_TELEMETRY_TIMEMS , TASK_FOREVER, &task_telemetry);
Task tController(TASK_CONTROLLER_TIMEMS, TASK_FOREVER, &task_controller);

/* Cria o Scheduler */
Scheduler xSchRunner;

unsigned char state = 0;
signed int pump_pwm_duty_cycle;

extern pwm<pwm_pin::PWMH1_PA19> pwm_pump;

float p_1 = 0;
float p_2 = 0;
float p_3 = 0;
float p_4 = 0;

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

  /* Configura resolução do ADC */
  analogReadResolution(8);
  /* Inicia o PWM de HW */
  pwm_pump.start(PUMP_PWM_PERIOD_PIN_42, 5000);
  /* Inicia o scheduler */
  xSchRunner.init();
  /* Adiciona as tasks ao scheduler */
  xSchRunner.addTask(tTelemetry);
  xSchRunner.addTask(tController);
  /* Habilita as tasks */
  tTelemetry.enable();
  tController.enable();
}

void loop() {
  /* Executa o Scheduller */
  xSchRunner.execute();
}


void tc_setup() {

  PMC->PMC_PCER1 |= PMC_PCER1_PID35;                        // TC8 power ON : Timer Counter 2 channel 2 IS TC8 - see page 38

  //PIOD->PIO_PDR |= PIO_PDR_P7;                            // Set the pin to the peripheral
  //PIOD->PIO_ABSR |= PIO_PD7B_TIOA8;                       // Peripheral type B

  TC2->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1  // MCK/2 = 42 M Hz, clk on rising edge
                              | TC_CMR_WAVE               // Waveform mode
                              | TC_CMR_WAVSEL_UP_RC;       // UP mode with automatic trigger on RC Compare
  //  | TC_CMR_ACPA_CLEAR         // Clear TIOA2 on RA compare match
  //  | TC_CMR_ACPC_SET;          // Set TIOA2 on RC compare match


  TC2->TC_CHANNEL[2].TC_RC = 42;  //<*********************  Frequency = (Mck/2)/TC_RC  Hz
  //TC2->TC_CHANNEL[2].TC_RA = 21;  //<********************   Any Duty cycle in between 1 and TC_RC


  TC2->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;//TC_IER_CPAS;
  NVIC_EnableIRQ(TC8_IRQn);
  TC2->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Software trigger TC2 counter and enable
}

void TC8_Handler() {

  static uint32_t Count;

  TC2->TC_CHANNEL[2].TC_SR;                               // Read and clear status register
  if (Count++ == 1000000) {
    Count = 0;
    PIOB->PIO_ODSR ^= PIO_ODSR_P27;                       // Toggle LED every 1 Hz
  }
}
