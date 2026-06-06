#include "config.h"
#include "variant.h"
#include <FIR.h>

using namespace arduino_due::pwm_lib;
pwm<pwm_pin::PWMH1_PA19> pwm_pump;

#define SERIAL_TASK_US      1000000
#define CONTROLER_TASK_US     10000
#define IDSIST_DURATION_S       5
#define IDSIST_SAMPLE_TICKS   200
#define IDSIST_WAIT_TICKS   200000
#define IDSIST_N_SAMPLES      250

unsigned char state = 0;
signed int pump_pwm_duty_cycle;
extern pwm<pwm_pin::PWMH1_PA19> pwm_pump;

float p_1 = 0, p_2 = 0, p_3 = 0, p_4 = 0;
float p1MovAvr = 0, p2MovAvr = 0, p3MovAvr = 0, p4MovAvr = 0;

bool serial_task     = 0;
bool controler_task  = 0;
long count_serial    = 0;
long count_controler = 0;

FIR<float, 8> filtro1, filtro2, filtro3, filtro4;

int *v_p_1 = NULL, *v_p_2 = NULL, *v_p_3 = NULL, *v_p_4 = NULL;
int *p_ref_1_print = NULL, *p_ref_2_print = NULL;
int *p_ref_3_print = NULL, *p_ref_4_print = NULL;

int   sended   = 0;
int   t_test   = 300;
int   run_test = 0;
int   next_msg = 0;
int   imprime  = 0;

float p_ref_1 = 0, p_ref_2 = 0, p_ref_3 = 0, p_ref_4 = 0;
float erro = 0;
int x = 0, y = 0, counter = 0;

static int idsist_dc[IDSIST_N_SAMPLES];
static int idsist_p1[IDSIST_N_SAMPLES];
static int idsist_p2[IDSIST_N_SAMPLES];
static int idsist_p3[IDSIST_N_SAMPLES];
static int idsist_p4[IDSIST_N_SAMPLES];

volatile long count_idsist_sample = 0;
volatile long count_idsist_total  = 0;
volatile long count_idsist_wait   = 0;

volatile bool idsist_sample_tick = false;
volatile bool idsist_wait_done   = false;
volatile bool idsist_done        = false;

int idsist_sample_idx = 0;
int idsist_pwm_param  = 0;

void tc_setup();
void idsist_reset();
void idsist_amostra();
void idsist_envia_dados();

void setup() {
  Serial.begin(9600);
  Serial.println("Connected!");
  pinMode(43, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(ESC_C1_V1, OUTPUT);  pinMode(ESC_C1_V2, OUTPUT);
  pinMode(ESC_C2_V1, OUTPUT);  pinMode(ESC_C2_V2, OUTPUT);
  pinMode(ABS_C1_V1, OUTPUT);  pinMode(ABS_C1_V2, OUTPUT);
  pinMode(ABS_C2_V1, OUTPUT);  pinMode(ABS_C2_V2, OUTPUT);
  pinMode(ABS_C3_V1, OUTPUT);  pinMode(ABS_C3_V2, OUTPUT);
  pinMode(ABS_C4_V1, OUTPUT);  pinMode(ABS_C4_V2, OUTPUT);
  pinMode(PUMP, OUTPUT);

  analogReadResolution(8);

  float coef[8] = {1., 1., 1., 1., 1., 1., 1., 1.};
  filtro1.setFilterCoeffs(coef);
  filtro2.setFilterCoeffs(coef);
  filtro3.setFilterCoeffs(coef);
  filtro4.setFilterCoeffs(coef);

  pwm_pump.start(PUMP_PWM_PERIOD_PIN_42, 5000);
  tc_setup();
  pwm_pump.set_duty(DC_PUMP_OFF * PUMP_PWM_DUTY_TO_PERIOD_COEF);
  digitalWrite(43, HIGH);
}

void tc_setup() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID35;
  TC2->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1
                             | TC_CMR_WAVE
                             | TC_CMR_WAVSEL_UP_RC;
  TC2->TC_CHANNEL[2].TC_RC  = 420;
  TC2->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC8_IRQn);
  TC2->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;
}

void TC8_Handler() {
  static int ledBlinkController = 0;
  static int ledBlinkSerial     = 0;

  TC2->TC_CHANNEL[2].TC_SR;

  count_serial++;
  count_controler++;

  if (count_controler == CONTROLER_TASK_US) {
    ledBlinkController ^= 1;
    digitalWrite(43, ledBlinkController ? HIGH : LOW);
    count_controler = 0;
    if (state == 3) controler_task = 1;
  }
  if (count_serial == SERIAL_TASK_US) {
    ledBlinkSerial ^= 1;
    digitalWrite(45, ledBlinkSerial ? HIGH : LOW);
    count_serial = 0;
    serial_task  = 1;
  }

  if (state == 21 || state == 31) {
    count_idsist_wait++;
    if (count_idsist_wait >= IDSIST_WAIT_TICKS) {
      count_idsist_wait = 0;
      idsist_wait_done  = true;
    }
  }

  if (state == 10) {
    count_idsist_sample++;
    count_idsist_total++;
    if (count_idsist_sample >= IDSIST_SAMPLE_TICKS) {
      count_idsist_sample = 0;
      idsist_sample_tick  = true;
    }
    if (count_idsist_total >= (long)IDSIST_DURATION_S * 100000L) {
      idsist_done = true;
      state = 11;
    }
  }
}

void loop() {
  if (serial_task == 1) {
    switch (state) {

      case 0:
        Serial.println("Iniciar teste? 1 = Teste1 (controle), 2 = Teste2 (identificacao), 3 = Teste3 (ABS)");
        while (Serial.available() == 0);
        x = Serial.parseInt();
        if      (x == 1) state = 1;
        else if (x == 2) state = 20;
        else if (x == 3) state = 30;
        else           { state = 0; run_test = 0; }
        break;

      case 1:
        Serial.println("Entre com a pressao");
        while (Serial.available() == 0); p_ref_1 = Serial.parseInt();
        Serial.println("Entre com a pressao");
        while (Serial.available() == 0); p_ref_2 = Serial.parseInt();
        Serial.println("Entre com a pressao");
        while (Serial.available() == 0); p_ref_3 = Serial.parseInt();
        Serial.println("Entre com a pressao");
        while (Serial.available() == 0); p_ref_4 = Serial.parseInt();
        if ((signed int)p_ref_1 != 0) state = 2;
        break;

      case 2:
        Serial.println("Entre com o tempo");
        while (Serial.available() == 0);
        t_test = Serial.parseInt();
        t_test = t_test * 100;
        v_p_1 = new int[t_test]; v_p_2 = new int[t_test];
        v_p_3 = new int[t_test]; v_p_4 = new int[t_test];
        p_ref_1_print = new int[t_test]; p_ref_2_print = new int[t_test];
        p_ref_3_print = new int[t_test]; p_ref_4_print = new int[t_test];
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
          y = 0; x = 0;
          state   = 5;
          p_ref_1 = -1;
        }
        break;

      case 5:
        Serial.println("Imprimindo");
        for (int i = 0; i < t_test; i++) {
          Serial.println(v_p_1[i],         DEC); delay(0.1);
          Serial.println(v_p_2[i],         DEC); delay(0.1);
          Serial.println(v_p_3[i],         DEC); delay(0.1);
          Serial.println(v_p_4[i],         DEC); delay(0.1);
          Serial.println(p_ref_1_print[i], DEC); delay(0.1);
          Serial.println(p_ref_2_print[i], DEC); delay(0.1);
          Serial.println(p_ref_3_print[i], DEC); delay(0.1);
          Serial.println(p_ref_4_print[i], DEC); delay(0.1);
        }
        delete [] v_p_1;         delete [] v_p_2;
        delete [] v_p_3;         delete [] v_p_4;
        delete [] p_ref_1_print; delete [] p_ref_2_print;
        delete [] p_ref_3_print; delete [] p_ref_4_print;
        t_test = 0;
        state  = 0;
        break;

      case 20:
        Serial.println("Entre com o PWM da bomba (%)");
        while (Serial.available() == 0);
        idsist_pwm_param = Serial.parseInt();
        idsist_reset();
        controle_ESC_circuito_1(DC_ESC_ON);
        controle_ESC_circuito_2(DC_ESC_ON);
        Serial.println("Aguardando 2s...");
        state = 21;
        break;

      case 21:
        break;

      case 30:
        Serial.println("Entre com o PWM das valvulas ABS (%)");
        while (Serial.available() == 0);
        idsist_pwm_param = Serial.parseInt();
        idsist_reset();
        controle_ESC_circuito_1(DC_ESC_ON);
        controle_ESC_circuito_2(DC_ESC_ON);
        controle_ABS_isola_1(idsist_pwm_param);
        controle_ABS_isola_2(idsist_pwm_param);
        controle_ABS_isola_3(idsist_pwm_param);
        controle_ABS_isola_4(idsist_pwm_param);
        Serial.println("Aguardando 2s...");
        state = 31;
        break;

      case 31:
        break;

      case 11:
        pwm_pump.set_duty(DC_PUMP_OFF * PUMP_PWM_DUTY_TO_PERIOD_COEF);
        controle_ESC_circuito_1(DC_ESC_OFF);
        controle_ESC_circuito_2(DC_ESC_OFF);
        controle_ABS_isola_1(DC_ABS_ISOLA_NULL);
        controle_ABS_isola_2(DC_ABS_ISOLA_NULL);
        controle_ABS_isola_3(DC_ABS_ISOLA_NULL);
        controle_ABS_isola_4(DC_ABS_ISOLA_NULL);
        state = 12;
        break;

      case 12:
        Serial.println("Identificacao concluida. Enviando dados...");
        Serial.println("IDSIST_START");
        Serial.println(idsist_sample_idx, DEC);
        idsist_envia_dados();
        Serial.println("IDSIST_END");
        state = 0;
        break;

      default:
        break;
    }
    serial_task = 0;
  }

  if (idsist_wait_done) {
    idsist_wait_done = false;
    pwm_pump.set_duty(idsist_pwm_param * PUMP_PWM_DUTY_TO_PERIOD_COEF);
    state = 10;
  }

  if (controler_task == 1) {
    if (counter < t_test && run_test == 1) {
      p1MovAvr = filtro1.processReading(analogRead(sensor_press_1));
      p2MovAvr = filtro2.processReading(analogRead(sensor_press_2));
      p3MovAvr = filtro3.processReading(analogRead(sensor_press_3));
      p4MovAvr = filtro4.processReading(analogRead(sensor_press_4));
      p_1 = 333 * ((p1MovAvr * (3.3 / 1280.0)) - 0.12);
      p_2 = 333 * ((p2MovAvr * (3.3 / 1280.0)) - 0.12);
      p_3 = 333 * ((p3MovAvr * (3.3 / 1280.0)) - 0.12);
      p_4 = 333 * ((p4MovAvr * (3.3 / 1280.0)) - 0.12);

      v_p_1[counter] = (int)p_1;
      v_p_2[counter] = (int)p_2;
      v_p_3[counter] = (int)p_3;
      v_p_4[counter] = (int)p_4;

      controle_ESC_circuito_1(DC_ESC_ON);
      controle_ESC_circuito_2(DC_ESC_ON);

      p_ref_1_print[counter] = p_ref_1;
      p_ref_2_print[counter] = p_ref_2;
      p_ref_3_print[counter] = p_ref_3;
      p_ref_4_print[counter] = p_ref_4;

      erro = controle_roda_1(1, p_1, (float)p_ref_1);
      controle_roda_2(1, p_2, (float)p_ref_2);
      controle_roda_3(1, p_3, (float)p_ref_3);
      controle_roda_4(1, p_4, (float)p_ref_4);

      if (erro > 5)
        pwm_pump.set_duty(DC_PUMP_ON * PUMP_PWM_DUTY_TO_PERIOD_COEF);
      else if (erro > -5)
        pwm_pump.set_duty(50         * PUMP_PWM_DUTY_TO_PERIOD_COEF);
      else
        pwm_pump.set_duty(40         * PUMP_PWM_DUTY_TO_PERIOD_COEF);

      counter++;
    } else {
      pwm_pump.set_duty(DC_PUMP_OFF * PUMP_PWM_DUTY_TO_PERIOD_COEF);
      controle_ESC_circuito_1(DC_ESC_OFF);
      controle_ESC_circuito_2(DC_ESC_OFF);
      controle_roda_1(0, p_1, p_ref_1);
      controle_roda_2(0, p_2, p_ref_2);
      controle_roda_3(0, p_3, p_ref_3);
      controle_roda_4(0, p_4, p_ref_4);
      p1MovAvr = filtro1.processReading(0);
      p2MovAvr = filtro2.processReading(0);
      p3MovAvr = filtro3.processReading(0);
      p4MovAvr = filtro4.processReading(0);
      if (counter >= t_test) {
        sended  = 1;
        counter = 0;
        state   = 4;
      }
      run_test = 0;
    }
    controler_task = 0;
  }

  if (state == 10 && idsist_sample_tick) {
    idsist_sample_tick = false;
    idsist_amostra();
  }
}

void idsist_reset() {
  idsist_sample_idx   = 0;
  count_idsist_sample = 0;
  count_idsist_total  = 0;
  count_idsist_wait   = 0;
  idsist_sample_tick  = false;
  idsist_wait_done    = false;
  idsist_done         = false;
  memset(idsist_dc, 0, sizeof(idsist_dc));
  memset(idsist_p1, 0, sizeof(idsist_p1));
  memset(idsist_p2, 0, sizeof(idsist_p2));
  memset(idsist_p3, 0, sizeof(idsist_p3));
  memset(idsist_p4, 0, sizeof(idsist_p4));
}

void idsist_amostra() {
  if (idsist_sample_idx >= IDSIST_N_SAMPLES) return;
  idsist_dc[idsist_sample_idx] = idsist_pwm_param;
  idsist_p1[idsist_sample_idx] = analogRead(sensor_press_1);
  idsist_p2[idsist_sample_idx] = analogRead(sensor_press_2);
  idsist_p3[idsist_sample_idx] = analogRead(sensor_press_3);
  idsist_p4[idsist_sample_idx] = analogRead(sensor_press_4);
  idsist_sample_idx++;
}

void idsist_envia_dados() {
  for (int i = 0; i < idsist_sample_idx; i++) {
    Serial.println(idsist_dc[i], DEC);
    Serial.println(idsist_p1[i], DEC);
    Serial.println(idsist_p2[i], DEC);
    Serial.println(idsist_p3[i], DEC);
    Serial.println(idsist_p4[i], DEC);
  }
}
