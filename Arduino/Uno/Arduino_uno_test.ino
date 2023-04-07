#include <SoftwareSerial.h>

int i = 0;
int state = 0;
int x = 0;
int y = -1;
int p_ref = -1;
void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Native USB only
  }
}

void loop()
{
  switch(state){
    case 0:
      Serial.println("comecar teste");
      x = Serial.parseInt();
      if(x == 1){
        state = 1;
      }
      break;
    case 1:
      Serial.println("Entre com a pressao");
      p_ref = Serial.parseInt();
      if(p_ref != -1){
        state = 2;
      }
      break;
    case 2:
      Serial.println("Imprimir resultado?");
      y = Serial.parseInt();
      if(y == 1){
        state = 3;
        y = 0;
        x = 0;
        p_ref = -1;
      }
      break;
    case 3:
      Serial.println("Imprimindo");
      for(i = 0; i < 50;i++){
        Serial.println(i+20, DEC);
        delay(10);
        Serial.println(i+30, DEC);
        delay(10);
        Serial.println(i+5, DEC);
        delay(10);
        Serial.println(i+8, DEC);
        delay(10);
      }
      state = 0;
      break;
  }
}

