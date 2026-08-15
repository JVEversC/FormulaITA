#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

#define DAC_RESOLUTION  4095
#define CH1 A2
#define CH2 A3
#define LED_APPS_OK 3
#define RELAY_APPS 6
#define V_REF 5.0

//Limites inferior dos canais
#define BOTTOM_CH1 560
#define BOTTOM_CH2 181

//Modulo dos coeficientes da combinação linear 
#define c1 -0.75
#define c2 5.73

void setup()
{
  Serial.begin(9600);
  dac.begin(0x60);

  pinMode(LED_APPS_OK, OUTPUT);
  pinMode(RELAY_APPS, OUTPUT);
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
}

void loop()
{
  //obtendo os sinais 
  int ch1 = analogRead(CH1);
  delay(20);
  int ch2 = analogRead(CH2);
  delay(20);

  //flag é falsa se o apps está inoperante
  bool flag = (ch1 > 0.8 * BOTTOM_CH1) && (ch2 > 0.8 * BOTTOM_CH2);

  if (flag)
  {
    float out = c1 * (float)ch1 + c2 * (float)ch2;
    uint16_t dac_value = constrain(out, 0, DAC_RESOLUTION);

    digitalWrite(RELAY_APPS, HIGH);
    digitalWrite(LED_APPS_OK, HIGH);

    dac.setVoltage(dac_value, false);
  }
  else
  {
    //Desliga a saída
    dac.setVoltage(0, false);

    //Desliga o relé e pisca o LED
    digitalWrite(RELAY_APPS, LOW);
    digitalWrite(LED_APPS_OK, LOW);
    delay(500);
    digitalWrite(LED_APPS_OK, HIGH);
    delay(500);
  }

  delay(60);
}
