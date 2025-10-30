#include <Wire.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Adafruit_INA219.h>
#include <Servo.h>

// ---------- DS18B20 ----------
#define DS18B20_1_PIN 2
#define DS18B20_2_PIN 3
#define DS18B20_3_PIN 4

OneWire oneWire1(DS18B20_1_PIN);
DallasTemperature sensor1(&oneWire1);

OneWire oneWire2(DS18B20_2_PIN);
DallasTemperature sensor2(&oneWire2);

OneWire oneWire3(DS18B20_3_PIN);
DallasTemperature sensor3(&oneWire3);

// ---------- INA219 ----------
Adafruit_INA219 ina219;

// ---------- ESC (Servo) ----------
Servo esc;
int motorPin = 11;
int velocidadeMicrossegundos = 1000;
int valorPotenciometroSimulado = 0;

void setup() {
  Serial.begin(115200);

  // Inicializa ESC
  esc.attach(motorPin);
  esc.writeMicroseconds(500);
  delay(10000);  // Tempo para o ESC inicializar

  // Inicializa sensores
  sensor1.begin();
  sensor2.begin();

  if (!ina219.begin()) {
    Serial.println("Falha ao encontrar INA219!");
    while (1) delay(10);
  }

  ina219.setCalibration_32V_2A();
}

void loop() {
  // ---------- Controle do ESC via Serial ----------
    // Acelera gradualmente

  if (Serial.available() > 0) 
  {
      String comando = Serial.readStringUntil('\n');
      if (comando == "1")
      {
          LigaEsc();
      }
      else if (comando == "2")
      {
          DesligaEsc();
      }
  }

  // ---------- Leitura dos sensores ----------
  float busVoltage = ina219.getBusVoltage_V();
  float current = ina219.getCurrent_mA();
  sensor1.requestTemperatures();
  sensor2.requestTemperatures();
  float temp1 = sensor1.getTempCByIndex(0);
  float temp2 = sensor2.getTempCByIndex(0);

  // ---------- Envio Serial ----------
  Serial.print(temp1); Serial.print(":");
  Serial.print(temp2); Serial.print(":");
  Serial.println(busVoltage);

  delay(2000);
}

void DesligaEsc()
{
  esc.writeMicroseconds(500);
}
void LigaEsc()
{
  esc.writeMicroseconds(1500);
}
