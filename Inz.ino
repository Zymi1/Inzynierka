/*
VSS - MINUS
VDD - PLUS
V0 - 2K OHM - MINUS
RS - 12
RW - MINUS
E - 13
D4 DO D7 - 5 DO 2
A - 10
K - MINUS

https://docs.arduino.cc/learn/electronics/lcd-displays
*/

#include <LiquidCrystal.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Smoothed.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

byte Ddd[8] = 
{
  0b10001,
  0b11001,
  0b11001,
  0b01101,
  0b01101,
  0b01101,
  0b00111,
  0b00111,
};

const int rs = 12, en = 13, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int rawValue; // A/D readings
int offset = 400; // zero pressure adjust 400
int fullScale = 9630; // max pressure (span) adjust
int filterWeight = 4;
float R = 287.15; //indywidualna stała gazowa dla powietrza
float kappa = 1.4; //wykładnik adiabaty dla powietrza
float ps; // final pressure
float H; //wysokosc
float Hfeet;
float pspotega;
float psulamek;
int average_pd;
int average_ps;
float pd;
float T_H;
float rho_H;
float v_IAS;
float v_TAS;
float T;
float a_H;
float Ma;
int ekran = 1;
Smoothed <int> mySensor;
Smoothed <int> mySensor2;

void setup() 
{
  unsigned status;

  status = bme.begin();  

      if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }

  mySensor.begin(SMOOTHED_AVERAGE, 10);
  mySensor2.begin(SMOOTHED_AVERAGE, 30);

  lcd.begin(16, 2);
  
  pinMode(10, OUTPUT); //USTAWIENIE JASNOSCI
  pinMode(8, INPUT);

  digitalWrite(10,255);

  lcd.createChar(0, Ddd);

}

void loop() 
{

  mySensor.add(analogRead(A0));
  mySensor2.add(analogRead(A2));

  rawValue = 0;
    average_ps = average_ps + mySensor.get() - average_ps;

      for (int x = 0; x < 10; x++) rawValue = rawValue + average_ps;
      ps = (rawValue - offset) * 700.0 / (fullScale - offset); // pressure conversion

  psulamek = ps/101.325;
  pspotega = pow(psulamek, (1/5.256)); 
  H = 44331*(1-pspotega);
  Hfeet = H *3.281;

  T = bme.readTemperature() + 273,15;  

  T_H = T - 6.5*(H/1000);
  rho_H = (ps*1000)/(R*T_H);

  for (int i = 0; i < 20; i++)  
  {
    if (mySensor2.get() < 12) average_pd = 0;
    else
    average_pd = average_pd + mySensor2.get() - average_pd;
  }
  pd = ((average_pd*5.0)/1024.0)*48;

  v_IAS = (sqrt((2*pd*1000)/rho_H))*3,6;
  v_TAS = (v_IAS * sqrt(1.225/rho_H));

  a_H = sqrt(kappa*R*T_H);
  Ma = v_TAS/a_H;

  lcd.clear();
  if (digitalRead(8) == 1) ekran++;
    
  switch (ekran)
  {
    case 0:
      lcd.setCursor(0, 0); 
      if (H<1000) {
        lcd.print(" WYSOKOSC");
        lcd.print(" ");
        lcd.print(H,0);
        lcd.print(" m");
      }
      else if (H>1000)
      { 
      lcd.print("WYSOKOSC  ");
      lcd.print(H,0);
      lcd.print(" m");
      }
      
      lcd.setCursor(0, 1);
      
      lcd.print("  Ps ");
      lcd.print(ps,2);
      lcd.print(" kPa");
      break;

    case 1:
      lcd.setCursor(2, 0); 
      lcd.print("PREDKOSC IAS");

      lcd.setCursor(5, 1);
      lcd.print(v_IAS,0);
      lcd.print(" km/h");
      break;

    case 2:
      lcd.setCursor(2, 0); 
      lcd.print("PREDKOSC TAS");

      lcd.setCursor(5, 1);
      lcd.print(v_TAS,0);
      lcd.print(" km/h");
      break;

    case 3:
      lcd.setCursor(3, 0); 
      lcd.print("a ");
      lcd.print(a_H,1);
      lcd.print(" m/s");
      
      lcd.setCursor(5, 1);
      lcd.print("Ma ");
      lcd.print(Ma,2);
      break;

    case 4:
      lcd.setCursor(0, 0); 
      lcd.print(mySensor2.get());
      break;

    case 5:
      ekran = ekran * 0;
      break;
  
  }
  delay(300);

}
