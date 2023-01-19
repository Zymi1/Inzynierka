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

const int rs = 6, en = 7, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int rawValue; // A/D readings
int offset = 400; // zero pressure adjust 400
int fullScale = 9630; // max pressure (span) adjust
int filterWeight = 4;
float R = 287.15; //indywidualna stała gazowa dla powietrza
float kappa = 1.4; //wykładnik adiabaty dla powietrza
float ps; // final pressure
float H; //wysokosc
float pspotega; // (ps/101.325)^(1/5.256)
float psulamek; // ps/101.325
int average_pd; // srednie cisnienie dynamiczne
int average_ps; // srednie cisnienie statyczne
float pd; //cisnienie dynamiczne
float T_H; //temperatura na wysokosci
float rho_H; //gestosc na wysokosci
float v_IAS; //predkosc indukowana
float v_TAS; //predkosc rzeczywista
float T; //temperatura w sali
float a_H; //predkosc dzwieku na wysokosci
float Ma; //liczba Macha
int ekran = 0;  //obecnie wyswietlany ekran, 0 na uruchomieniu
Smoothed <int> mySensor; //uruchomienie modułu uśredniającego cisnienia statycznego
Smoothed <int> mySensor2; //uruchomienie modułu uśredniającego cisnienia dynamicznego


void setup() //startup
{
  
  unsigned status; //poczatek uruchomienia modułu BME280

  status = bme.begin();  

      if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    } //zakonczenie ucuhamiania modułu BME280

  mySensor.begin(SMOOTHED_AVERAGE, 10); //wyznaczenie rodzaju i ilosci pomiarow modulu usredniajacego cisnienia statycznego
  mySensor2.begin(SMOOTHED_AVERAGE, 30); //wyznaczenie rodzaju i ilosci pomiarow modulu usredniajacego cisnienia dynamicznego


  lcd.begin(16, 2); //uruchomienie ekranu LCD o rozmiarach 16 pol na 2 pola
  
  pinMode(13, INPUT); //ustalenie pinu 13 jako wejscia
  pinMode(12, INPUT);
  pinMode(11, INPUT);
  pinMode(10, INPUT);
  pinMode(9, INPUT);
  pinMode(8, INPUT);

}

void loop() //petla w ktorej wykonuja sie pozostale czynnosci
{

  mySensor.add(analogRead(A0)); //odczyt wartosci z czujnika cisnienia systycznego
  mySensor2.add(analogRead(A2)); //odczyt wartosci z czujnika cisnienia dynamicznego


  rawValue = 0;
    average_ps = average_ps + mySensor.get() - average_ps; //usrednienie wartosci cisnienia

      for (int x = 0; x < 10; x++) rawValue = rawValue + average_ps; //konwersja wartosci cisnienia do dalszych obliczen
      ps = (rawValue - offset) * 700.0 / (fullScale - offset); // konwersja cisnienia

  psulamek = ps/101.325;
  pspotega = pow(psulamek, (1/5.256)); 
  H = 44331*(1-pspotega);

  T = bme.readTemperature() + 273,15;  //odczyt temperatury w sali w Kelwinach

  T_H = T - 6.5*(H/1000);
  rho_H = (ps*1000)/(R*T_H);

  for (int i = 0; i < 20; i++)  //petla usredniajaca wartosc cisnienia dynamicznego
  {
    if (mySensor2.get() < 12) average_pd = 0; //jesli wartosc odczytu sensora roznicowego < 12, ustaw warstosc odczytu jako "0"
    else
    average_pd = average_pd + mySensor2.get() - average_pd; //usrednienie wartosci cisnienia
  }
  pd = ((average_pd*5.0)/1024.0)*48; //liniowa konwersja wartosci napiecia wejsciowego (max 5V) w skali 1024 punktow (2^10) pomnozona przez wspolczynnik skalujacy

  v_IAS = (sqrt((2*pd*1000)/rho_H))*3,6;
  v_TAS = (v_IAS * sqrt(1.225/rho_H));

  a_H = sqrt(kappa*R*T_H);
  Ma = v_TAS/a_H;

  lcd.clear(); //przed wyswietleniem czegokolwiek, wyczysc ekran

  if (digitalRead(13) == 1) ekran = ekran * 0; //jesli zostal przycisniety przycisk "13", zmien wartosc "ekran" na 0
  if (digitalRead(12) == 1) ekran = ekran * 0 + 1; //jesli zostal przycisniety przycisk "12", zmien wartosc "ekran" na 1
  if (digitalRead(11) == 1) ekran = ekran * 0 + 2;
  if (digitalRead(10) == 1) ekran = ekran * 0 + 3;
  if (digitalRead(9) == 1) ekran = ekran * 0 + 4;
  if (digitalRead(8) == 1) ekran = ekran * 0 + 5;  

  switch (ekran) //switch case zalezny od wartosci parametru ekranu
  {
    case 0: //jesli wartosc zmiennej "ekran" = 0
      lcd.setCursor(3, 0); //ustaw kursor na gornej czesci ekranu, cztery miejsca od lewej
      lcd.print(" WYSOKOSC"); //wyswietl "WYSOKOSC" na ekranie

      if (H<1000) // centrowanie tekstu dla H < 1000
      {
        lcd.setCursor(6, 1); //ustaw kursor na dolnej czesci ekranu, siedem miejsc od lewej
        lcd.print(H,0); //wyswietl wysokosc bez miejsc po przecinku
        lcd.print(" m   ");
      }

      if (H<=9999 && H>=1000) // centrowanie tekstu dla 9999 > H > 1000
      {
        lcd.setCursor(5, 1);
        lcd.print(H,0);
        lcd.print(" m ");
      }

      if (H>9999) // centrowanie tekstu dla H > 9999
      {
        lcd.setCursor(4, 1);
        lcd.print(H,0);
        lcd.print(" m ");
      }
      break; //wroc na poczatek danego case'a

    case 1: //jesli wartosc zmiennej "ekran" = 1
      lcd.setCursor(2, 0); //ustaw kursor na gornej czesci ekranu, trzy miejsca od lewej
      lcd.print("PREDKOSC IAS");

      if (v_IAS < 1) // centrowanie tekstu dla predkosci indukowanej < 1
      {
        lcd.setCursor(5, 1);
        lcd.print(v_IAS,0); //wyswietl predkosc indukowana bez miejsc po przecinku
        lcd.print(" km/h");
      }

      if (v_IAS > 1) // centrowanie tekstu dla predkosci indukowanej > 1
      {
        lcd.setCursor(4, 1);
        lcd.print(v_IAS,0); //wyswietl predkosc indukowana bez miejsc po przecinku
        lcd.print(" km/h");
      } 
      break;

    case 2: //jesli wartosc zmiennej "ekran" = 2
      lcd.setCursor(2, 0); 
      lcd.print("PREDKOSC TAS");

      if (v_TAS < 1)
      {
        lcd.setCursor(5, 1);
        lcd.print(v_TAS,0);
        lcd.print(" km/h");
      }

      if (v_TAS > 1)
      {
        lcd.setCursor(4, 1);
        lcd.print(v_TAS,0);
        lcd.print(" km/h");
      }
      break;

    case 3: //jesli wartosc zmiennej "ekran" = 3
      lcd.setCursor(0, 0); //ustaw kursor w lewym gornym rogu ekranu
      lcd.print("PREDKOSC DZWIEKU");
      
      lcd.setCursor(3, 1);
      lcd.print(a_H,2); //wyswietl predkosc dzwieku do dwoch miejsc po przecinku
      lcd.print(" m/s");
      break;

    case 4: //jesli wartosc zmiennej "ekran" = 4
      lcd.setCursor(2, 0); 
      lcd.print("LICZBA MACHA");

      lcd.setCursor(6, 1); 
      lcd.print(Ma,2);  
      break;

    case 5: //jesli wartosc zmiennej "ekran" = 5

      lcd.setCursor(3, 0); 
      lcd.print("Ps = ");
      lcd.print(ps,2);

      lcd.setCursor(3, 1); 
      lcd.print("Pd = ");
      lcd.print(pd,2);
      break;
  }
  
  delay(150); //poczekaj 150 ms przed powtorzeniem calosci obliczen i wyswietlania danych na ekranie

}
