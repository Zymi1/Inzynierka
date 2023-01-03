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

#include <LiquidCrystal.h>

const int rs = 12, en = 13, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int rawValue; // A/D readings
int offset = 400; // zero pressure adjust
int fullScale = 9630; // max pressure (span) adjust
float R = 287.15; //indywidualna stała gazowa dla powietrza
float kappa = 1.4 //wykładnik adiabaty dla powietrza
float ps; // cisnienie statyczne
float H; //wysokosc
float Hfeet; //wysokosc w stopach
float pspotega; //(ps/p0)^1/5.256
float psulamek; //ps/p0
float pd; //cisnienie dynamiczne
float T_H; //temperatura na wysokosci
float rho_H; //gestosc na wysokosci
float v_IAS; //predkosc indykowana
float v_TAS; //predkosz rzeczywista
float a_H; //predkosc dzwieku na wysokosci

void setup() 
{

  lcd.begin(16, 2);
  
  pinMode(9, OUTPUT); //USTAWIENIE JASNOSCI

  digitalWrite(9,255);

  lcd.createChar(0, Ddd);

}

void loop() 
{
  rawValue = 0;
    for (int x = 0; x < 10; x++) rawValue = rawValue + analogRead(A5);
    ps = (rawValue - offset) * 700.0 / (fullScale - offset); // pressure conversion

  psulamek = ps/101.325;
  pspotega = pow(psulamek, (1/5.256)); 
  H = 44331*(1-pspotega);
  Hfeet = H *3.281;

  T_H = 288 - 6.5*(H/1000);
  rho_H = (ps*1000)/(R*T_H);

  pd = ((analogRead(A2)*5.0)/1024.0)*50;

  v_IAS=sqrt((2*pd*1000)/rho_H);
  v_TAS = v_IAS * sqrt(1.225/rho_H);

  a_H = sqrt(kappa*R*T_H);
  Ma = V_TAS/a_H;
  
  lcd.clear();

  lcd.setCursor(0, 0); 
  lcd.print("WYSOKOSC ");
  lcd.print(analogRead(A5));
  lcd.print(H,1);
  lcd.print(" m");

  lcd.setCursor(0, 1);
  lcd.print(Hfeet,0);
  lcd.print(" FT");

  delay(300);

}
