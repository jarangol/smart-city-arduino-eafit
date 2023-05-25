// Name:         arduinoSmartCity
// Description:  Testbench simulating a smart city using Arduino, LDR Light sensors, traffic lights LEDs, CO2 sensr
// Author:       David Velasquez
// Date:         12/09/2018

// Library definition
#include <Wire.h>              //Library required for I2C comms (LCD)
#include <LiquidCrystal_I2C.h> //Library for LCD display via I2C
#include <math.h>              //Mathematics library for pow function (CO2 computation)

// I/O pin labeling
#define LDR1 0  // LDR Light sensor from traffic light 1 connected in pin A0
#define LDR2 1  // LDR Light sensor from traffic light 2 connected in pin A1
#define CO2 3   // CO2 sensor connected in pin A3
#define P1 37   // Traffic light 1 button connected in pin 37
#define P2 36   // Traffic light 2 button connected in pin 36
#define CNY1 35 // Infrared sensor 1 in traffic light 1 connected in pin 35
#define CNY2 34 // Infrared sensor 2 in traffic light 1 connected in pin 34
#define CNY3 33 // Infrared sensor 3 in traffic light 1 connected in pin 33
#define CNY4 32 // Infrared sensor 4 in traffic light 2 connected in pin 32
#define CNY5 31 // Infrared sensor 5 in traffic light 2 connected in pin 31
#define CNY6 30 // Infrared sensor 6 in traffic light 2 connected in pin 30
#define LRC 22  // Red traffic light 1 connected in pin 22
#define LAC 23  // Yellow traffic light 1 connected in pin 23
#define LVC 24  // Green traffic light 1 connected in pin 24
#define LRT 25  // Red traffic light 2 connected in pin 25
#define LVT 27  // Green traffic light 2 connected in pin 27
#define LAT 26  // Yellow traffic light 2 connected in pin 26

// Constant definitions
//->CO2
const float DC_GAIN = 8.5; // define the DC gain of amplifier CO2 sensor
// const float ZERO_POINT_VOLTAGE = 0.4329; //define the output of the sensor in volts when the concentration of CO2 is 400PPM
const float ZERO_POINT_VOLTAGE = 0.265;                                                  // define the output of the sensor in volts when the concentration of CO2 is 400PPM
const float REACTION_VOLTAGE = 0.059;                                                    // define the “voltage drop” of the sensor when move the sensor from air into 1000ppm CO2
const float CO2Curve[3] = {2.602, ZERO_POINT_VOLTAGE, (REACTION_VOLTAGE / (2.602 - 3))}; // Line curve with 2 points

// Variable definitions
char comm = '\0'; // Command to test an actuator or sensor
float volts = 0;  // Variable to store current voltage from CO2 sensor
float co2 = 0;    // Variable to store CO2 value

// Library definitions
LiquidCrystal_I2C lcd(0x27, 16, 4); // Set the LCD address to 0x27 for a 16 chars and 4 line display

void calculateCo2()
{
  volts = analogRead(CO2) * 5.0 / 1023.0; // Convert CO2 ADC to volts
  if (volts / DC_GAIN >= ZERO_POINT_VOLTAGE)
  {
    return;
  }

  co2 = pow(10, ((volts / DC_GAIN) - CO2Curve[1]) / CO2Curve[2] + CO2Curve[0]);
}

int getLDR1()
{
  Serial.println(analogRead(LDR1));
  return analogRead(LDR1);
}

int getLDR2()
{
  return analogRead(LDR2);
}

void printInLCDLine1(String a)
{
  lcd.setCursor(0, 0);
  lcd.print(a);
}

void printInLCDLine2(String a)
{
  lcd.setCursor(0, 1);
  lcd.print(a);
}

void clearLCD()
{
  lcd.clear();
}

void readComm()
{
  if (Serial.available() > 0)
  {
    comm = Serial.read();
    comm = '\0';
  }
}

void checkDigitalIn()
{ // Subroutine to check all digital inputs
  if (digitalRead(P1) == HIGH)
  {
    Serial.println("P1: ON");
    delay(300); // Debouncing for buttons using delay of 300 ms
    while (digitalRead(P1) == HIGH)
    {
    } // Debouncing
  }
  if (digitalRead(P2) == HIGH)
  {
    Serial.println("P2: ON");
    delay(300); // Debouncing for buttons using delay of 300 ms
    while (digitalRead(P2) == HIGH)
    {
    } // Debouncing
  }
  if (digitalRead(CNY1) == LOW)
  {
    Serial.println("CNY1: ON");
    delay(300); // Debouncing for buttons using delay of 300 ms
    while (digitalRead(CNY1) == LOW)
    {
    } // Debouncing
  }
  if (digitalRead(CNY2) == LOW)
  {
    Serial.println("CNY2: ON");
    delay(300); // Debouncing for buttons using delay of 300 ms
    while (digitalRead(CNY2) == LOW)
    {
    } // Debouncing
  }
  if (digitalRead(CNY3) == LOW)
  {
    Serial.println("CNY3: ON");
    delay(300); // Debouncing for buttons using delay of 300 ms
    while (digitalRead(CNY3) == LOW)
    {
    } // Debouncing
  }
  if (digitalRead(CNY4) == LOW)
  {
    Serial.println("CNY4: ON");
    delay(300); // Debouncing for buttons using delay of 300 ms
    while (digitalRead(CNY4) == LOW)
    {
    } // Debouncing
  }
  if (digitalRead(CNY4) == LOW)
  {
    Serial.println("CNY4: ON");
    delay(300); // Debouncing for buttons using delay of 300 ms
    while (digitalRead(CNY4) == LOW)
    {
    } // Debouncing
  }
  if (digitalRead(CNY5) == LOW)
  {
    Serial.println("CNY5: ON");
    delay(300); // Debouncing for buttons using delay of 300 ms
    while (digitalRead(CNY5) == LOW)
    {
    } // Debouncing
  }
  if (digitalRead(CNY6) == LOW)
  {
    Serial.println("CNY6: ON");
    delay(300); // Debouncing for buttons using delay of 300 ms
    while (digitalRead(CNY6) == LOW)
    {
    } // Debouncing
  }
}

/////////////////////////////////////////////////// MEF SM1
#define VERDE 0
#define AMARILLO 1
#define ROJO 2
#define POFF 3

const unsigned long tvc = 10000, tac = 3000;
const unsigned long tvt = 5000, tat = 3000, LDR_MIN = 300, PTIME = 1000;

int eSemaforo1 = VERDE, eSemaforo2 = ROJO;
;
bool CA = 0, TA = 0;
unsigned long trc = 0, trt = 0;

/////////////////////////////////////////////////// FUNCIONES SM1

void pintarSemaforo1Verde()
{
  digitalWrite(LVC, HIGH);
  digitalWrite(LRC, LOW);
  digitalWrite(LAC, LOW);
}

void moverSemaforo1Amarillo()
{
  eSemaforo1 = AMARILLO;
  trc = millis();
}

void pintarSemaforo1Amarillo()
{
  digitalWrite(LVC, LOW);
  digitalWrite(LRC, LOW);
  digitalWrite(LAC, HIGH);
}

void moverSemaforo1Rojo()
{
  eSemaforo1 = ROJO;
  TA = 1;
}

void pintarSemaforo1Rojo()
{
  digitalWrite(LAC, LOW);
  digitalWrite(LVC, LOW);
  digitalWrite(LRC, HIGH);
}

void moverSemaforo1Verde()
{
  CA = 0;
  eSemaforo1 = VERDE;
  trc = millis();
}

void apagarSemaforo1()
{
  digitalWrite(LAC, LOW);
  digitalWrite(LVC, LOW);
  digitalWrite(LRC, LOW);
}

void apagarSemaforo2()
{
  digitalWrite(LAT, LOW);
  digitalWrite(LVT, LOW);
  digitalWrite(LRT, LOW);
}
/////////////////////////////////////////////////// FUNCIONES

void MEF_SM1()
{
  switch (eSemaforo1)
  {
  case VERDE:
    pintarSemaforo1Verde();

    if (getLDR1() < LDR_MIN)
    {
      eSemaforo1 = POFF;
      trc = millis();
    }
    else if ((millis() - trc > tvc))
    {
      moverSemaforo1Amarillo();
    }

    break;
  case AMARILLO:
    pintarSemaforo1Amarillo();
    if (getLDR1() < LDR_MIN && (millis() - trc > PTIME))
    {
      eSemaforo1 = POFF;
      trc = millis();
    }
    else if ((millis() - trc > tac))
    {
      moverSemaforo1Rojo();
    }
    break;
  case ROJO:
    pintarSemaforo1Rojo();
    if (digitalRead(P1) == HIGH)
    {
      CA = 1;
      eSemaforo2 = ROJO;
    }

    if (getLDR1() < LDR_MIN)
    {
      eSemaforo1 = POFF;
      trc = millis();
    }
    else if (CA)
    {
      moverSemaforo1Verde();
    }
    break;
  case POFF:
    apagarSemaforo1();
    if (millis() - trc >= PTIME)
    {
      eSemaforo1 = AMARILLO;
      trc = millis();
    }

    break;
  }
}

/////////////////////////////////////////////////// MEF SM2
void MEF_SM2()
{
  switch (eSemaforo2)
  {
  case VERDE:
    digitalWrite(LRT, LOW);
    digitalWrite(LAT, LOW);
    digitalWrite(LVT, HIGH);

    if (getLDR2() < LDR_MIN)
    {
      eSemaforo2 = POFF;
    }
    else if ((millis() - trt >= tvt))
    {
      eSemaforo2 = AMARILLO;
    }
    trt = millis();
    break;
  case AMARILLO:
    digitalWrite(LVT, LOW);
    digitalWrite(LRT, LOW);
    digitalWrite(LAT, HIGH);
    if (getLDR2() < LDR_MIN && (millis() - trt > PTIME))
    {
      eSemaforo2 = POFF;
      trt = millis();
    }
    else if ((millis() - trt >= tat))
    {
      CA = 1;
      eSemaforo2 = ROJO;
    }
    break;
  case ROJO:
    digitalWrite(LAT, LOW);
    digitalWrite(LVT, LOW);
    digitalWrite(LRT, HIGH);

    if (getLDR2() < LDR_MIN)
    {
      eSemaforo2 = POFF;
      trt = millis();
    }
    else if (digitalRead(P2) == HIGH)
    {
      TA = 1;
      eSemaforo1 = ROJO;
    }

    if (TA)
    {
      TA = 0;
      eSemaforo2 = VERDE;
      trt = millis();
    }
    break;
  case POFF:
    apagarSemaforo2();
    if (millis() - trt >= PTIME)
    {
      eSemaforo2 = AMARILLO;
      trt = millis();
    }

    break;
  }
}

// Configuration
void setup()
{
  // Pin config
  pinMode(P1, INPUT);   // Traffic light 1 button as Input
  pinMode(P2, INPUT);   // Traffic light 2 button as Input
  pinMode(CNY1, INPUT); // Infrared sensor 1 in traffic light 1 as Input
  pinMode(CNY2, INPUT); // Infrared sensor 2 in traffic light 1 as Input
  pinMode(CNY3, INPUT); // Infrared sensor 3 in traffic light 1 as Input
  pinMode(CNY4, INPUT); // Infrared sensor 4 in traffic light 2 as Input
  pinMode(CNY5, INPUT); // Infrared sensor 5 in traffic light 2 as Input
  pinMode(CNY6, INPUT); // Infrared sensor 6 in traffic light 2 as Input
  pinMode(LRC, OUTPUT); // Red traffic light 1 as Output
  pinMode(LAC, OUTPUT); // Yellow traffic light 1 as Output
  pinMode(LVC, OUTPUT); // Green traffic light 1 as Output
  pinMode(LRT, OUTPUT); // Red traffic light 2 as Output
  pinMode(LAT, OUTPUT); // Yellow traffic light 2 as Output
  pinMode(LVT, OUTPUT); // Green traffic light 2 as Output

  // Output cleaning
  digitalWrite(LRC, LOW); // Turn Off Red traffic light 1
  digitalWrite(LAC, LOW); // Turn Off Yellow traffic light 1
  digitalWrite(LVC, LOW); // Turn Off Green traffic light 1
  digitalWrite(LRT, LOW); // Turn Off Red traffic light 2
  digitalWrite(LAT, LOW); // Turn Off Yellow traffic light 2
  digitalWrite(LVT, LOW); // Turn Off Green traffic light 2

  // Communications
  Serial.begin(9600); // Start Serial communications with computer via Serial0 (TX0 RX0) at 9600 bauds
  // lcd.begin();  //Start communications with LCD display
  // lcd.backlight();  //Turn on LCD backlight
  // Serial.println("In order to test Analog Sensors and Actuators, send the following commands through the Serial Monitor:");
  // Serial.println("A: Tests LDR1 Sensor");
  // Serial.println("B: Tests LDR2 Sensor");
  // Serial.println("C: Test CO2 Sensor");
  // Serial.println("D: Tests Traffic Light 1 (LR1, LY1 and LG1)");
  // Serial.println("E: Tests Traffic Light 2 (LR2, LY2 and LG2)");
  // Serial.println("F: Tests LCD");
}

void loop()
{
  MEF_SM1();
  MEF_SM2();
}
