#include "rgb_lcd.h"
rgb_lcd lcd;

//=================================================
// Latch Variables

uint8_t counterLatch = 0; //Latch for Button
uint8_t convertCounter = 0; //Default Button presses

uint8_t modeLatch = 0; //Latch for modeButton
uint8_t modeCounter = 1; //mode Button presses

double factor = 60.6;
double rawData = 0;

//=================================================
// Potentiometer Pin

uint8_t PotmeterInputPin = 11;
uint16_t sensorValue;

//=================================================
// Fan Variables

double InitialFanSpeed = 0;
uint8_t FanInputPin = 10;

int PWMFreq = 5000;
int PWMChannel = 0;
int PWMResolution = 10;
int MAX_DUTY_CYCLE = 1023;

//=================================================

uint8_t RedInputPin = 4;
uint8_t GreenInputPin = 5;
uint8_t BlueInputPin = 6;

//==================================================================
//                                                         Functions

uint16_t rangeFinder( uint8_t placeholder )
{
  uint16_t converted = 0;

  pinMode(7,OUTPUT);

  digitalWrite(7,LOW);
  digitalWrite(7,HIGH);
  delayMicroseconds(5);
  digitalWrite(7,LOW);

  pinMode(7,INPUT); //Set to read
  rawData = pulseIn(7,HIGH);  //Reads the data

  converted = rawData / factor;

  return converted;
}

//=================================================

double inverter ( double number )
{
  double inverted;

  if (number > 0 )
  {
    inverted = 1 / InitialFanSpeed;
    return inverted;
  }
  
}

//=================================================

void result ()
{

  if (modeCounter % 2 == 0)   // Safety Mode
  {
    lcd.setCursor(0,0);
    lcd.print("Safety Mode");


    lcd.setCursor(0,1);
    lcd.print("Spd:");
    lcd.setCursor(4,1);
    lcd.print(sensorValue/10);

    lcd.setCursor(7,1);
    lcd.print("DIS:");
    lcd.setCursor(11,1);
    lcd.print(rangeFinder(1));

  }
  else // Normal Operating Mode
  {
    lcd.setCursor(0,0);
    lcd.print("Manual Mode");

    lcd.setCursor(0,1);
    lcd.print("Speed:");
    lcd.setCursor(6,1);
    lcd.print(sensorValue/10);

    //=======================
    // Button Press Display
    lcd.setCursor(11,1);
    lcd.print("BP:");
    lcd.setCursor(14,1);
    lcd.print(modeCounter-1);

  //=======================


  }

  // Screen wipes
  delay(100);
  lcd.clear();

}

//==================================================================
//                                                             Setup

void setup() 
{
  lcd.begin(16,2);
  lcd.blink();

  pinMode(1,INPUT);  // Mode Button Input

  Serial.begin(9600); // Pot.Meter Signal Format
  analogReadResolution(PWMResolution); // Flatten Pot.Meter

  ledcSetup(PWMChannel, PWMFreq, PWMResolution); // Fan Setup
  ledcAttachPin(FanInputPin, PWMChannel); // Fan Signal
  ledcWrite(PWMChannel, 0 ); // Prevent Startup Full Speed

  ledcSetup(1, PWMFreq, PWMResolution); // Red Setup
  ledcAttachPin(RedInputPin, 1); // Red Signal

  ledcSetup(2, PWMFreq, PWMResolution); // Red Setup
  ledcAttachPin(GreenInputPin, 2); // Red Signal

  ledcSetup(3, PWMFreq, PWMResolution); // Red Setup
  ledcAttachPin(BlueInputPin, 3); // Red Signal


}
//==================================================================
//                                                              Loop
void loop() 
{

  uint8_t modePress = digitalRead(1); // Reads the Mode button input

  //===========================
  // Mode Button Latch
  if (modePress == 1 && modeLatch == 0)
  {
    modeCounter = modeCounter + 1;
    modeLatch = 1;
  }
  else if (modePress == 0)
  {
    modeLatch = 0;
  }

  //===========================
  // Mode CrossRoad

  if (modeCounter % 2 == 0)   // Safety Mode
  {
    ledcWrite(PWMChannel, analogRead(PotmeterInputPin) - (rangeFinder(1)) );

    if(rangeFinder(1) > 10)
    {
      ledcWrite(1, 0); // Red Light
      ledcWrite(2, 1000); // Green Light
      ledcWrite(3, 0); // Blue Light
    }
    else if (rangeFinder(1) <= 10 && rangeFinder(1) >= 5)
    {
      ledcWrite(1, 400); // Red Light
      ledcWrite(2, 600); // Green Light
      ledcWrite(3, 0); // Blue Light
    }
    else if (rangeFinder(1) <= 5 )
    {
      ledcWrite(1, 1000); // Red Light
      ledcWrite(2, 0); // Green Light
      ledcWrite(3, 0); // Blue Light
    }
    else
    {
      ledcWrite(1, 0); // Red Light
      ledcWrite(2, 0); // Green Light
      ledcWrite(3, 0); // Blue Light
    }


  }
  else // Normal Operating Mode
  {
    InitialFanSpeed = analogRead(PotmeterInputPin);
    // Push Pot.Meter value to Motor
    ledcWrite(PWMChannel, analogRead(PotmeterInputPin));

    // Push Pot.Meter value to RGB Light
    ledcWrite(1, analogRead(PotmeterInputPin)); // Red Light
    ledcWrite(2, analogRead(PotmeterInputPin)); // Green Light
    ledcWrite(3, analogRead(PotmeterInputPin)); // Blue Light
  }

  result();

  //===========================
  // Internal Pot.Meter value display
  sensorValue = analogRead(PotmeterInputPin);
  Serial.println(sensorValue);

}