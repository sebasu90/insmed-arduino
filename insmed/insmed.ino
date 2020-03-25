/**

  TO DO:

  Limit inputs
  Read pressure sensor faster
  Send pressure BT and timestamp

  Button switch change up

*/

#include <EEPROM.h>
#include <Wire.h>  // Libreria de comunicacion I2C/TWI de Arduino IDE
#include <LiquidCrystal_I2C.h>  // Libreria LCD-I2C de fmalpartida
#include <SoftwareSerial.h>
#include <Adafruit_ADS1015.h>
#include <AccelStepper.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

Adafruit_ADS1115 ads(0x48);

// Pin definitions
int encoderPinA = 2; // Throttle
int encoderPinB = 3;
int buttonPin = 4;

int BT_Rx = 5;      // BT
int BT_Tx = 6;

int sensorPin = 11; // Inductive sensor to control motor range
int startButton = 12; // Start switch

// Motor outputs
int pulsePin = A0;
int dirPin = A2;
int enPin = A1;

bool hysterisis = LOW;

long frecTimer;

SoftwareSerial BT (BT_Rx, BT_Tx);
AccelStepper motor(AccelStepper::DRIVER, pulsePin, dirPin);

long timerOn = 0;

volatile int lastEncoded = 0;
volatile int encoderValue[6];
long lastencoderValue = 0;

int16_t adc0; // ADS1015 reading

int lastMSB = 0;
int lastLSB = 0;

// Manage the LDC cursor and screens

int contCursor = 0;
int contCursor2 = 0;

long currentTime;

bool Start = false;

bool botonAnterior;

bool Read = LOW;

bool estadoPinA, estadoPinB;

bool estadoBoton = false;
long contadorBoton = 0;
long contadorLCD = 0;
long contadorBoton2 = 0;
long contadorCiclo = 0;
long contadorControl = 0;
long contadorLectura = 0;
long contadorLecturapresion = 0;

int lcdTimer = 300;
int serialTimer = 50;
int buttonTimer = 130;
int changeScreenTimer = 3000;

int index = 0;
int FSM;

int pulsePerRevolution = 400; // Set in the stepper driver
int maxPulses = 600; // Mechanical limit of the CAM
int currentPulses = 0;
int maxFrec = 10000;
int numInterrupts;
int maxPosition;

int delaymicros;

int inhaleSpeed = 60; // 1-100
int exhaleSpeed = 400; // 1-100

bool startCycle = LOW;
bool oldValue = LOW;

/*

  Max Speed is 400 RPM
  Max frec = 400 RPM * 400 ppr / 60 = 2.66 kHz ~ 2.5 kHz
  T = 400 us (100% speed)
  Delay Time = 20.000 us / Speed

  Max pulses = Max degrees CAM * 2 rev stepper * 400 Pulses     800 * Max degree CAM
                                 -------------   ----------- =
                                  1 rev CAM     1 rev stepper

*/

// Process Variables

bool cycleON = LOW;

float inhaleTime = 0.0;
float exhaleTime = 0.0;

float setPressure = 0.0;
float pressure = 0.0;
float pressureRead = 0;
long offsetPresion = 0;

float error = 0.05;

int readEncoderValue(int index) {
  return ((encoderValue[index - 1] / 4));
}

String inputString, outputString;

boolean isButtonPushDown(void) {
  if (!digitalRead(buttonPin)) {
    if (!digitalRead(buttonPin))
      return true;
  }
  return false;
}

/*******************************************************/

/*******************( SETUP )***************************/

void setup()   //Las instrucciones solo se ejecutan una vez, despues del arranque
{
  Serial.begin(115200);

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(startButton, INPUT);
  pinMode(sensorPin, INPUT);

  pinMode(pulsePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  digitalWrite(encoderPinA, HIGH); //turn pullup resistor on
  digitalWrite(encoderPinB, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);

  cli();//stop interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 3200;// = Crystal of 16Mhz / 3200 cycles = 5 kHz Timer 1 frequency
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 for no prescaler
  TCCR1B |= (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts

  lcd.init();
  lcd.clear();
  lcd.backlight();

  estadoPinA = digitalRead(encoderPinA);
  estadoPinB = digitalRead(encoderPinB);

  contadorLCD = millis();
  contadorLectura = millis();

  BT.begin(9600);
  ads.begin();

  // Read from EEPROM the machine's parameters

  EEPROM.get(10, encoderValue[0]);
  EEPROM.get(20, encoderValue[1]);
  EEPROM.get(30, encoderValue[2]);
  EEPROM.get(40, encoderValue[3]);
  EEPROM.get(50, encoderValue[4]);
  EEPROM.get(60, encoderValue[5]);

  setPressure = readEncoderValue(1) / 10.0;
  inhaleTime = readEncoderValue(2) / 10.0;
  exhaleTime = readEncoderValue(3) / 10.0;

  maxPulses = readEncoderValue(4);
  inhaleSpeed = readEncoderValue(5);
  exhaleSpeed = readEncoderValue(6);

  motor.setAcceleration(4000.0); // To test


  lcd.print("                    ");
  lcd.setCursor(0, 0);
  lcd.print(" CALIBRACION SENSOR");
  lcd.setCursor(0, 2);
  lcd.print("  ASEGURE NO TENER");
  lcd.setCursor(0, 3);
  lcd.print("      PRESION");
  delay(5000);

  offsetPresion = ads.readADC_SingleEnded(0);

  //  Serial.println(offsetPresion);

  lcd.clear();

  lcd.setCursor(0, 1);
  lcd.print("    CALIBRACION");
  lcd.setCursor(0, 2);
  lcd.print("     TERMINADA");

  delay(1000);

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("ASSISTED VENTILATOR");
  lcd.setCursor(0, 1);
  lcd.print("cm H2O:");
  lcd.setCursor(0, 2);
  lcd.print("t INHA:");
  lcd.setCursor(0, 3);
  lcd.print("t EXHA:");

}  //Fin del Setup

/*******************************************************/

/*******************( LOOP )****************************/

void loop()
{

  if ((millis() - contadorLectura) > serialTimer) {
    //    timerOn++;
    //    outputString = 't';
    //    outputString += timerOn;
    //    outputString += 'p';
    //    outputString += pressureRead;
    //    outputString += ';';

    //    Serial.print("FrecTimer: ");
    //    Serial.print(frecTimer);
    //    Serial.print("    FSM: ");
    //    Serial.print(FSM);
    //    Serial.print("    Pulses: ");
    Serial.print(setPressure + 2.0);
    Serial.print("\t");
    Serial.print(setPressure - 2.0);
    Serial.print("\t");
    Serial.print(pressureRead);
    Serial.print("\t");
    Serial.println(setPressure);

    contadorLectura = millis();
  }

  if ((millis() - contadorLCD) > lcdTimer) { // Refresh LCD

    contadorLCD = millis();

    if (contCursor2 == 0) { // Pantalla 1

      setPressure = readEncoderValue(1) / 10.0;
      inhaleTime = readEncoderValue(2) / 10.0;
      exhaleTime = readEncoderValue(3) / 10.0;

      lcd.setCursor(8, 1); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print("           ");  //Escribimos texto
      lcd.setCursor(8, 1); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print(setPressure);  //Escribimos texto
      //      lcd.setCursor(14, 1); //Apuntamos a la direccion LCD(caracter,linea)
      //      lcd.print(pressureRead);  //Escribimos texto

      lcd.setCursor(8, 2); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print("          ");  //Escribimos texto
      lcd.setCursor(8, 2); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print(inhaleTime);  //Escribimos texto

      lcd.setCursor(8, 3); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print("          ");  //Escribimos texto
      lcd.setCursor(8, 3); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print(exhaleTime);  //Escribimos texto

      if (contCursor == 1) {
        lcd.setCursor(8, 1); //Apuntamos a la direccion LCD(caracter,linea)
      }

      if (contCursor == 2) {
        lcd.setCursor(8, 2); //Apuntamos a la direccion LCD(caracter,linea)
      }

      if (contCursor == 3) {
        lcd.setCursor(8, 3); //Apuntamos a la direccion LCD(caracter,linea)
      }

    } // End if pantalla 1

    else if (contCursor2 == 2) { // Pantalla 2

      lcd.setCursor(8, 0); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print("          ");  //Escribimos texto
      lcd.setCursor(8, 0); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print(pressureRead);  //Escribimos texto

      lcd.setCursor(8, 1); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print("          ");  //Escribimos texto
      lcd.setCursor(8, 1); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print(int(60 / (inhaleTime + exhaleTime))); //Escribimos texto

      lcd.setCursor(8, 2); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print("          ");  //Escribimos texto
      lcd.setCursor(8, 2); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print(float(exhaleTime) / float(inhaleTime)); //Escribimos texto

      lcd.setCursor(8, 3); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print("          ");  //Escribimos texto
      lcd.setCursor(8, 3); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print(maxPosition); //Escribimos texto

    } // End if pantalla 1

    else if (contCursor2 == 1) { // Pantalla 2

      maxPulses = readEncoderValue(4);
      inhaleSpeed = readEncoderValue(5);
      exhaleSpeed = readEncoderValue(6);

      lcd.setCursor(10, 1); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print("          ");  //Escribimos texto
      lcd.setCursor(10, 1); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print(maxPulses);  //Escribimos texto

      lcd.setCursor(10, 2); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print("          ");  //Escribimos texto
      lcd.setCursor(10, 2); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print(inhaleSpeed);  //Escribimos texto

      lcd.setCursor(10, 3); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print("          ");  //Escribimos texto
      lcd.setCursor(10, 3); //Apuntamos a la direccion LCD(caracter,linea)
      lcd.print(exhaleSpeed);  //Escribimos texto

      if (contCursor == 4) {
        lcd.setCursor(10, 1); //Apuntamos a la direccion LCD(caracter,linea)
      }

      if (contCursor == 5) {
        lcd.setCursor(10, 2); //Apuntamos a la direccion LCD(caracter,linea)
      }

      if (contCursor == 6) {
        lcd.setCursor(10, 3); //Apuntamos a la direccion LCD(caracter,linea)
      }

    } // End if pantalla 1

  } // End refresh LCD


  if (!isButtonPushDown()) { // Anti-Bounce Button Switch
    contadorBoton = millis();
    contadorBoton2 = millis();
  }

  if ((millis() - contadorBoton2) > changeScreenTimer) {
    contadorBoton2 = millis();

    if (contCursor2 == 0) {
      contCursor2 = 1;
      contCursor = 0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("     PARAMETERS");  //Escribimos texto
      lcd.setCursor(0, 1);
      lcd.print("VOLUME:");  //Escribimos texto
      lcd.setCursor(0, 2);
      lcd.print("UP SPEED:");  //Escribimos texto
      lcd.setCursor(0, 3);
      lcd.print("DN SPEED:");  //Escribimos texto
    }

    else if (contCursor2 == 1) {
      contCursor2 = 2;
      contCursor = 0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("cm H2O:");
      lcd.setCursor(0, 1);
      lcd.print("BPM:");
      lcd.setCursor(0, 2);
      lcd.print("E/I:");
      lcd.setCursor(0, 3);
      lcd.print("Vt:");
    }

    else if (contCursor2 == 2) {
      contCursor2 = 0;
      contCursor = 0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ASSISTED VENTILATOR");
      lcd.setCursor(0, 1);
      lcd.print("cm H2O:");
      lcd.setCursor(0, 2);
      lcd.print("t INHA:");
      lcd.setCursor(0, 3);
      lcd.print("t EXHA:");
    }
  }

  if ((millis() - contadorBoton) > buttonTimer) {
    contadorBoton = millis();
    if (contCursor2 == 0) {
      if (contCursor == 0) {
        lcd.blink();
        contCursor = 1;
      }

      else if (contCursor == 1) {
        EEPROM.put(contCursor * 10, encoderValue[contCursor - 1]);
        contCursor = 2;
      }

      else if (contCursor == 2) {
        EEPROM.put(contCursor * 10, encoderValue[contCursor - 1]);
        contCursor = 3;
      }

      else if (contCursor == 3) {
        EEPROM.put(contCursor * 10, encoderValue[contCursor - 1]);
        lcd.noBlink();
        contCursor = 0;
      }
    }

    else if (contCursor2 == 1) {
      if (contCursor == 0) {
        lcd.blink();
        contCursor = 4;
      }

      else if (contCursor == 4) {
        EEPROM.put(contCursor * 10, encoderValue[contCursor - 1]);
        contCursor = 5;
      }

      else if (contCursor == 5) {
        EEPROM.put(contCursor * 10, encoderValue[contCursor - 1]);
        contCursor = 6;
      }

      else if (contCursor == 6) {
        EEPROM.put(contCursor * 10, encoderValue[contCursor - 1]);
        lcd.noBlink();
        contCursor = 0;
      }
    }
  }

  pressureRead = readPressure(); // Once per cycle

  if (digitalRead(startButton))
    digitalWrite(enPin, HIGH); // disable motor

  if (!digitalRead(startButton)) { // Start

    digitalWrite(enPin, LOW); // Enable motor

    switch (FSM) {
      case 0:
        contadorCiclo = millis();
        FSM = 1;
        digitalWrite(dirPin, HIGH); // Up
        motor.setMaxSpeed(inhaleSpeed * 5);
        motor.moveTo(maxPulses);
        break;

      case 1: // Inhalation Cycle
        //        if ((millis() - contadorLecturapresion) > 50) {
        //          contadorLecturapresion = millis();
        //        }

        if (setPressure < pressureRead) {
          motor.stop();
          hysterisis = HIGH;
        }

        if ((pressureRead < setPressure * 0.95) && hysterisis) {
          motor.moveTo(maxPulses);
          hysterisis = LOW;
        }

        if (((motor.currentPosition( ) > maxPulses * 0.9) || (pressureRead > setPressure * 0.85)) && (motor.maxSpeed() > 100.0)) {
          motor.setMaxSpeed(120); //Cte
        }

        //        if ((pressureRead < setPressure * 0.75) && (motor.currentPosition( ) < maxPulses * 0.9)) {
        //          motor.setMaxSpeed(inhaleSpeed * 5); //Cte
        //        }

        if ((millis() - contadorCiclo) >= int(inhaleTime * 1000)) { // Condition to change state
          contadorCiclo = millis();
          maxPosition = motor.currentPosition();
          FSM = 2;
          motor.setMaxSpeed(exhaleSpeed * 5);
          motor.moveTo(-10);
          digitalWrite(dirPin, LOW);
        }

        break;

      case 2: // Exhalation Cycle
        if (!digitalRead(sensorPin)) {
          motor.stop();
          motor.setCurrentPosition(0);
        }

        if ((millis() - contadorCiclo) >= int(exhaleTime * 1000)) {
          FSM = 0;
          contadorCiclo = millis();
        }
        break;

      default:
        break;
    } // End cases
  } // End machine cycle
}  //End Loop

void updateEncoder() {
  int MSB = digitalRead(encoderPinA); //MSB = most significant bit
  int LSB = digitalRead(encoderPinB); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if (contCursor > 0) {

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[contCursor - 1] ++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[contCursor - 1] --;

  }

  lastEncoded = encoded; //store this value for next time
}

float readPressure() {
  adc0 = ads.readADC_SingleEnded(0);
  return (71.38 * (adc0 - offsetPresion) / offsetPresion);
}

ISR(TIMER1_COMPA_vect) {
  motor.run();  // Motor keepalive (at least once per step).
}


/*******************************************************/
