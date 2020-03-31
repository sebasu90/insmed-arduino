/**
  TO DO:
  Limit inputs
*/

#include <EEPROM.h>
#include <Wire.h>  // Libreria de comunicacion I2C/TWI de Arduino IDE
#include <LiquidCrystal_I2C.h>  // Libreria LCD-I2C de fmalpartida
//#include <SoftwareSerial.h>
#include <Adafruit_ADS1015.h>
#include <AccelStepper.h>

//#include <MemoryUsage.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

Adafruit_ADS1115 ads(0x48);

// Pin definitions
#define encoderPinA 2 // Throttle
#define encoderPinB 3
#define buttonPin 4

#define offsetLevas 49.5
#define coeff 1.0511

//#define BT_Rx 5     // BT
//#define BT_Tx 6

#define sensorPin 11 // Inductive sensor to control motor range
#define startButton 12 // Start switch

// Motor outputs
#define pulsePin A0
#define dirPin A2
#define enPin A1
#define alarmPin 7

bool hysterisis = LOW;

bool alarmaSensor = LOW;
bool alarmaPresionAlta = LOW;
bool alarmaPresionBaja = LOW;

int contadorAlarmaSensor = 0;
int contadorAlarmaPresionAlta = 0;
int contadorAlarmaPresionBaja = 0;

//SoftwareSerial BT (BT_Rx, BT_Tx);
AccelStepper motor(AccelStepper::DRIVER, pulsePin, dirPin);

volatile int lastEncoded = 0;
volatile int encoderValue[10];
long lastencoderValue = 0;

int16_t adc0; // ADS1015 reading

// Manage the LDC cursor and screens

byte contCursor = 0;
byte contCursor2 = 0;

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
long contadorHorometro = 0;

bool refreshLCD = LOW;

long t1;

#define lcdTimer 400
#define serialTimer 100
#define buttonTimer 130
#define changeScreenTimer 2000

byte index = 0;
byte FSM;

//int pulsePerRevolution = 400; // Set in the stepper driver
//int maxpulses = 600; // Mechanical limit of the CAM
//int currentPulses = 0;
//int maxFrec = 10000;
//int numInterrupts;
int maxPosition;

//int delaymicros;

//int inhaleSpeed = 60; // 1-100
//int exhaleSpeed = 400; // 1-100

//int lowSpeed = 200;

bool startCycle = LOW;

long numCiclos; // Not so frequecnt EEPROM write
byte updatenumCiclos = 0;

//String bufferString;

float maxPressure;  // Serial
float maxPressure2; // LCD
float maxPressure3;
float compliance;
float Volumen;
// Process Variables

float inhaleTime = 0.0;
float exhaleTime = 0.0;

float setPressure = 0.0;
float pressure = 0.0;
float pressureRead = 0;
long offsetPresion = 0;

int readEncoderValue(byte index) {
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

  //  pinMode(pulsePin, OUTPUT);
  //  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  pinMode(alarmPin, OUTPUT);

  //  motor.setPinsInverted(HIGH); // Depende del prototipo

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

//  BT.begin(9600);
  ads.begin();

  // Read from EEPROM the machine's parameters

  EEPROM.get(10, encoderValue[0]);
  EEPROM.get(20, encoderValue[1]);
  EEPROM.get(30, encoderValue[2]);
  EEPROM.get(40, encoderValue[3]);
  EEPROM.get(50, encoderValue[4]);
  EEPROM.get(60, encoderValue[5]);
  EEPROM.get(70, encoderValue[6]);
  EEPROM.get(80, numCiclos);

  setPressure = readEncoderValue(1) / 10.0;
  inhaleTime = readEncoderValue(2) / 10.0;
  exhaleTime = readEncoderValue(3) / 10.0;

  //  readEncoderValue(4) = readEncoderValue(4);
  //  inhaleSpeed = readEncoderValue(5);
  //  exhaleSpeed = readEncoderValue(6);
  //  lowSpeed = readEncoderValue(7);

  motor.setAcceleration(4000.0); // To test
  motor.setMinPulseWidth(500);

  lcd.setCursor(0, 0);
  lcd.print(F(" CALIBRACION SENSOR"));
  lcd.setCursor(0, 2);
  lcd.print(F("  ASEGURE PRESION"));
  lcd.setCursor(0, 3);
  lcd.print(F("     AMBIENTE"));
  delay(3000);

  offsetPresion = ads.readADC_SingleEnded(0);

  //  Serial.println(offsetPresion);

  lcd.clear();

  lcd.setCursor(0, 1);
  lcd.print(F("    CALIBRACION"));
  lcd.setCursor(0, 2);
  lcd.print(F("     TERMINADA"));

  delay(1000);

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(F("ASSISTED VENTILATOR"));
  lcd.setCursor(0, 1);
  lcd.print(F("SP cmH2O:"));
  lcd.setCursor(0, 2);
  lcd.print(F("t INHA:"));
  lcd.setCursor(0, 3);
  lcd.print(F("t EXHA:"));

}  //Fin del Setup

/*******************************************************/

/*******************( LOOP )****************************/

void loop()
{

  if (contadorAlarmaSensor > 3) {
    alarmaSensor = HIGH;
  }

  if (contadorAlarmaPresionAlta > 2) {
    alarmaPresionAlta = HIGH;
  }

  if (contadorAlarmaPresionBaja > 2) {
    alarmaPresionBaja = HIGH;
  }



  if ((millis() - contadorLectura) > serialTimer) {
    //    Protocolo BlueTooth
    //
    //    outputString = 't';
    //    outputString += millis();
    //    outputString += 'p';
    //    outputString += maxPressure;
    //    outputString += 'v';
    //    outputString += motor.currentPosition();
    //    outputString += ';';
    //    BT.print(outputString);

    //    Serial.print(80 + motor.currentPosition() / 10);
    //    Serial.print("\t");

    Serial.print(setPressure * 1.05);
    Serial.print("\t");
    Serial.print(setPressure * 0.95);
    Serial.print("\t");
    Serial.print(maxPressure);
    maxPressure = 0.0;
    Serial.print("\t");
    //    Serial.print(FSM);
    Serial.println(setPressure);

    contadorLectura = millis();
  }

  // Check execution Times;

  //    Serial.println(micros() - t1);
  //    t1 = micros();

  //  MEMORY_PRINT_START
  //  MEMORY_PRINT_HEAPSTART
  //  MEMORY_PRINT_HEAPEND
  //  MEMORY_PRINT_STACKSTART
  //  MEMORY_PRINT_END
  //  MEMORY_PRINT_HEAPSIZE
  //
  //  Serial.println();
  //  Serial.println();
  //
  //  FREERAM_PRINT;

  if (refreshLCD || !startCycle) {
    if ((millis() - contadorLCD) > lcdTimer) { // Refresh LCD

      contadorLCD = millis();

      if (contCursor2 == 0) { // Pantalla 1

        setPressure = readEncoderValue(1) / 10.0;
        inhaleTime = readEncoderValue(2) / 10.0;
        exhaleTime = readEncoderValue(3) / 10.0;

        lcd.setCursor(12, 1);
        lcd.print("        ");
        lcd.setCursor(12, 1);
        lcd.print(setPressure, 1);

        lcd.setCursor(12, 2);
        lcd.print("        ");
        lcd.setCursor(12, 2);
        lcd.print(inhaleTime, 2);

        lcd.setCursor(12, 3);
        lcd.print("        ");
        lcd.setCursor(12, 3);
        lcd.print(exhaleTime, 2);

        if (contCursor == 1) {
          lcd.setCursor(12, 1);
        }

        if (contCursor == 2) {
          lcd.setCursor(12, 2);
        }

        if (contCursor == 3) {
          lcd.setCursor(12, 3);
        }

      } // End if pantalla 1

      else if (contCursor2 == 2) { // Pantalla 2
        if (FSM == 2) {
          if (maxPressure2 <= 5.0)
            compliance = 0.0;
          else
            compliance = Volumen / maxPressure3;
        }

        //        if (maxPosition < offsetLevas)
        //          Volumen = 0;
        //        else
        Volumen = coeff * maxPosition - offsetLevas;

        lcd.setCursor(6, 0);
        lcd.print("     ");
        lcd.setCursor(6, 0);
        lcd.print(maxPressure2, 1);
        //        maxPressure2 = 0.0;

        lcd.setCursor(15, 0);
        lcd.print("     ");

        lcd.setCursor(15, 0);
        lcd.print(compliance, 1);

        // Aca iban //

        //////////////

        lcd.setCursor(4, 3);
        lcd.print("   ");
        lcd.setCursor(4, 3);
        lcd.print(Volumen, 0);
        lcd.setCursor(14, 3);
        lcd.print("      ");
        lcd.setCursor(14, 3);
        lcd.print(numCiclos);


      } // End if pantalla 1

      else if (contCursor2 == 1) { // Pantalla 2

        //        maxPulses = readEncoderValue(4);
        //        inhaleSpeed = readEncoderValue(5);
        //        exhaleSpeed = readEncoderValue(6);
        //        lowSpeed = readEncoderValue(7);

        lcd.setCursor(10, 0);
        lcd.print("          ");
        lcd.setCursor(10, 0);
        lcd.print(readEncoderValue(4));

        lcd.setCursor(10, 1);
        lcd.print("          ");
        lcd.setCursor(10, 1);
        lcd.print(readEncoderValue(5));

        lcd.setCursor(10, 2);
        lcd.print("          ");
        lcd.setCursor(10, 2);
        lcd.print(readEncoderValue(6));

        lcd.setCursor(10, 3);
        lcd.print("          ");
        lcd.setCursor(10, 3);
        lcd.print(readEncoderValue(7) / 4);

        if (contCursor == 4) {
          lcd.setCursor(10, 0);
        }

        if (contCursor == 5) {
          lcd.setCursor(10, 1);
        }

        if (contCursor == 6) {
          lcd.setCursor(10, 2);
        }

        if (contCursor == 7) {
          lcd.setCursor(10, 3);
        }

      } // End if pantalla 1

    } // End refresh LCD Timer
    refreshLCD = LOW;
  } // If refreshLCD


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
      lcd.print(F("MAX VOL:"));
      lcd.setCursor(0, 1);
      lcd.print(F("UP SPEED:"));
      lcd.setCursor(0, 2);
      lcd.print(F("DN SPEED:"));
      lcd.setCursor(0, 3);
      lcd.print(F("LOW SPEED:"));
    }

    else if (contCursor2 == 1) {
      contCursor2 = 2;
      contCursor = 0;
      lcd.noBlink();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("cmH2O:"));
      lcd.setCursor(12, 0);
      lcd.print(F("LC:"));
      lcd.setCursor(0, 1);
      lcd.print(F("BPM:"));
      lcd.setCursor(12, 1);
      lcd.print(F("tIN:"));
      lcd.setCursor(0, 2);
      lcd.print(F("I:E: 1:"));
      lcd.setCursor(12, 2);
      lcd.print(F("tEX:"));
      lcd.setCursor(0, 3);
      lcd.print(F("Vt:"));
      lcd.setCursor(12, 3);
      lcd.print(F("#:"));

      // Estos iban arriba ///////////

      lcd.setCursor(6, 1);
      lcd.print("    ");
      lcd.setCursor(6, 1);
      lcd.print(int(60 / (inhaleTime + exhaleTime)));

      lcd.setCursor(17, 1);
      lcd.print(inhaleTime, 1);

      lcd.setCursor(7, 2);
      lcd.print("    ");
      lcd.setCursor(7, 2);
      float ratio = float(exhaleTime) / float(inhaleTime);
      lcd.print(ratio, 2);
      lcd.setCursor(17, 2);
      lcd.print(exhaleTime, 1);

      /////////////////////////////////

    }

    else if (contCursor2 == 2) {
      contCursor2 = 0;
      contCursor = 0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("ASSISTED VENTILATOR"));
      lcd.setCursor(0, 1);
      lcd.print(F("SP cmH2O:"));
      lcd.setCursor(0, 2);
      lcd.print(F("t INHA:"));
      lcd.setCursor(0, 3);
      lcd.print(F("t EXHA:"));
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
        contCursor = 7;
      }

      else if (contCursor == 7) {
        EEPROM.put(contCursor * 10, encoderValue[contCursor - 1]);
        lcd.noBlink();
        contCursor = 0;
      }
    }
  }

  pressureRead = readPressure(); // Once per cycle

  // Save the greatest value to monitor (Peaks)

  if (pressureRead > maxPressure2)
    maxPressure2 = pressureRead;

  if (pressureRead > maxPressure)
    maxPressure = pressureRead;

  if (pressureRead > maxPressure3)
    maxPressure3 = pressureRead;

  if (digitalRead(startButton))
    digitalWrite(enPin, HIGH); // disable motor

  if (!digitalRead(startButton) || startCycle) { // Start
    startCycle = HIGH;
    digitalWrite(enPin, LOW); // Enable motor

    switch (FSM) {
      case 0:
        contadorCiclo = millis();
        FSM = 1;
        //        digitalWrite(dirPin, LOW); // Up
        motor.setMaxSpeed(readEncoderValue(5) * 5);
        motor.moveTo(readEncoderValue(4));
        maxPressure2 = 0.0;
        //        hysterisis = LOW;
        break;

      case 1: // Inhalation Cycle

        if ((setPressure*0.95) < pressureRead) {
          motor.setMaxSpeed(0);
          motor.stop();
        }

        if ((millis() - contadorCiclo) >= int(inhaleTime * 1000)) { // Condition to change state
          motor.stop();
          contadorCiclo = millis();
          maxPosition = motor.currentPosition();
          FSM = 2;
          hysterisis = LOW;
          motor.setMaxSpeed(readEncoderValue(6) * 5);
          motor.moveTo(-100);
          //          digitalWrite(dirPin, HIGH);
        }

        break;

      case 2: // Exhalation Cycle
        if (!digitalRead(sensorPin)) {
          motor.stop();
          motor.setCurrentPosition(0);
          contadorAlarmaSensor = 0;
          refreshLCD = HIGH;
        }

        if ((millis() - contadorCiclo) >= int(exhaleTime * 1000)) {
          FSM = 0;
          contadorCiclo = millis();
          numCiclos++;
          updatenumCiclos++;
          if (updatenumCiclos > 50) { // Solo actualizo la EEPROM cada 50 ciclos.
            EEPROM.put(80, numCiclos);
            updatenumCiclos = 0;
          }
          if (digitalRead(sensorPin))
            contadorAlarmaSensor++;
          if (digitalRead(startButton))
            startCycle = LOW;
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
