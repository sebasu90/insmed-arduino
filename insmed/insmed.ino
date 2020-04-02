/**
  TO DO:
  Limit inputs
*/

#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1015.h>
#include <AccelStepper.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

Adafruit_ADS1115 ads(0x48);

// Pin definitions
#define encoderPinA 2 // Throttle
#define encoderPinB 3
#define buttonPin 4

#define sensorPin 11 // Inductive sensor to control motor range
#define startButton 12 // Start switch

// Motor outputs
#define pulsePin A0
#define dirPin A2
#define enPin A1
#define alarmPin 7

bool alarmaSensor = LOW;
bool alarmaPresionAlta = LOW;
bool alarmaPresionBaja = LOW;
bool alarmaAmbu = LOW;
bool alarmas = LOW;
bool oldAlarmas = LOW; // Rising edge to clear screen

bool alarmaSensorOldValue = LOW;
bool alarmaPresionAltaOldValue = LOW;
bool alarmaPresionBajaOldValue = LOW;

float pressMinLimit = 5.0;
float pressMaxLimit = 40.0;

float pressMinMovil = 0.0;
float pressMaxMovil = 0.0;

bool resetAlarmas = LOW;

bool hysterisis = LOW;

byte contadorAlarmaPresionBaja = 0;

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
bool checkSensor = LOW;

long t1;

#define lcdTimer 400
#define serialTimer 100
#define buttonTimer 130
#define changeScreenTimer 2000

byte index = 0;
byte FSM;

int maxPosition = 800;
int inhaleSpeed = 500;
int exhaleSpeed = 600;

bool startCycle = LOW;

#define maxnumCiclos 60000
long numCiclos; // Not so frequecnt EEPROM write
byte updatenumCiclos = 0;

//String bufferString;

float maxPressure;  // Serial
float maxPressure2; // LCD
float peepPressure = 0.0; // LCD

//float compliance;
//float Volumen;

// Process Variables

float inhaleTime = 0.0;
float exhaleTime = 0.0;

float presControl;
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

  pinMode(enPin, OUTPUT);
  pinMode(alarmPin, OUTPUT);

  digitalWrite(encoderPinA, HIGH); //turn pullup resistor on
  digitalWrite(encoderPinB, HIGH); //turn pullup resistor on

  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);

  cli();//stop interrupts
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

  //  maxPulses = readEncoderValue(4);
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
  delay(2000);

  offsetPresion = ads.readADC_SingleEnded(0);

  lcd.clear();

  lcd.setCursor(0, 1);
  lcd.print(F("    CALIBRACION"));
  lcd.setCursor(0, 2);
  lcd.print(F("     TERMINADA"));

  delay(1000);

  cargarLCD ();
  contCursor2 = 2;

}  //Fin del Setup

/*******************************************************/

/*******************( LOOP )****************************/

void loop()
{

  //  if (contadorAlarmaSensor > 2) {
  //    alarmaSensor = HIGH;
  //  }
  //
  //  if (contadorAlarmaPresionAlta > 2) {
  //    alarmaPresionAlta = HIGH;
  //  }
  //
  //  if (contadorAlarmaPresionBaja > 2) {
  //    alarmaPresionBaja = HIGH;
  //  }


  // Imprimir Serial ////////////////////////////////////

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

    Serial.print(setPressure * 1.1);
    Serial.print("\t");
    Serial.print(setPressure * 0.9);
    Serial.print("\t");
    Serial.print(maxPressure);
    maxPressure = 0.0;
    Serial.print("\t");
    Serial.println(setPressure);

    //        Serial.print(contadorAlarmaPresionBaja);
    //        Serial.print('\t');
    //    Serial.print(contCursor);
    //    Serial.print('\t');
    //    Serial.print(oldAlarmas);
    //    Serial.print('\t');
    //    Serial.print(alarmas);
    //    Serial.print('\t');
    //    Serial.print(digitalRead(alarmPin));
    //    Serial.print('\t');
    //    Serial.println(alarmaAmbu);

    contadorLectura = millis();
  }

  if (alarmaSensor || alarmaPresionAlta || alarmaPresionBaja || alarmaAmbu) {
    digitalWrite(alarmPin, HIGH);
    alarmas = HIGH;
  }
  else
    digitalWrite(alarmPin, LOW);

  if (alarmas && !oldAlarmas) {
    lcd.clear();
    lcd.noCursor();
    contCursor = 0;
    oldAlarmas = HIGH;
  }

  // Refrescar LCD // Solo se hace en Stop o al fin del ciclo

  if (((FSM == 2) && refreshLCD) || !startCycle) {
    if ((millis() - contadorLCD) > lcdTimer) { // Refresh LCD
      contadorLCD = millis();

      if (alarmas) {
        if (alarmaPresionAlta) {
          lcd.setCursor(0, 0);
          lcd.print(F("    PRESION ALTA"));
          if ((pressureRead < pressMaxLimit) && (pressureRead < pressMaxMovil))
            alarmaPresionAlta = LOW;
        }
        if (alarmaPresionBaja) {
          lcd.setCursor(0, 1);
          lcd.print(F("    PRESION BAJA"));
        }
        if (alarmaSensor) {
          lcd.setCursor(0, 2);
          lcd.print(F(" ATENCION MECANISMO"));
        }

        if (alarmaAmbu) {
          lcd.setCursor(0, 3);
          lcd.print(F("    CAMBIO AMBU"));
        }
      }

      else {

        presControl = readEncoderValue(1);
        inhaleTime = readEncoderValue(2) / 10.0;
        exhaleTime = readEncoderValue(3) / 10.0;

        //          lcd.setCursor(12, 1);
        //          lcd.print("        ");
        //          lcd.setCursor(12, 1);
        //          lcd.print(setPressure, 1);
        //
        //          lcd.setCursor(12, 2);
        //          lcd.print("        ");
        //          lcd.setCursor(12, 2);
        //          lcd.print(inhaleTime, 2);
        //
        //          lcd.setCursor(12, 3);
        //          lcd.print("        ");
        //          lcd.setCursor(12, 3);
        //          lcd.print(exhaleTime, 2);

        lcd.setCursor(5, 3);
        lcd.print(F("   "));
        lcd.setCursor(5, 3);
        lcd.print(int(60 / (inhaleTime + exhaleTime)));  // BPM

        lcd.setCursor(17, 1);
        lcd.print(inhaleTime, 1);


        lcd.setCursor(7, 2);
        float ratio = float(exhaleTime) / float(inhaleTime); // I:E
        lcd.print(ratio, 1);
        lcd.setCursor(17, 2);
        lcd.print(exhaleTime, 1);

        lcd.setCursor(5, 0);
        lcd.print(F("    "));
        lcd.setCursor(5, 0);
        lcd.print(maxPressure2, 1); // PIP

        lcd.setCursor(17, 0);
        lcd.print("  ");

        lcd.setCursor(17, 0);
        lcd.print(presControl, 0);

        // Aca iban //

        //////////////

        lcd.setCursor(5, 1);
        lcd.print("    ");
        lcd.setCursor(5, 1); // PEEP
        lcd.print(peepPressure, 1);

        lcd.setCursor(14, 3);
        lcd.print("      ");
        lcd.setCursor(14, 3);
        lcd.print(numCiclos);


        //        maxPulses = readEncoderValue(4);
        //        inhaleSpeed = readEncoderValue(5);
        //        exhaleSpeed = readEncoderValue(6);
        //        lowSpeed = readEncoderValue(7);

        if (contCursor == 1) {
          lcd.setCursor(17, 0);
        }

        if (contCursor == 2) {
          lcd.setCursor(17, 1);
        }

        if (contCursor == 3) {
          lcd.setCursor(17, 2);
        }

        refreshLCD = LOW;
      } // If no Alarmas
    } // If refreshLCD
  }


  if (!isButtonPushDown()) { // Anti-Bounce Button Switch
    contadorBoton = millis();
    contadorBoton2 = millis();
  }

  //  if (resetAlarmas) {
  //    contCursor = 0;
  //    cargarLCD ();
  //    resetAlarmas = LOW;
  //  }

  if (((millis() - contadorBoton2) > changeScreenTimer)) {
    contadorBoton2 = millis();
    if (alarmaAmbu) {
      numCiclos = 0;
      updatenumCiclos = 0;
      EEPROM.put(80, numCiclos);
      alarmaAmbu = LOW;
    }
    alarmas = LOW;
    oldAlarmas = LOW;
    contCursor = 0;
    cargarLCD ();
  }

  //      if (contCursor2 == 0) {
  //        contCursor2 = 1;
  //        contCursor = 0;
  //        lcd.clear();
  //        lcd.setCursor(0, 0);
  //        lcd.print(F("MAX VOL:"));
  //        lcd.setCursor(0, 1);
  //        lcd.print(F("UP SPEED:"));
  //        lcd.setCursor(0, 2);
  //        lcd.print(F("DN SPEED:"));
  //        lcd.setCursor(0, 3);
  //        lcd.print(F("LOW SPEED:"));
  //      }
  //
  //      else if (contCursor2 == 1) {
  //        contCursor2 = 2;
  //        contCursor = 0;
  //        lcd.noBlink();
  //        cargarLCD ();
  //        /////////////////////////////////
  //
  //      }
  //
  //      else if (contCursor2 == 2) {
  //        contCursor2 = 0;
  //        contCursor = 0;
  //        lcd.clear();
  //        lcd.setCursor(0, 0);
  //        lcd.print(F("ASSISTED VENTILATOR"));
  //        lcd.setCursor(0, 1);
  //        lcd.print(F("SP cmH2O:"));
  //        lcd.setCursor(0, 2);
  //        lcd.print(F("t INHA:"));
  //        lcd.setCursor(0, 3);
  //        lcd.print(F("t EXHA:"));
  //      }
  //    } // End if Change Screen

  if (((millis() - contadorBoton) > buttonTimer) && !alarmas) {
    contadorBoton = millis();
    if (contCursor == 0) {
      lcd.cursor();
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
      lcd.noCursor();
      contCursor = 0;
    }
  } // End If Button Switch


  if (digitalRead(startButton)) {
    digitalWrite(enPin, HIGH); // disable motor
  }

  if (!digitalRead(startButton) || startCycle) { // Start
    startCycle = HIGH;
    digitalWrite(enPin, LOW); // Enable motor

    pressureRead = readPressure(); // Once per cycle

    if (numCiclos > maxnumCiclos)
      alarmaAmbu = HIGH;

    // Save the greatest value to monitor (Peaks)

    if (pressureRead > maxPressure2)
      maxPressure2 = pressureRead;

    if (pressureRead > maxPressure)
      maxPressure = pressureRead;

    if (pressureRead < peepPressure)
      peepPressure = pressureRead;

    if (FSM != 2) {
      setPressure = presControl + peepPressure;
      pressMinMovil = setPressure * 0.8;
      pressMaxMovil = setPressure * 1.2;
    }

    if ((pressureRead > pressMaxLimit) || (pressureRead > pressMaxMovil))
      alarmaPresionAlta = HIGH;

    switch (FSM) {
      case 0:
        contadorCiclo = millis();
        FSM = 1;
        motor.setMaxSpeed(inhaleSpeed);
        motor.moveTo(maxPosition);
        maxPressure2 = 0.0;
        hysterisis = LOW;
        break;

      case 1: // Inhalation Cycle

        //        if ((pressureRead < (setPressure * 0.95)) && hysterisis) {    // Reemplazar por PID
        //          motor.setMaxSpeed(20);
        //          motor.moveTo(maxPosition);
        //          hysterisis = LOW;
        //        }

        if (pressureRead > (setPressure)) {
          motor.setMaxSpeed(0);
          motor.stop();
          hysterisis = HIGH;
        }

        if (((millis() - contadorCiclo) >= int(inhaleTime * 1000)) || alarmaPresionAlta) { // Condition to change state
          motor.stop();
          contadorCiclo = millis();
          FSM = 2;
          refreshLCD = LOW;
          motor.setMaxSpeed(exhaleSpeed);
          motor.moveTo(-100);
          peepPressure = 99.0;
        }

        break;

      case 2: // Exhalation Cycle
        if (!digitalRead(sensorPin)) {
          motor.stop();
          motor.setCurrentPosition(0);
          refreshLCD = HIGH;
          if (alarmaSensor)
            alarmaSensor = LOW;

          if (!checkSensor) {
            if (maxPressure2 < pressMinLimit)
              contadorAlarmaPresionBaja++;
            else
              contadorAlarmaPresionBaja = 0;

            if (contadorAlarmaPresionBaja > 1)
              alarmaPresionBaja = HIGH;
            else
              alarmaPresionBaja = LOW;
          }
          checkSensor = HIGH;
        }

        if ((millis() - contadorCiclo) >= int(exhaleTime * 1000)) {
          FSM = 0;
          checkSensor = LOW;
          contadorCiclo = millis();
          numCiclos++;
          updatenumCiclos++;
          if (updatenumCiclos > 50) { // Solo actualizo la EEPROM cada 50 ciclos.
            EEPROM.put(80, numCiclos);
            updatenumCiclos = 0;
          }
          if (digitalRead(sensorPin))
            alarmaSensor = HIGH;
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

void cargarLCD () {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(F("PIP "));
  lcd.setCursor(12, 0);
  lcd.print(F("PRES"));
  lcd.setCursor(0, 1);
  lcd.print(F("PEEP "));
  lcd.setCursor(12, 1);
  lcd.print(F("tIN "));
  lcd.setCursor(12, 2);
  lcd.print(F("tEX "));
  lcd.setCursor(0, 2);
  lcd.print(F("I:E  1:"));
  lcd.setCursor(0, 3);
  lcd.print(F("BPM "));
  lcd.setCursor(12, 3);
  lcd.print(F("# "));
}


/*******************************************************/
