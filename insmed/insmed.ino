/**
  TO DO:
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

#define batteryPin 9
#define eStopPin 10
#define sensorPin 11 // Inductive sensor to control motor range
#define startButton 12 // Start switch

// Motor outputs
#define pulsePin A0
#define dirPin A2
#define enPin A1
#define alarmPin 7

bool goHome = LOW;

bool alarmaSensor = LOW;
bool alarmaSensor2 = LOW;
bool alarmaPresionAlta = LOW;
bool alarmaPresionBaja = LOW;
bool alarmaAmbu = LOW;
bool alarmaBloqueo = LOW;
bool alarmaBateria = LOW;

bool alarmas = LOW;
bool oldAlarmas = LOW; // Rising edge to clear screen

byte numAlarmas = 0;
byte numCol = 0;

bool newAlarm = LOW;

bool alarmaSensorOld = LOW;
bool alarmaPresionAltaOld = LOW;
bool alarmaPresionBajaOld = LOW;
bool alarmaSensor2Old = LOW;
bool alarmaAmbuOld = LOW;
bool alarmaBloqueoOld = LOW;
bool alarmaBateriaOld = LOW;

bool alarmaeStop = LOW;
bool alarmaeStopOld = LOW;

float pressMinLimit = 3.0;
float pressMaxLimit = 41.0;

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
#define buttonTimer 110
#define changeScreenTimer 2000

byte index = 0;
byte FSM;

int maxPosition = 3400;
int inhaleSpeed = 3000;
int exhaleSpeed = 3000;

//int maxPosition = 3500;
//int inhaleSpeed = 3000;
//int exhaleSpeed = 3000;

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
  pinMode(batteryPin, INPUT);
  pinMode(eStopPin, INPUT);

  pinMode(enPin, OUTPUT);
  pinMode(alarmPin, OUTPUT);

  pinMode(pulsePin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(encoderPinA, HIGH); //turn pullup resistor on
  digitalWrite(encoderPinB, HIGH); //turn pullup resistor on

  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);

  cli();//stop interrupts
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 1600;// = Crystal of 16Mhz / 3200 cycles = 5 kHz Timer 1 frequency
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

  motor.setAcceleration(10000.0); // To test
  motor.setMinPulseWidth(50);

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
  //  motor.setCurrentPosition(0);
  //  digitalWrite(enPin, LOW); // Enable motor
  //  delay(50);
  //  goHome = HIGH;
  //  motor.setMaxSpeed(600);
  //  motor.moveTo(-10000);
  t1 = millis();

}  //Fin del Setup

/*******************************************************/

/*******************( LOOP )****************************/

void loop() {

  //  if ((!digitalRead(sensorPin)) && goHome) {
  //    motor.setMaxSpeed(0);
  //    motor.stop();
  //    motor.setCurrentPosition(0);
  //    digitalWrite(enPin, HIGH); // Enable motor
  //    delay(500);
  //    goHome = LOW;
  //  }

  // Imprimir Serial ////////////////////////////////////

  if ((millis() - contadorLectura) > serialTimer) {
    //    Protocolo BlueTooth
    //
    //    outputString = 't';
    //    outputString += millis();
    //    outputString += 'p';
    //    outputString += maxPressure;
    //    outputString += ';';
    //    Serial.print(outputString);

    Serial.print(setPressure * 1.1);
    Serial.print("\t");
    Serial.print(setPressure * 0.9);
    Serial.print("\t");
    Serial.print(maxPressure);
    maxPressure = 0.0;
    Serial.print("\t");
    Serial.println(setPressure);

    contadorLectura = millis();

  } // End Serial

  //  Serial.println(millis() - t1);
  //  t1 = millis();

  if (digitalRead(batteryPin)) {
    alarmaBateria = HIGH;
  }
  else {
    alarmaBateriaOld = LOW;
    alarmaBateria = LOW;
  }

  if (digitalRead(eStopPin)) {
    alarmaeStop = HIGH;
  }
  else {
    alarmaeStop = LOW;
    alarmaeStopOld = LOW;
  }

  if (((alarmaSensor || alarmaPresionAlta || alarmaPresionBaja || alarmaAmbu || alarmaSensor2) && startCycle) || alarmaBloqueo || alarmaBateria || alarmaeStop) {
    digitalWrite(alarmPin, HIGH);
    alarmas = HIGH;
  }
  else
    digitalWrite(alarmPin, LOW);

  if (alarmaeStop && !alarmaeStopOld) {
    alarmaeStopOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaSensor && !alarmaSensorOld) {
    alarmaSensorOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaPresionAlta && !alarmaPresionAltaOld) {
    alarmaPresionAltaOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaPresionBaja && !alarmaPresionBajaOld) {
    alarmaPresionBajaOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaAmbu && !alarmaAmbuOld) {
    alarmaAmbuOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaSensor2 && !alarmaSensor2Old) {
    alarmaSensor2Old = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaBloqueo && !alarmaBloqueoOld) {
    alarmaBloqueoOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaBateria && !alarmaBateriaOld) {
    alarmaBateriaOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmas && !oldAlarmas) {
    lcd.noCursor();
    lcd.clear();
    numAlarmas = 0;
    numCol = 0;
    contCursor = 0;
    oldAlarmas = HIGH;
  }

  // Refrescar LCD // Solo se hace en Stop o al fin del ciclo

  if (((FSM == 2) && ((refreshLCD && !alarmas) || newAlarm)) || (!startCycle && !alarmas)) {
    if ((millis() - contadorLCD) > lcdTimer) { // Refresh LCD
      contadorLCD = millis();

      if (newAlarm) {
        if (alarmaPresionAlta) {
          if (numAlarmas > 3) {
            numAlarmas = 0;
            numCol = 10;
          }
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F("          "));
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F(" P. ALTA"));
          numAlarmas++;
        }
        if (alarmaPresionBaja) {
          if (numAlarmas > 3) {
            numAlarmas = 0;
            numCol = 10;
          }
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F("          "));
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F(" P. BAJA"));
          numAlarmas++;
        }
        if (alarmaSensor || alarmaSensor2) {
          if (numAlarmas > 3) {
            numAlarmas = 0;
            numCol = 10;
          }
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F("          "));
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F("MECANISMO"));
          numAlarmas++;
        }

        if (alarmaAmbu) {
          if (numAlarmas > 3) {
            numAlarmas = 0;
            numCol = 10;
          }
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F("          "));
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F("CAMBIO AMBU"));
          numAlarmas++;
        }
        if (alarmaBloqueo) {
          if (numAlarmas > 3) {
            numCol = 10;
            numAlarmas = 0;
          }
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F("          "));
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F(" BLOQUEO"));
          numAlarmas++;
        }

        if (alarmaBateria) {
          if (numAlarmas > 3) {
            numAlarmas = 0;
            numCol = 10;
          }
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F("          "));
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F(" BATERIA"));
          numAlarmas++;
        }

        if (alarmaeStop) {
          if (numAlarmas > 3) {
            numAlarmas = 0;
            numCol = 10;
          }
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F("          "));
          lcd.setCursor(numCol, numAlarmas);
          lcd.print(F("  E STOP"));
          numAlarmas++;
        }
        newAlarm = LOW;
      }

      else {

        presControl = readEncoderValue(1);
        inhaleTime = readEncoderValue(2) / 10.0;
        exhaleTime = readEncoderValue(3) / 10.0;

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

  if (((millis() - contadorBoton2) > changeScreenTimer) && alarmas) {
    contadorBoton2 = millis();

    if (alarmaAmbu) {
      //      numCiclos = 0;
      //      updatenumCiclos = 0;
      //      EEPROM.put(80, numCiclos);
      alarmaAmbu = LOW;
      alarmaAmbuOld = LOW;
    }

    alarmas = LOW;
    oldAlarmas = LOW;
    contCursor = 0;
    cargarLCD ();
  }

  if (((millis() - contadorBoton2) > 6000)) {
    numCiclos = 0;
    updatenumCiclos = 0;
    EEPROM.put(80, numCiclos);
  }

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


  if (!startCycle || digitalRead(eStopPin)) {
    digitalWrite(enPin, HIGH); // disable motor
  }

  if (((!digitalRead(startButton) && !digitalRead(eStopPin)) || startCycle)) { // Start
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

    if ((pressureRead < peepPressure) && pressureRead > 0)
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

        if ((pressureRead > (setPressure)) && !hysterisis) {
          motor.setMaxSpeed(0);
          motor.stop();
          hysterisis = HIGH;
        }

        if (((millis() - contadorCiclo) >= int(inhaleTime * 1000)) || alarmaPresionAlta) { // Condition to change state
          refreshLCD = LOW;
          motor.setMaxSpeed(0);
          motor.stop();

          if ((motor.currentPosition() < 1000) && hysterisis)
            alarmaBloqueo = HIGH;
          else {
            alarmaBloqueo = LOW;
            alarmaBloqueoOld = LOW;
          }
          hysterisis = LOW;
          contadorCiclo = millis();
          //          motor.setMaxSpeed(exhaleSpeed);
          //          motor.moveTo(-1200);


          //          if (!digitalRead(sensorPin))
          //            alarmaSensor2 = HIGH;
          //          else{
          //            alarmaSensor2 = LOW;
          //            alarmaSensor2Old=LOW;
          //          }
          FSM = 22;
        }
        break;

      case 22:
        peepPressure = 99.0;
        motor.setMaxSpeed(0);
        motor.stop();
        motor.setMaxSpeed(exhaleSpeed);
        motor.move(-1200 - maxPosition);

        //        Serial.print(motor.currentPosition());
        //        Serial.print('\t');
        //        Serial.println(motor.targetPosition());
        FSM = 2;
        break;

      case 2: // Exhalation Cycle
        //        if (digitalRead(sensorPin))
        //          motor.moveTo(-1200);

        if (!digitalRead(sensorPin)) {
          motor.setMaxSpeed(0);
          motor.stop();
          motor.setCurrentPosition(0);
          if (alarmaSensor) {
            alarmaSensor = LOW;
            alarmaSensorOld = LOW;
          }

          if (!checkSensor) {  // Flanco subida sensor regreso
            // Si hay presion baja
            if ((maxPressure2 - peepPressure) < pressMinLimit)
              contadorAlarmaPresionBaja++;
            else
              contadorAlarmaPresionBaja = 0;

            if (contadorAlarmaPresionBaja > 1)
              alarmaPresionBaja = HIGH;
            else {
              alarmaPresionBaja = LOW;
              alarmaPresionBajaOld = LOW;
            }
          }

          checkSensor = HIGH;
          refreshLCD = HIGH;
        }

        if ((millis() - contadorCiclo) >= int(exhaleTime * 1000)) {
          motor.setMaxSpeed(0);
          motor.stop();
          FSM = 0;
          checkSensor = LOW;
          contadorCiclo = millis();
          numCiclos++;
          updatenumCiclos++;
          if ((pressureRead < pressMaxLimit) && (pressureRead < pressMaxMovil)) {
            alarmaPresionAlta = LOW;
            alarmaPresionAltaOld = LOW;
          }
          if (updatenumCiclos > 50) { // Solo actualizo la EEPROM cada 50 ciclos.
            EEPROM.put(80, numCiclos);
            updatenumCiclos = 0;
          }
          if (digitalRead(sensorPin)) {
            alarmaSensor = HIGH;
          }
          if ((digitalRead(startButton)) || digitalRead(eStopPin))
            startCycle = LOW;
          //          refreshLCD = HIGH;
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

    if (encoderValue[0] > 160)
      encoderValue[0] = 160;

    if (encoderValue[1] < 20.0)
      encoderValue[1] = 20.0;

    if (encoderValue[2] < 20.0)
      encoderValue[2] = 20.0;

  }

  lastEncoded = encoded; //store this value for next time
}

float readPressure() {
  adc0 = ads.readADC_SingleEnded(0);
  return ((71.38 * (adc0 - offsetPresion) / offsetPresion) * 1.107 + 3.29);
}

ISR(TIMER1_COMPA_vect) {
  motor.run();  // Motor keepalive (at least once per step).
}

void cargarLCD () {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("PIP "));
  lcd.setCursor(12, 0);
  lcd.print(F("P IN"));
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
