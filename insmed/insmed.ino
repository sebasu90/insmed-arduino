/**
  TO DO:
*/

#include <EEPROM.h>
#include <Wire.h>
#include "src/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#include "src/Adafruit_ADS1X15/Adafruit_ADS1015.h"
#include "src/AccelStepper/AccelStepper.h"

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

byte unlockChar[8] = {
  B01110,
  B10001,
  B10001,
  B10001,
  B11111,
  B11011,
  B11011,
  B11111
};

byte lockChar[8] = {
  B01110,
  B10001,
  B10000,
  B10000,
  B11111,
  B11011,
  B11011,
  B11111
};

LiquidCrystal_I2C lcd(0x38, 20, 4);

Adafruit_ADS1115 ads(0x48);

// Pin definitions
#define encoderPinA 7 // Throttle
#define encoderPinB 11
#define buttonPin 10

#define startButton 4
#define batteryPin 8
#define sensorPin A1   // Inductive sensor to control motor range
#define rstAlarmPin A0 // Start switch

// Motor outputs
#define pulsePin A2 // Motor drive 3
#define dirPin A3 // Motor drive 2
#define enPin A4 // Motor drive 1
#define mosfetPin 5

#define buzzerPin 9
#define ledAlarm 13

bool goHome = LOW;

bool setAlarmas = LOW;
bool alarmaSensor = LOW;
bool alarmaSensor2 = LOW;
bool alarmaPresionAlta = LOW;
bool alarmaPresionBaja = LOW;
bool alarmaAmbu = LOW;
bool alarmaBloqueo = LOW;
bool alarmaBateria = LOW;
bool alarmaBateriaBaja = LOW;
bool buzzer = LOW;

bool alarmas = LOW;
bool oldAlarmas = LOW; // Rising edge to clear screen

byte numAlarmas = 0;
byte numCol = 0;

bool newAlarm = LOW;

bool psvMode = LOW;

bool alarmaSensorOld = LOW;
bool alarmaPresionAltaOld = LOW;
bool alarmaPresionBajaOld = LOW;
bool alarmaSensor2Old = LOW;
bool alarmaAmbuOld = LOW;
bool alarmaBloqueoOld = LOW;
bool alarmaBateriaOld = LOW;
bool alarmaBateriaBajaOld = LOW;

bool alarmaeStop = LOW;
bool alarmaeStopOld = LOW;

float pressMinLimit = 4.0;
float pressMaxLimit = 41.0;

float pressMinMovil = 0.0;
float pressMaxMovil = 0.0;

bool hysterisis = LOW;

byte contadorAlarmaPresionBaja = 0;

byte lcdIndex;

AccelStepper motor(AccelStepper::DRIVER, pulsePin, dirPin);

volatile int lastEncoded = 0;
volatile int encoderValue[10];
long lastencoderValue = 0;

int16_t adc0; // ADS1015 reading

// Manage the LDC cursor and screens

byte contCursor = 0;
byte contCursor2 = 0;

bool lockState = LOW;
bool lockStateOld = LOW;

long currentTime;

bool Start = LOW;
bool startButtonState = LOW;

bool botonAnterior;

bool Read = LOW;

bool estadoBoton = LOW;
long contadorBoton = 0;
long contadorLCD = 0;
long contadorBoton2 = 0;
long contadorBotonStart = 0;
long contadorCiclo = 0;
long contadorControl = 0;
long contadorLectura = 0;
long contadorLed = 0;
long contadorBuzzer = 0;
long contadorLecturapresion = 0;
long contadorHorometro = 0;
long contadorRstAlarmas = 0;

bool refreshLCD = LOW;
bool checkSensor = LOW;

long t1;
long t2;
long t3;
long dt1;
long dt2;
long dt3;

#define lcdTimer 600
#define serialTimer 100
#define buttonTimer 110
#define changeScreenTimer 2000
#define ledTimer 250
#define buzzerTimer 250
#define rstAlarmasTimer 110

byte index = 0;
byte FSM;

int maxPosition = 2500;
int inhaleSpeed = 2800;
int exhaleSpeed = 3200;

signed int minPosition = -4400;

bool startCycle = LOW;

long numCiclosOld;
int presControlOld;
int bpmOld;
float ieRatioOld;
float peepPressureLCDOld = 98;
float maxPressureLCDOld = 98;

#define maxnumCiclos 60000
long numCiclos; // Not so frequecnt EEPROM write
byte updatenumCiclos = 0;

//String bufferString;

float maxPressure;        // Serial
float maxPressure2;       // LCD
float peepPressure = 0.0; // LCD

float maxPressureLCD;
float peepPressureLCD;

//float compliance;
//float Volumen;

// Process Variables

int bpm = 0;
float ieRatio = 0.0;

float inhaleTime = 0.0;
float exhaleTime = 0.0;

int presControl;
float setPressure = 0.0;
float pressure = 0.0;
float pressureRead = 0;
long offsetPresion = 0;

int readEncoderValue(byte index)
{
  return ((encoderValue[index - 1] / 4));
}

String inputString, outputString;

boolean isButtonPushDown(void)
{
  if (!digitalRead(buttonPin))
  {
    if (!digitalRead(buttonPin))
      return true;
  }
  return false;
}

int readPresControlValue()
{
  return readEncoderValue(1);
};

float readIeRatioValue()
{
  return readEncoderValue(3) / 10.0;
};

int readBpmValue()
{
  return readEncoderValue(2);
};

/*
  Variables:

  presControl
  ieRatio
  bpm

  Alarmas:

  alarmaSensor
  alarmaPresionAlta
  alarmaPresionBaja
  alarmaAmbu
  alarmaBloqueo
  alarmaeStop
  alarmaBateria

  btSerial.alarm("alarmaSensor");
*/

#define HAND_SHAKE_CHAR 'h'
#define SEND_ALL_PARAMETERS_CHAR 's'

#define PRES_CONTROL_CHAR 'p'
#define BPM_CHAR 'b'
#define IE_RATIO_CHAR 'i'

#define debugSerial 0

class BTSerial
{
    bool handShaked = LOW;
    unsigned long beatTimer = 0;

    int _presControl = 0;
    bool _presControlAvailable = LOW;

    int _ieRatio = 0;
    bool _ieRatioAvailable = LOW;

    int _bpm = 0;
    bool _bpmAvailable = LOW;

    char readingChar = ' ';
    String inputString = "";

    void parseIntoVar()
    {
      switch (readingChar)
      {
        case PRES_CONTROL_CHAR:
          _presControlAvailable = HIGH;
          _presControl = inputString.toInt();
          _presControl = min(max(_presControl, 5), 40);
          readingChar = ' ';
          inputString = "";
          break;

        case IE_RATIO_CHAR:
          _ieRatioAvailable = HIGH;
          _ieRatio = inputString.toInt();
          _ieRatio = min(max(_ieRatio, 10), 99);
          readingChar = ' ';
          inputString = "";
          break;

        case BPM_CHAR:
          _bpmAvailable = HIGH;
          _bpm = inputString.toInt();
          _bpm = min(max(_bpm, 6), 40);
          readingChar = ' ';
          inputString = "";
          break;

        default:
          break;
      }
    }

  public:
    void setup()
    {
      Serial1.begin(9600);

      // Reset BT card
      Serial1.println("AT+RESTART");
    }

    void loop()
    {
      if (handShaked && (millis() - beatTimer) > 10 * 1000)
      {
        handShaked = LOW;
        Serial1.println("AT+RESTART");

        if (debugSerial) {
          Serial.println("");
          Serial.println("BLE restarted");
          Serial.println("");
        }
      }

      if (Serial1.available())
      {
        char inChar = Serial1.read();
        if (debugSerial)
          Serial.write(inChar);

        if (inChar == HAND_SHAKE_CHAR)
        {
          handShaked = HIGH;
          beatTimer = millis();

          if (debugSerial)
            Serial.println("handshaked");
        }

        if (inChar == SEND_ALL_PARAMETERS_CHAR)
        {
          String res = "s";
          res += PRES_CONTROL_CHAR;
          res += readPresControlValue();

          res += ";s";
          res += BPM_CHAR;
          res += readBpmValue();

          res += ";s";
          res += IE_RATIO_CHAR;
          res += ((int)(readIeRatioValue() * 10.0));

          res += ";";

          Serial1.println(res);
          if (debugSerial)
            Serial.println(res);
        }

        if (inChar == PRES_CONTROL_CHAR || inChar == BPM_CHAR || inChar == IE_RATIO_CHAR)
        {
          parseIntoVar();
          readingChar = inChar;
        }
        else if (readingChar != ' ' && isDigit(inChar))
        {
          inputString += inChar;
        }
        else if (inChar == ';')
        {
          parseIntoVar();
        }
      }
    }

    void print(String in)
    {
      if (handShaked)
      {
        Serial1.print(in);
        // Serial.print(in);
      }
    }

    bool presControlAvailable()
    {
      return _presControlAvailable;
    }
    int presControl()
    {
      _presControlAvailable = LOW;
      return _presControl;
    }

    bool bpmAvailable()
    {
      return _bpmAvailable;
    }
    int bpm()
    {
      _bpmAvailable = LOW;
      return _bpm;
    }

    bool ieRatioAvailable()
    {
      return _ieRatioAvailable;
    }
    int ieRatio()
    {
      _ieRatioAvailable = LOW;
      return _ieRatio;
    }
};

BTSerial btSerial;

/*******************************************************/

/*******************( SETUP )***************************/

void setup() //Las instrucciones solo se ejecutan una vez, despues del arranque
{
  Serial.begin(115200);
  btSerial.setup();

  pinSetup();

  lcd.createChar(2, unlockChar);
  lcd.createChar(1, lockChar);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);

  cli();      //stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  //initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 3200; // = Crystal of 16Mhz / 3200 cycles = 5 kHz Timer 1 frequency
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 for no prescaler
  TCCR1B |= (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei(); //allow interrupts

  lcd.init();
  lcd.clear();
  lcd.backlight();

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

  motor.setAcceleration(20000.0); // To test
  motor.setMinPulseWidth(25);

  offsetPresion = ads.readADC_SingleEnded(1);

  cargarLCD();
  contCursor2 = 2;
  t1 = millis();
  lockState = HIGH;

} //Fin del Setup

/*******************************************************/

/*******************( LOOP )****************************/

float getPIPValue() {
  return maxPressureLCD;
}

float getPEEPValue() {
  return peepPressureLCD;
}

long getNumCiclosValue() {
  return numCiclos;
}

int btParamSendIndex = 0;

void loop()
{

  // Imprimir Serial ////////////////////////////////////

  if ((millis() - contadorLectura) > serialTimer)
  {
    //    Protocolo BlueTooth
    //
    outputString = 't';
    outputString += millis();
    outputString += 'p';
    outputString += pressureRead;
    btParamSendIndex++;
    if (btParamSendIndex >= 10) {
      outputString += 'i';
      outputString += getPIPValue();
      outputString += 'e';
      outputString += getPEEPValue();
      outputString += 'n';
      outputString += getNumCiclosValue();
      btParamSendIndex = 0;
    }
    outputString += ';';


    btSerial.print(outputString);

    //    Serial.print(setPressure * 1.1);
    //    Serial.print("\t");
    //    Serial.print(setPressure * 0.9);
    //    Serial.print("\t");
    //    Serial.print(maxPressure);
    //    maxPressure = 0.0;
    //    Serial.print("\t");
    //    Serial.println(setPressure);

    contadorLectura = millis();

  } // End Serial


  btSerial.loop();

  if (btSerial.presControlAvailable()) {
    encoderValue[0] = btSerial.presControl() * 4;
  }

  if (btSerial.ieRatioAvailable()) {
    encoderValue[2] = btSerial.ieRatio() * 4;
  }

  if (btSerial.bpmAvailable()) {
    encoderValue[1] = btSerial.bpm() * 4;
  }

  ////// Alarmas //////////

  if (checkBattery() < 0.0)
    alarmaBateriaBaja = HIGH;
  else {
    alarmaBateriaBaja = LOW;
    alarmaBateriaBajaOld = LOW;
  }

  if (digitalRead(batteryPin))
  {
    alarmaBateria = HIGH;
  }
  else
  {
    alarmaBateria = LOW;
    alarmaBateriaOld = LOW;
  }

  if (((alarmaSensor || alarmaPresionAlta || alarmaPresionBaja || alarmaAmbu || alarmaSensor2 || alarmaBloqueo) && startCycle) || alarmaBateria)
  {
    alarmas = HIGH;
  }
  else
    alarmas = LOW;

  if (buzzer) {
    if ((millis() - contadorBuzzer) > buzzerTimer) {
      digitalWrite(buzzerPin, !digitalRead(buzzerPin));
      contadorBuzzer = millis();
    }
  }
  else
    digitalWrite(buzzerPin, LOW);

  if (alarmas) {
    if (millis() - contadorLed > ledTimer) {
      digitalWrite(ledAlarm, !digitalRead(ledAlarm));
      contadorLed = millis();
    }
  }
  else {
    digitalWrite(ledAlarm, LOW);
  }

  if (startButtonState)
    digitalWrite(mosfetPin, HIGH);
  else
    digitalWrite(mosfetPin, LOW);


  if (alarmaSensor && !alarmaSensorOld)
  {
    alarmaSensorOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaPresionAlta && !alarmaPresionAltaOld)
  {
    alarmaPresionAltaOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaPresionBaja && !alarmaPresionBajaOld)
  {
    alarmaPresionBajaOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaAmbu && !alarmaAmbuOld)
  {
    alarmaAmbuOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaSensor2 && !alarmaSensor2Old)
  {
    alarmaSensor2Old = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaBloqueo && !alarmaBloqueoOld)
  {
    alarmaBloqueoOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaBateria && !alarmaBateriaOld) {
    alarmaBateriaOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaBateriaBaja && !alarmaBateriaBajaOld) {
    alarmaBateriaBajaOld = HIGH;
    newAlarm = HIGH;
  }

  if (newAlarm)
  {
    setAlarmas = HIGH;
    buzzer = HIGH;
  }

  // Refrescar LCD // Solo se hace en Stop o al fin del ciclo

  if ((millis() - contadorLCD) > lcdTimer)
  { // Refresh LCD
    contadorLCD = millis();
    refreshLCD = HIGH;
  }

  if ((!setAlarmas || newAlarm) && refreshLCD)
  {
    if (newAlarm)
    {
      displayAlarmas();
      newAlarm = LOW;
    }

    else if (numAlarmas == 0)
    {
      refreshLCDvalues();

    } // If no Alarmas
  }   // If refreshLCD

  if (!isButtonPushDown())
  { // Anti-Bounce Button Switch
    contadorBoton = millis();
    contadorBoton2 = millis();
  }

  if (!digitalRead(rstAlarmPin))
    contadorRstAlarmas = millis();

  if ((millis() - contadorRstAlarmas) > rstAlarmasTimer) {
    if (buzzer) {
      contadorRstAlarmas = millis();
      buzzer = LOW;
    }
  }

  Serial.print(lockState);
  Serial.print('\t');
  Serial.println(millis() - contadorBoton2);

  if ((millis() - contadorBoton2) > changeScreenTimer) {
    contadorBoton2 = millis();
    lockState = !lockState;
  }

  if (((millis() - contadorRstAlarmas) > changeScreenTimer) && setAlarmas)
  {
    contadorRstAlarmas = millis();

    resetAlarmas();

    presControlOld = 0.1;
    peepPressureLCDOld = 0.1;
    maxPressureLCDOld = 0.1;
    bpmOld = 98;
    numCiclosOld = 0;
    ieRatioOld = 98;
    lockState = HIGH;

    cargarLCD();
  }

  if (((millis() - contadorRstAlarmas) > 8000))
  {
    numCiclos = 0;
    updatenumCiclos = 0;
    EEPROM.put(80, numCiclos);
  }

  if (((millis() - contadorBoton) > buttonTimer))
  {
    contadorBoton = millis();
    switchCursor();

    // End Else no Buzzer
  }   // End If Button Switch

  if (digitalRead(startButton)) {
    contadorBotonStart = millis();
  }

  if ((millis() - contadorBotonStart) > 1000) {
    contadorBotonStart = millis();
    startButtonState = !startButtonState;
  }

  if (!startCycle)
  {
    digitalWrite(enPin, HIGH); // disable motor
  }


  if (startButtonState || startCycle)
  { // Start
    startCycle = HIGH;
    digitalWrite(enPin, LOW); // Enable motor

    updatePressure();

    if (numCiclos > maxnumCiclos)
      alarmaAmbu = HIGH;

    if ((pressureRead > pressMaxLimit) || (pressureRead > pressMaxMovil))
      alarmaPresionAlta = HIGH;

    switch (FSM)
    {
      case 0:
        if ((psvMode && (peepPressure - pressureRead > 5.0)) || (!psvMode)) {
          contadorCiclo = millis();
          FSM = 1;
          motor.setMaxSpeed(inhaleSpeed);
          motor.moveTo(maxPosition);
          maxPressure2 = 0.0;
          hysterisis = LOW;
        }
        break;

      case 1: // Inhalation Cycle

        if ((pressureRead > (setPressure)) && !hysterisis)
        {
          motor.stop();
          hysterisis = HIGH;
        }

        if (((millis() - contadorCiclo) >= int(inhaleTime * 1000)) || alarmaPresionAlta)
        { // Condition to change state
          motor.stop();
          delay(2);
          if ((motor.currentPosition() < 2000) && hysterisis)
            alarmaBloqueo = HIGH;
          else
          {
            alarmaBloqueo = LOW;
            alarmaBloqueoOld = LOW;
          }
          hysterisis = LOW;
          motor.setMaxSpeed(exhaleSpeed);
          contadorCiclo = millis();
          //          delay(1);
          FSM = 22;
        }
        break;

      case 22:
        maxPressureLCD = maxPressure2;
        peepPressure = 99.0;
        motor.setMaxSpeed(exhaleSpeed);
        motor.move(minPosition);
        contadorCiclo = millis();
        FSM = 2;
        break;

      case 2: // Exhalation Cycle
        //        if (digitalRead(sensorPin))
        //          motor.moveTo(-1200);

        if (!digitalRead(sensorPin))
        {
          //            motor.setMaxSpeed(0);
          motor.stop();
          motor.setCurrentPosition(0);
          if (alarmaSensor)
          {
            alarmaSensor = LOW;
            alarmaSensorOld = LOW;
          }

          if (!checkSensor)
          { // Flanco subida sensor regreso
            // Si hay presion baja
            if ((maxPressure2 - peepPressure) < pressMinLimit)
              contadorAlarmaPresionBaja++;
            else
              contadorAlarmaPresionBaja = 0;

            if (contadorAlarmaPresionBaja > 1)
              alarmaPresionBaja = HIGH;
            else
            {
              alarmaPresionBaja = LOW;
              alarmaPresionBajaOld = LOW;
            }
          }
          checkSensor = HIGH;
        }

        if ((millis() - contadorCiclo) >= int(exhaleTime * 1000))
        {
          motor.setMaxSpeed(0);
          motor.stop();
          FSM = 0;
          checkSensor = LOW;
          contadorCiclo = millis();
          numCiclos++;
          updatenumCiclos++;
          peepPressureLCD = peepPressure;
          if ((pressureRead < pressMaxLimit) && (pressureRead < pressMaxMovil))
          {
            alarmaPresionAlta = LOW;
            alarmaPresionAltaOld = LOW;
          }
          if (updatenumCiclos > 50)
          { // Solo actualizo la EEPROM cada 50 ciclos.
            EEPROM.put(80, numCiclos);
            updatenumCiclos = 0;
          }
          if (digitalRead(sensorPin))
          {
            alarmaSensor = HIGH;
          }
          if (!startButtonState)
            startCycle = LOW;
        }
        break;

      default:
        break;
    } // End cases
  }   // End machine cycle
} //End Loop


///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

void updateEncoder()
{
  int MSB = digitalRead(encoderPinA); //MSB = most significant bit
  int LSB = digitalRead(encoderPinB); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB;         //converting the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if (contCursor > 0)
  {

    if (sum == 5 || sum == 6 ||  sum == 9)
      encoderValue[contCursor - 1]++;
    if (sum == 3 || sum == 12 || sum == 8)

      encoderValue[contCursor - 1]--;

    if (encoderValue[0] > 160)
      encoderValue[0] = 160;

    if (encoderValue[0] < 20)
      encoderValue[0] = 20;

    if (encoderValue[2] < 40)
      encoderValue[2] = 40;

    if (encoderValue[2] > 200)
      encoderValue[2] = 200;

    if (encoderValue[1] < 20)
      encoderValue[1] = 20;

    if (encoderValue[1] > 160)
      encoderValue[1] = 160;
  }

  lastEncoded = encoded; //store this value for next time
}

float readPressure()
{
  adc0 = ads.readADC_SingleEnded(0);
  //    return ((71.38 * (adc0 - offsetPresion) / offsetPresion));
  return ((71.38 * (adc0 - offsetPresion) / offsetPresion) * 1.197 + 0.38);
}

ISR(TIMER1_COMPA_vect)
{
  motor.run(); // Motor keepalive (at least once per step).
}

void cargarLCD()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("PIP"));
  lcd.setCursor(11, 0);
  lcd.print(F("PCONT"));
  lcd.setCursor(0, 1);
  lcd.print(F("PEEP"));
  lcd.setCursor(0, 2);
  lcd.print(F("tIN"));
  lcd.setCursor(0, 3);
  lcd.print(F("tEX"));
  lcd.setCursor(11, 2);
  lcd.print(F("I:E 1:"));
  lcd.setCursor(11, 1);
  lcd.print(F("FR"));
  lcd.setCursor(11, 3);
  lcd.print(F("#"));
}

void refreshLCDvalues()
{
  switch (lcdIndex)
  {
    case 0:
      if (maxPressureLCDOld == maxPressureLCD)
        lcdIndex = 2;
      else
      {
        maxPressureLCDOld = maxPressureLCD;
        t1 = millis();
        lcd.setCursor(5, 0);
        lcd.print(F("    "));
        lcdIndex++;
      }
      break;

    case 1:
      lcd.setCursor(5, 0);
      lcd.print(getPIPValue(), 1); // PIP
      lcdIndex++;
      break;

    case 2:
      if (peepPressureLCDOld == peepPressureLCD)
        lcdIndex = 4;
      else
      {
        peepPressureLCDOld = peepPressureLCD;
        lcd.setCursor(5, 1);
        lcd.print("    ");
        lcdIndex++;
      }
      break;

    case 3:
      lcd.setCursor(5, 1); // PEEP
      lcd.print(getPEEPValue(), 1);
      lcdIndex++;
      break;

    case 4:
      ieRatio = readIeRatioValue();
      bpm = readBpmValue();
      if (ieRatioOld == ieRatio && bpmOld == bpm)
        lcdIndex = 9;
      else
      {
        inhaleTime = 60.0 / (bpm * (1 + ieRatio));
        lcd.setCursor(5, 2);
        lcd.print(inhaleTime, 1);
        lcdIndex++;
      }
      break;

    case 5:
      exhaleTime = inhaleTime * ieRatio;
      lcd.setCursor(5, 3);
      lcd.print(exhaleTime, 1);
      lcdIndex++;
      break;

    case 6:
      if (ieRatioOld == ieRatio)
        lcdIndex = 7;
      else
      {
        lcd.setCursor(17, 2);
        lcd.print(ieRatio, 1);
        lcdIndex++;
        ieRatioOld = ieRatio;
      }
      break;

    case 7:
      if (bpmOld == bpm)
        lcdIndex = 9;
      else
      {
        lcd.setCursor(17, 1);
        lcd.print(F("  "));
        bpmOld = bpm;
        lcdIndex++;
      }
      break;

    case 8:
      lcd.setCursor(17, 1);
      lcd.print(bpm); // BPM
      lcdIndex++;
      break;

    case 9:
      presControl = readPresControlValue();
      if (presControlOld == presControl)
        lcdIndex = 11;
      else
      {
        lcd.setCursor(17, 0);
        lcd.print("  ");
        lcdIndex++;
      }
      break;

    case 10:
      lcd.setCursor(17, 0);
      lcd.print(presControl);
      presControlOld = presControl;
      lcdIndex++;
      break;

    case 11:
      if (numCiclosOld == numCiclos)
        lcdIndex = 13;
      else
      {
        lcd.setCursor(14, 3);
        lcd.print("      ");
        lcdIndex++;
      }
      break;

    case 12:
      lcd.setCursor(12, 3);
      lcd.print(getNumCiclosValue());
      numCiclosOld = getNumCiclosValue();
      lcdIndex++;
      break;

    case 13:
      if (lockState == lockStateOld) {
        lcdIndex = 15;
      }
      else {
        lcd.setCursor(18, 3);
        lcdIndex++;
      }
      break;

    case 14:
      if (lockState) {
        lcd.printByte(byte(1));
      }
      else {
        lcd.printByte(byte(2));
      }
      lockStateOld = lockState;
      //      lcd.setCursor(17, 0);
      lcdIndex++;
      break;


    case 15:
      if (contCursor == 1)
      {
        lcd.setCursor(17, 0);
      }

      if (contCursor == 2)
      {
        lcd.setCursor(17, 1);
      }

      if (contCursor == 3)
      {
        lcd.setCursor(17, 2);
      }
      lcdIndex++;
      break;

    case 16:
      refreshLCD = LOW;
      lcdIndex = 0;
      break;

    default:
      break;
  }
}

void switchCursor () {
  if (contCursor == 0)
  {
    lcd.blink();
    contCursor = 1;
  }

  else if (contCursor == 1)
  {
    EEPROM.put(contCursor * 10, encoderValue[contCursor - 1]);
    contCursor = 2;
  }

  else if (contCursor == 2)
  {
    EEPROM.put(contCursor * 10, encoderValue[contCursor - 1]);
    contCursor = 3;
  }

  else if (contCursor == 3)
  {
    EEPROM.put(contCursor * 10, encoderValue[contCursor - 1]);
    lcd.noBlink();
    contCursor = 0;
  }

  else if (contCursor > 3)
  {
    contCursor = 0;
    lcd.noBlink();
  }
}  //

void resetAlarmas() {
  if (alarmaAmbu)
  {
    alarmaAmbu = LOW;
    alarmaAmbuOld = LOW;
  }

  setAlarmas = LOW;
  oldAlarmas = LOW;
  numAlarmas = 0;
  numCol = 0;
  contCursor = 0;
}

void updatePressure() {
  pressureRead = readPressure(); // Once per cycle

  if (pressureRead < -70.0)
    pressureRead = -70.0;

  // Save the greatest value to monitor (Peaks)

  if (pressureRead > maxPressure2)
    maxPressure2 = pressureRead;

  if (pressureRead > maxPressure)
    maxPressure = pressureRead;

  if ((pressureRead < peepPressure) && pressureRead > -70.0 && ((millis() - contadorCiclo) < 1000) && FSM == 2)
    peepPressure = pressureRead;

  if (FSM != 2)
  {
    setPressure = presControl + peepPressure;
    pressMinMovil = setPressure * 0.8;
    pressMaxMovil = setPressure * 1.2;
  }

}

void displayAlarmas() {
  if (numAlarmas == 0)
  {
    lcd.noCursor();
    lcd.clear();
  }
  if (alarmaPresionAlta)
  {
    if (numAlarmas > 3)
    {
      numAlarmas = 0;
      numCol = 10;
    }
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("          "));
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F(" P. ALTA"));
    numAlarmas++;
  }
  if (alarmaPresionBaja)
  {
    if (numAlarmas > 3)
    {
      numAlarmas = 0;
      numCol = 10;
    }
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("          "));
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F(" P. BAJA"));
    numAlarmas++;
  }
  if (alarmaSensor || alarmaSensor2)
  {
    if (numAlarmas > 3)
    {
      numAlarmas = 0;
      numCol = 10;
    }
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("          "));
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("MECANISMO"));
    numAlarmas++;
  }

  if (alarmaAmbu)
  {
    if (numAlarmas > 3)
    {
      numAlarmas = 0;
      numCol = 10;
    }
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("          "));
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("CAMBIO AMBU"));
    numAlarmas++;
  }
  if (alarmaBloqueo)
  {
    if (numAlarmas > 3)
    {
      numCol = 10;
      numAlarmas = 0;
    }
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("          "));
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F(" BLOQUEO"));
    numAlarmas++;
  }

  if (alarmaeStop)
  {
    if (numAlarmas > 3)
    {
      numAlarmas = 0;
      numCol = 10;
    }
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("          "));
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("  E STOP"));
    numAlarmas++;
  }

  if (alarmaBateria)
  {
    if (numAlarmas > 3)
    {
      numAlarmas = 0;
      numCol = 10;
    }
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("          "));
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("DESC ELECT"));
    numAlarmas++;
  }

  if (alarmaBateriaBaja)
  {
    if (numAlarmas > 3)
    {
      numAlarmas = 0;
      numCol = 10;
    }
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("          "));
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("LOW BATT"));
    numAlarmas++;
  }
}

void pinSetup () {

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(startButton, INPUT);
  pinMode(sensorPin, INPUT);
  pinMode(rstAlarmPin, INPUT);

  pinMode(enPin, OUTPUT);
  pinMode(pulsePin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  pinMode(buzzerPin, OUTPUT);
  pinMode(ledAlarm, OUTPUT);

  pinMode(mosfetPin, OUTPUT);

  digitalWrite(startButton, HIGH); //turn pullup resistor on
  digitalWrite(encoderPinA, HIGH); //turn pullup resistor on
  digitalWrite(encoderPinB, HIGH); //turn pullup resistor on

}

float checkBattery () {
  return (ads.readADC_SingleEnded(2) / 906.14);

}

/*******************************************************/
