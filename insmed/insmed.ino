#include <EEPROM.h>
#include <Wire.h>
#include "src/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#include "src/Adafruit_ADS1X15/Adafruit_ADS1015.h"

bool motorRun = LOW;
volatile int motorPulses = 0;
int nBase;
volatile int frecIndex = 0;
bool pulsePinState;
bool dirState;
int nSubida = 4;
int nBajada = 3;

int peepIndex = 0;

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
bool alarmaPresionBaja2 = LOW;
bool alarmaAmbu = LOW;
bool alarmaBloqueo = LOW;
bool alarmaBateria = LOW;
bool alarmaBateriaBaja = LOW;
bool alarmaPeep = LOW;

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
bool alarmaPresionBaja2Old = LOW;
bool alarmaSensor2Old = LOW;
bool alarmaAmbuOld = LOW;
bool alarmaBloqueoOld = LOW;
bool alarmaBateriaOld = LOW;
bool alarmaBateriaBajaOld = LOW;
bool alarmaPeepOld = LOW;

bool alarmaeStop = LOW;
bool alarmaeStopOld = LOW;

float pressMinLimit = 4.0;
float pressMaxLimit = 41.0;

float pressMinMovil = 0.0;
float pressMaxMovil = 0.0;

bool hysterisis = LOW;

byte contadorAlarmaPresionBaja = 0;

byte lcdIndex;

volatile int lastEncoded = 0;
volatile int encoderValue[10];
long lastencoderValue = 0;

int16_t adc0; // ADS1015 reading
int16_t adc2; // ADS1015 reading

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
long contadorUnlock = 0;

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
#define buttonTimer 130
#define changeScreenTimer 2000
#define ledTimer 250
#define buzzerTimer 250
#define rstAlarmasTimer 110
#define maxnumCiclos 60000
#define unlockTimer 600000

byte index = 0;
byte FSM;

int maxPosition = 2500;
int inhaleSpeed = 2300;
int exhaleSpeed = 3500;

int mindelay = 8;

signed int minPosition = -2500;

bool startCycle = LOW;

long numCiclosOld;
int presControlOld;
int bpmOld;
float ieRatioOld;
bool psvModeOld;
float pTriggerOld;
float peepPressureLCDOld = 98;
float maxPressureLCDOld = 98;

float minBattVoltage = 22.5;

long numCiclos; // Not so frequecnt EEPROM write
byte updatenumCiclos = 0;

//String bufferString;

float maxPressure;        // Serial
float maxPressure2;       // LCD
float peepPressure = 0.0; // LCD

float maxPressureLCD;
float peepPressureLCD;
float setPeepPressure;
float pTrigger;

//float compliance;
float volumen;
float flujo;

// Process Variables

int bpm = 0;
float ieRatio = 0.0;

float inhaleTime = 0.0;
float exhaleTime = 0.0;

int presControl;
float setPressure = 0.0;
float pressure = 0.0;
float pressureRead = 0;
int16_t offsetPresion = 0;
int16_t offsetFlujo = 0;

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
      //      if (handShaked)
      //      {
      Serial1.print(in);
      //      }
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

//int spiIndex;
//
//ISR (SPI_STC_vect)
//{
//  byte Slavereceived = SPDR;
//  Serial.print("Send: ");
//  Serial.println(spiIndex);
//  spiIndex++;
//  if (Slavereceived == 1)
//    SPDR = spiIndex;
//}

/*******************( SETUP )***************************/

void setup() //Las instrucciones solo se ejecutan una vez, despues del arranque
{
  Serial.begin(115200);
  btSerial.setup();

  pinSetup();

  lcd.init();
  lcd.clear();
  lcd.backlight();

  lcd.createChar(2, unlockChar);
  lcd.createChar(1, lockChar);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);

  cli();      //stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  //initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 800; // = Crystal of 16Mhz / 800 cycles = 20 kHz Timer 1 frequency
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 for no prescaler
  TCCR1B |= (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei(); //allow interrupts

  contadorLCD = millis();
  contadorLectura = millis();

  ads.begin();

  // Read from EEPROM the machine's parameters

  EEPROM.get(10, encoderValue[0]); // P Control
  EEPROM.get(20, encoderValue[1]); // FR
  EEPROM.get(30, encoderValue[2]); // I:E
  EEPROM.get(40, encoderValue[3]); // P trigger PSV Mode
  EEPROM.get(50, encoderValue[4]);
  EEPROM.get(60, encoderValue[5]);
  EEPROM.get(70, encoderValue[6]);
  EEPROM.get(80, numCiclos); // num ciclos

  offsetPresion = ads.readADC_SingleEnded(1);
  offsetFlujo = ads.readADC_SingleEnded(1);

  cargarLCD();
  contCursor2 = 2;
  //  t1 = millis();
  lockState = HIGH;

  //  pinMode(MISO, OUTPUT); // have to send on master in so it set as output
  //  SPCR |= _BV(SPE); // turn on SPI in slave mode

  //  SPI.begin();
  //  SPI.setClockDivider(SPI_CLOCK_DIV2);//divide the clock by 4

  //  SPI.attachInterrupt(); // turn on interrupt

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
      outputString += 'b';
      outputString += readBpmValue();
      outputString += 'r';
      outputString += readIeRatioValue();
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
    //    Serial.println(outputString);

    //    SPI.transfer('a');

    //    Serial.print(peepPressure);
    //    Serial.print("\t");
    //    Serial.print(peepIndex);
    //    Serial.print("\t");
    //    Serial.print(maxPressure);
    //    maxPressure = 0.0;
    //    Serial.print("\t");
    //    Serial.println(setPressure);

    //    Serial.print(motorPulses);
    //    Serial.print("\t");
    //    Serial.print(ads.readADC_SingleEnded(0));
    //    Serial.print("\t");
    Serial.print(flujo);
    Serial.print("\t");
    Serial.println(pressureRead);


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

  if (checkBattery() < minBattVoltage)
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

  if (((alarmaSensor || alarmaPresionAlta || alarmaPeep || alarmaPresionBaja || alarmaPresionBaja2 || alarmaAmbu || alarmaSensor2 || alarmaBloqueo) && startCycle) || alarmaBateria || alarmaBateriaBaja)
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

  if (alarmaPresionBaja2 && !alarmaPresionBaja2Old)
  {
    alarmaPresionBaja2Old = HIGH;
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

  if (alarmaPeep && !alarmaPeepOld) {
    alarmaPeepOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaBateriaBaja && !alarmaBateriaBajaOld) {
    alarmaBateriaBajaOld = HIGH;
    newAlarm = HIGH;
  }

  if (alarmaPeep && !alarmaPeepOld) {
    alarmaPeepOld = HIGH;
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

  if ((!setAlarmas || newAlarm) && refreshLCD) {
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

  if (!isButtonPushDown()) { // Anti-Bounce Button Switch
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

  if ((millis() - contadorBoton2) > changeScreenTimer) {
    contadorBoton2 = millis();
    if (lockState)
      lockState = LOW;
    else {
      contCursor = 0;
      lcd.noBlink();
      lockState = HIGH;
    }
  }

  if (lockState)
    contadorUnlock = millis();

  if ((millis() - contadorUnlock) > unlockTimer) {
    contCursor = 0;
    lcd.noBlink();
    lockState = HIGH;
  }

  if (((millis() - contadorRstAlarmas) > changeScreenTimer))
  {
    if (setAlarmas) {
      contadorRstAlarmas = millis();

      resetAlarmas();


      presControlOld = 0.1;
      peepPressureLCDOld = 0.1;
      maxPressureLCDOld = 0.1;
      bpmOld = 98;
      numCiclosOld = 0;
      ieRatioOld = 98;
      lockStateOld = !lockState;
      psvModeOld = !psvMode;
      pTriggerOld = 98;
      cargarLCD();
    }
    else if (alarmas)
      newAlarm = HIGH;
  }

  if (((millis() - contadorRstAlarmas) > 8000))
  {
    numCiclos = 0;
    numCiclosOld = 99;
    updatenumCiclos = 0;
    EEPROM.put(80, numCiclos);
  }

  if (((millis() - contadorBoton) > buttonTimer))
  {
    contadorBoton = millis();
    if (!lockState)
      switchCursor();

    // End Else no Buzzer
  }   // End If Button Switch

  // Read Control Parameters //

  presControl = readPresControlValue();

  pTrigger = readEncoderValue(5) / 10.0;

  psvMode = readEncoderValue(4) % 2;

  ieRatio = readIeRatioValue();
  bpm = readBpmValue();
  inhaleTime = 60.0 / (bpm * (1 + ieRatio));
  exhaleTime = inhaleTime * ieRatio;

  startButtonState = !digitalRead(startButton);

  // Start Cycle
  updatePressure();
  flujo = readFlow();
  dtFlujo = millis() - tFlujo;
  volumen += 

  if (startButtonState && !startCycle) {
    startCycle = HIGH;
  }

  if (startButtonState || startCycle)
  { // Start
    updatePressure();

    if (((numCiclos % maxnumCiclos) == 0) && (numCiclos != 0))
      alarmaAmbu = HIGH;

    if ((pressureRead > pressMaxLimit) || (pressureRead > pressMaxMovil))
      alarmaPresionAlta = HIGH;

    switch (FSM)
    {
      case 0:
        nBase = nSubida;
        digitalWrite(dirPin, HIGH);
        dirState = HIGH;
        contadorCiclo = millis();
        FSM = 1;
        motorRun = HIGH;
        maxPressure2 = 0.0;
        hysterisis = LOW;
        break;

      case 1: // Inhalation Cycle

        if ((pressureRead > (setPressure)) && !hysterisis)
        {
          motorRun = LOW;
          hysterisis = HIGH;
        }

        if (((millis() - contadorCiclo) >= int(inhaleTime * 1000) + 150) || alarmaPresionAlta)
        { // Condition to change state
          motorRun = LOW;
          if ((motorPulses < (785 + 36 * presControl)) && hysterisis)
            alarmaBloqueo = HIGH;
          else
          {
            alarmaBloqueo = LOW;
            alarmaBloqueoOld = LOW;
          }
          if (!digitalRead(sensorPin))
            alarmaSensor2 = HIGH;
          else {
            alarmaSensor2 = LOW;
            alarmaSensor2Old = LOW;
          }

          if (maxPressure2 < (setPressure * 0.8)) {
            alarmaPresionBaja2 = HIGH;
          }
          else {
            alarmaPresionBaja2Old = LOW;
            alarmaPresionBaja2 = LOW;
          }

          hysterisis = LOW;
          nBase = nBajada;
          digitalWrite(dirPin, LOW);
          dirState = LOW;
          contadorCiclo = millis();
          FSM = 22;
        }
        break;

      case 22:
        maxPressureLCD = maxPressure2;
        peepPressure = 99.0;
        motorRun = HIGH;
        contadorCiclo = millis();
        FSM = 2;
        break;

      case 2: // Exhalation Cycle

        if (!digitalRead(sensorPin))
        {
          motorRun = LOW;
          motorPulses = 0;
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

        if ((((millis() - contadorCiclo) >= int(exhaleTime * 1000 - 150)) && (!psvMode)) || ((psvMode && (peepPressure - pressureRead > pTrigger))))
        {
          motorRun = LOW;
          FSM = 0;
          checkSensor = LOW;
          contadorCiclo = millis();
          numCiclos++;
          updatenumCiclos++;
          peepPressureLCD = peepPressure;
          if (!lockState) {
            setPeepPressure = peepPressure;
            alarmaPeep = LOW;
            alarmaPeepOld = LOW;
          }
          else {
            if (abs(setPeepPressure - peepPressure) > 2.0)
              alarmaPeep = HIGH;
            else {
              alarmaPeep = LOW;
              alarmaPeepOld = LOW;
            }
          }

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

    if (encoderValue[1] < 20)
      encoderValue[1] = 20;

    if (encoderValue[1] > 160)
      encoderValue[1] = 160;

    if (encoderValue[2] < 40)
      encoderValue[2] = 40;

    if (encoderValue[2] > 200)
      encoderValue[2] = 200;

    if (encoderValue[3] < 0)
      encoderValue[3] = 0;

    if (encoderValue[3] > 4)
      encoderValue[3] = 4;

    if (encoderValue[4] < 80)
      encoderValue[4] = 80;

    if (encoderValue[4] > 200)
      encoderValue[4] = 200;
  }

  lastEncoded = encoded; //store this value for next time
}

float readPressure()
{
  adc0 = ads.readADC_SingleEnded(0);
  return ((71.38 * (adc0 - offsetPresion) / offsetPresion) * 1.1128); // No scorrection
}

ISR(TIMER1_COMPA_vect) {
  if (motorRun && startCycle) {
    if (((motorPulses < maxPosition) && dirState) || ((motorPulses > minPosition) && !dirState)) {
      frecIndex++;
      if (frecIndex >= nBase) { // Frec Base
        frecIndex = 0;
        pulsePinState = !pulsePinState;
        digitalWrite(pulsePin, pulsePinState);
        //        dt2 = micros() - t2;
        //        t2 = micros();
        if (pulsePinState) {
          if (dirState)
            motorPulses++;
          else
            motorPulses--;
        }
      } // End Frec Base
    }
  } // If no motor run
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
  lcd.print(F("MODO"));

  if (psvMode) {
    lcd.setCursor(0, 3);
    lcd.print(F("SENS"));
  }

  lcd.setCursor(11, 2);
  lcd.print(F("I:E 1:"));
  lcd.setCursor(11, 1);
  lcd.print(F("FR"));
  lcd.setCursor(11, 3);
  lcd.print(F("#"));
  lcd.noBlink();
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
      if (psvModeOld == psvMode) {
        if (psvMode)
          lcdIndex = 5;
        else
          lcdIndex = 6;
      }
      else {
        psvModeOld = psvMode;
        lcdIndex++;
        lcd.setCursor(5, 2);
        if (psvMode) {
          lcd.print("ACV");
          pTriggerOld = 98;
          lcdIndex = 45;
        }
        else {
          lcd.print("PCV");
          lcdIndex = 44;
        }
      }
      break;

    case 44:
      lcd.setCursor(0, 3);
      lcd.print("        ");
      lcdIndex = 6;
      break;

    case 45:
      lcd.setCursor(0, 3);
      lcd.print("SENS");
      lcdIndex = 5;
      break;

    case 5:
      if (pTriggerOld == pTrigger)
        lcdIndex++;
      else {
        pTriggerOld = pTrigger;
        lcd.setCursor(5, 3);
        lcd.print(pTrigger, 1);
        lcdIndex++;
      }
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
        lcd.setCursor(12, 3);
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
        lcd.setCursor(19, 3);
        lcdIndex++;
      }
      break;

    case 14:
      if (lockState) {
        lcd.write(byte(2));
      }
      else {
        lcd.write(byte(1));
      }
      lockStateOld = lockState;
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

      if (contCursor == 4)
      {
        lcd.setCursor(5, 2);
      }

      if (contCursor == 5)
      {
        lcd.setCursor(5, 3);
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
    lcd.blink();
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
    contCursor = 4;
  }

  else if (contCursor == 4)
  {
    EEPROM.put(contCursor * 10, encoderValue[contCursor - 1]);
    if (psvMode)
      contCursor = 5;
    else
      contCursor = 1;
  }

  else if (contCursor == 5)
  {
    EEPROM.put(contCursor * 10, encoderValue[contCursor - 1]);
    contCursor = 1;
  }

  else if (contCursor > 5)
  {
    contCursor = 1;
    //    lcd.noBlink();
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

float readFlow() {
  adc2 = ads.readADC_SingleEnded(1);
  return (48.64 * (71.38 * (adc2 - offsetFlujo) / offsetFlujo) * 1.1128); // No scorrection
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

  if ((pressureRead < peepPressure) && pressureRead > -70.0 && ((millis() - contadorCiclo) < 600) && FSM == 2) {
    peepPressure = pressureRead;
    peepIndex = 0;
  }

  //  if (pressureRead > -70.0 && ((millis() - contadorCiclo) > 200) && FSM == 2) {
  //    peepIndex++;
  //    peepPressure = (pressureRead + peepPressure * peepIndex) / (peepIndex + 1);
  //  }

  if (peepPressure < 0.0)
    peepPressure = 0.0;

  setPressure = presControl + peepPressureLCD;
  pressMinMovil = setPressure * 0.8;
  pressMaxMovil = setPressure * 1.2;
  //  }

}

void displayAlarmas() {
  lcd.noBlink();
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
    lcd.print(F(" P ALTA"));
    numAlarmas++;
  }
  if (alarmaPresionBaja2)
  {
    if (numAlarmas > 3)
    {
      numAlarmas = 0;
      numCol = 10;
    }
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("          "));
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F(" P BAJA"));
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
    lcd.print(F("DESC PAC."));
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
    lcd.print(F("FALLA GRAL"));
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
    lcd.print(F(" REV AMBU"));
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

  if (alarmaPeep)
  {
    if (numAlarmas > 3)
    {
      numAlarmas = 0;
      numCol = 10;
    }
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("          "));
    lcd.setCursor(numCol, numAlarmas);
    lcd.print(F("   PEEP"));
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
    lcd.print(F("BAT BAJA"));
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
