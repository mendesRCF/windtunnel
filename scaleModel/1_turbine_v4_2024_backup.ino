// HYDRO-K PROJECT
// TURBINES ARRANGEMET V2.0
// DATE:27/04/2015
// DATE V2.0: 28/03/2016
// DEVELOPER: Rafael Mendes
// output (RPM,SETPOINT,control_parameter[0-100])
#include "HX711.h"
#define DT A5
#define SCK A4
//const int LOADCELL_DOUT_PIN = 21;
//const int LOADCELL_SCK_PIN = 20;
//----------------------------------------------------------------
//                              Libraries
//----------------------------------------------------------------

HX711 scale;

//----------------------------------------------------------------
//                            Rpm sensor
//----------------------------------------------------------------
//T1
volatile long npulse = 0;
volatile float freq = 0;
int pin = 3;  // signal rpm pin 18/19/20
float rot;


//----------------------------------------------------------------

long previousMillis_time = 0;  // will store last time LED was updated
long interval_time = 200;      // tempo de impressao T

long previousMillis_pid = 0;  // will store last time LED was updated
long interval_pid = 100;      // tempo de impressao T

unsigned long currentMillis = 0.0;

//----------------------------------------------------------------
//                           Control-out (PWM)
//----------------------------------------------------------------

//T1
const int analogOutPin = 5;  // PWN SIGNAL FOR TRANSISTOR ---  5/6/7
int sensorValue = 0;         // value read from digital port
int outputValue = 0;         // value output to the PWM (analog out)
int lastsensorValue = 124;

int relay11 = 6;
int relay12 = 7;

//----------------------------------------------------------------
//                       PID variables
//----------------------------------------------------------------

float textbox;
float setPoint = 100;  //frontal



//T1
float erro = 0.0;
float deltaTempo = 0.0;
float ultimoMomento = 0.0;
float amostra = 0.0;
float AmostraAnterior = 0.0;
float pid = 0.0;
float lastpid = 124;

float control_pwm = 0;

float kP1 = -0.5809;     //-0.5809;
float kI1 = -0.0094527;  //-0.05;
float kD1 = -0.2835;     //-0.20;

float P = 0.0;
float I = 0.0;
float D = 0.0;


long Ntime;
long Ltime;
float torque = 0.0;


//----------------------------------------------------------------
void setup() {

  Serial.begin(9600);
  pinMode(relay11, OUTPUT);
  pinMode(relay12, OUTPUT);


  //----------------------------------------------------------------
  //                          LOAD CELL
  //----------------------------------------------------------------
  scale.begin(DT, SCK);
  scale.read_average(20);
  scale.set_scale(-86);  // 1313.4, 1263
  scale.tare();

  //----------------------------------------------------------------
  //                           RPM
  //----------------------------------------------------------------
  while (!Serial) {
    ;
  };

  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), sense_rpm, RISING);



  Ltime = millis();
}
void loop() {



  //----------------------------------------------------------------
  //                           Control
  //----------------------------------------------------------------

  if (Serial.available()) {
    textbox = Serial.parseInt();
    setPoint = textbox;  //setPoint
    //setPoint2 = setPoint;


    Ltime = millis();
  }
  // if(millis()>30000) setPoint = 1600;
  // setPoint2 = setPoint;

  //  Ltime = millis();


  //automatic control

  //      Ntime = millis();
  //      if (Ntime - Ltime >= 30000L) {
  //        setPoint = setPoint - 100;
  //        Ltime += 30000L;
  //      }


  //----------------------------------------------------------------
  //                            PID
  //----------------------------------------------------------------

  //T1
  currentMillis = millis();
  if (currentMillis - previousMillis_pid > interval_pid) {
    previousMillis_pid = currentMillis;
    if (freq < 3000) {

      amostra = freq;  //10ms
      erro = setPoint - amostra;
      digitalWrite(relay11, LOW);
      digitalWrite(relay12, LOW);




      float agora = millis();

      deltaTempo = (agora - ultimoMomento) / 1000.0f;  // Tempo em segundos
      ultimoMomento = agora;

      P = kP1 * erro;                                      // $ P = kP * erro $
      I += kI1 * (erro) * (deltaTempo);                    // $ I(t)=kI*\Int{e(t)}*\delta{t} $
      D = kD1 * (AmostraAnterior - amostra) / deltaTempo;  // $ D(t)= kD*\delta{amostra}/\delta{t} $

      pid = P + I + D;

      AmostraAnterior = amostra;

      pid = (pid >= 50 ? 50 : pid);
      pid = (pid <= -50 ? -50 : pid);

      sensorValue = (lastpid + pid);
      sensorValue = (sensorValue >= 255 ? 255 : sensorValue);
      sensorValue = (sensorValue <= 0 ? 0 : sensorValue);
      lastpid = sensorValue;

      //outputValue = map(sensorValue, 0, 255, 0, 255);
      // change the analog out value:
      analogWrite(analogOutPin, sensorValue);
      delay(1);
    }

    control_pwm = outputValue / 255.0;
  }
  //analogWrite(analogOutPin, outputValue);



  //----------------------------------------------------------------
  //                          OUTPUTS
  //----------------------------------------------------------------

  currentMillis = millis();
  if (currentMillis - previousMillis_time > interval_time) {
    previousMillis_time = currentMillis;
    //    torque = scale.get_units();
    // if(freq2<3000){
    torque = 1.11 * scale.get_units() - 2.9;
    //if ((torque >= -100) && (torque <= 100)) {

    //if ((freq<9000) && (erro<9000)){
    //---------T1-----------//*
    Serial.print("w \t");
    Serial.print(freq, 0);
    Serial.print("\t erro \t");
    Serial.print(erro);
    Serial.print("\t pwm \t");
    Serial.print(control_pwm, 2);
    Serial.print("%\t set\t");
    Serial.print(setPoint);
    Serial.print("%\t P\t");
    Serial.print(P);

    Serial.print("%\t I\t");
    Serial.print(I);

    Serial.print("%\t D\t");
    Serial.print(D);

    Serial.print("%\t pid\t");
    Serial.print(pid);

    Serial.print("%\t Sv\t");
    Serial.print(sensorValue);

    Serial.print("%\t Dt\t");
    Serial.print(deltaTempo, 5);

    Serial.print("\t T \t");
    Serial.println(torque, 2);
  }
}

//----------------------------------------------------------------
//                          RPM FUNCTION
//----------------------------------------------------------------
void sense_rpm() {
  volatile long n2pulse = micros();
  freq = 1000000.000 / (n2pulse - npulse) * 60 / 4;  // RPM
  npulse = n2pulse;
}
