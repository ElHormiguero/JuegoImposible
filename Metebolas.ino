/*Juego arcade analógico donde el objetivo es meter la bola en el agujero correcto controlando la barra con dos joysticks. 
  Programado en Arduino Leonardo
  Autor: Javier Vargas. El Hormiguero.
  https://creativecommons.org/licenses/by/4.0/
*/
//PINES
#define PinSTEP1 8
#define PinSTEP2 5
#define PinDIR1 9
#define PinDIR2 6
#define PinFinalMotor1 A2
#define PinFinalMotor2 A3
#define PinLed1 A5
#define PinLed2 A4
#define PinLed3 4
#define PinBoton 7
#define PinInterruptor1 11
#define PinInterruptor2 10
#define PinInterruptor3 12
#define PinInterruptorFallo 13
#define PinJoystick1 A1
#define PinJoystick2 A0

//CONFIGURACION
#define Muestreo 20 //ms
//Motores
#define PasosVuelta 3200 //Pasos por vuelta del motor paso a paso
#define mmPaso 0.0025f //milimetros por paso del tornillo
#define K 0.3f //Flitro de la velocidad
#define velocidadMax 230 //rpm
#define velocidadMin 1 //rpm
#define velocidadLento 100 //rpm
#define DIRECCION1 LOW //
#define DIRECCION2 LOW //
#define distanciaMax 50 //(mm) Distancia permitida entre la rosca de ambos motores
#define alturaMax 240 //(mm) Altura maxima de los motores
#define MotorZero1 25 //Distancia que sube el motor 2 para recoger la bola
#define MotorZero2 43 //Distancia que sube para iniciar partida
#define MotorZero3 10 //Distancia por debajo de MotorZero2 que puede bajar
//Joystick
#define centro1 510 //Lectura analogica
#define max1 249 //Lectura analogica
#define min1 774 //Lectura analogica
#define centro2 511 //Lectura analogica
#define max2 261 //Lectura analogica
#define min2 761 //Lectura analogica
#define margen 5 //Margen entorno al centro 
//Partida
#define tiempoPartida 40 //Tiempo de una partida 
#define tiempoPartidaMin 10 //Tiempo minimo que puede durarar una partida
#define reduccionTiempo 10 //Reduccion de tiempo entre rondas

//LIBRERIAS Y VARIABLES

//Timer
#include <TimerOne.h>
#include <TimerThree.h>
#include <digitalWriteFast.h> //Más rapido!

//Pantalla
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
Adafruit_7segment matrix = Adafruit_7segment();

unsigned long m = 0, m2 = 0;
float rpm1, rpm2 = 0;
boolean dir1 = 1, dir2 = 1;
volatile unsigned long steps1, steps2 = 0;
byte objetivo = 0;
int tiempo = tiempoPartida;
int puntuacion = 0;
byte partida = 0;

void setup() {
  Serial.begin(115200);

  //PinMode
  pinMode(PinSTEP1, OUTPUT);
  pinMode(PinSTEP2, OUTPUT);
  pinMode(PinDIR1, OUTPUT);
  pinMode(PinDIR2, OUTPUT);
  pinMode(PinLed1, OUTPUT);
  pinMode(PinLed2, OUTPUT);
  pinMode(PinLed3, OUTPUT);
  pinMode(PinBoton, INPUT_PULLUP);
  pinMode(PinInterruptor1, INPUT_PULLUP);
  pinMode(PinInterruptor2, INPUT_PULLUP);
  pinMode(PinInterruptor3, INPUT_PULLUP);
  pinMode(PinInterruptorFallo, INPUT_PULLUP);
  pinMode(PinFinalMotor1, INPUT_PULLUP);
  pinMode(PinFinalMotor2, INPUT_PULLUP);

  //Inicio TIMER
  Timer1.initialize(250000);
  Timer1.attachInterrupt(Step1);
  Timer1.stop();

  Timer3.initialize(250000);
  Timer3.attachInterrupt(Step2);
  Timer3.stop();

  //Inicio 7 segmentos
  matrix.begin(0x70);
  Pantalla7segRayas();

  EfectoLeds();

  //Espera a pulsar el boton para empezar
  while (!Boton()) {
  }
}

////////////////////
////////LOOP////////
////////////////////

void loop() {

  //Coloca la barra en el inicio
  MotorZero();

  //Selecciona el agujero objetivo
  SeleccionarAgujero();

  //PARTIDA INICIADA!
  while (1) {
    if (m != millis() / Muestreo) {
      m = millis() / Muestreo;

      //Tiempo
      if (m2 != millis() / 1000) {
        m2 = millis() / 1000;
        tiempo--;
        Pantalla7segNumero(tiempo);
        //Fuera de tiempo
        if (tiempo == 0) {
          SetVelocidad(0, 0, 0);
          EfectoLeds();
          Pantalla7segParpadeo(puntuacion);
          //Espera a pulsar el boton
          while (!Boton()) {
          }
          //Reinicio de partida
          objetivo = 0;
          partida = 0;
          puntuacion = 0;
          tiempo = tiempoPartida;
          Pantalla7segRayas();
          return;
        }
      }

      //Velocidad obtenida con el joystick
      rpm1 = LecturaJoystick(analogRead(PinJoystick1), centro1, min1, max1, margen);
      rpm2 = LecturaJoystick(analogRead(PinJoystick2), centro2, min2, max2, margen);

      //Limita la velocidad rpm segun la posición de la tuerca
      Limites();

      //Timer de motores paso a paso
      SetVelocidad(rpm1, rpm2, 1);

      //Control de donde se cuela la bola
      switch (ControlBola()) {
        //Acierto
        case 1:
          SetVelocidad(0, 0, 0);
          Parpadeo(objetivo, 5);
          //Actualiza partida y tiempo de partida
          if (objetivo == 3) partida++;
          //Puntuacion
          Pantalla7segCuenta(puntuacion, puntuacion + objetivo * 50 + partida * 50 + tiempo);
          puntuacion += objetivo * 50 + partida * 50 + tiempo;
          Pantalla7segParpadeo(puntuacion);
          //Actualiza el tiempo de partida
          tiempo = constrain(tiempoPartida - partida * reduccionTiempo, tiempoPartidaMin, tiempoPartida);
          return;

        //Fallo
        case 2:
          SetVelocidad(0, 0, 0);
          EfectoLeds();
          Pantalla7segParpadeo(puntuacion);
          //Espera a pulsar el boton
          while (!Boton()) {
          }
          //Reinicio de partida
          objetivo = 0;
          partida = 0;
          puntuacion = 0;
          tiempo = tiempoPartida;
          Pantalla7segRayas();
          return;
      }
    }
  }
}


////////////////////
////////////////////
////////////////////

boolean Boton() {
  return !digitalRead(PinBoton);
}

void Pantalla7segNumero(int num) {
  matrix.print(num, DEC);
  matrix.writeDisplay();
  matrix.blinkRate(0);
  matrix.writeDisplay();
}

void Pantalla7segRayas() {
  matrix.print(10000, DEC);
  matrix.writeDisplay();
  matrix.blinkRate(0);
  matrix.writeDisplay();
}

void Pantalla7segBorrar() {
  matrix.clear();
  matrix.writeDisplay();
}

void Pantalla7segParpadeo(int num) {
  Pantalla7segNumero(num);
  matrix.blinkRate(2);
  matrix.writeDisplay();
}

void Pantalla7segCuenta(int num0, int num1) {
  for (int p = num0; p <= num1; p++) {
    Pantalla7segNumero(p);
    delay(5);
  }
}

byte SeleccionarAgujero() {
  //El objetivo es el agujero siguiente
  if (objetivo == 3) objetivo = 1;
  else objetivo++;
  //Enciende el led
  LedOn(objetivo);
}

byte ControlBola() {
  //Bola en agujero objetivo
  if (!digitalRead(PinInterruptor1)) {
    if (objetivo == 1) return 1;
    else return 2;
  }
  if (!digitalRead(PinInterruptor2)) {
    if (objetivo == 2) return 1;
    else return 2;
  }
  if (!digitalRead(PinInterruptor3)) {
    if (objetivo == 3) return 1;
    else return 2;
  }
  //Bola en agujero erróneo
  if (!digitalRead(PinInterruptorFallo)) return 2;
  //No hay bola detectada
  return 0;
}

void Limites() {

  //Limite por diferencia de altura entre motores
  float distancia = (float)((float)steps1 - (float)steps2) * mmPaso;
  //Motor 1 por encima de 2
  if (distancia >= distanciaMax) {
    if (rpm1 > 0) rpm1 = 0;
    if (rpm2 < 0) rpm2 = 0;
  }
  //Motor 2 por encima de 1
  if (distancia <= -distanciaMax) {
    if (rpm1 < 0) rpm1 = 0;
    if (rpm2 > 0) rpm2 = 0;
  }

  //Limite por final de recorrido superior
  if (steps1 * mmPaso >= alturaMax && rpm1 > 0) rpm1 = 0;
  if (steps2 * mmPaso >= alturaMax && rpm2 > 0) rpm2 = 0;

  //Limite por final de recorrido inferior
  if (steps1 == 0 && rpm1 < 0) rpm1 = 0;
  if (steps2 == 0 && rpm2 < 0) rpm2 = 0;
}

void MotorZero() {
  float distancia1 = 0, distancia2 = 0;
  float rpm1 = -velocidadMax;
  float rpm2 = -velocidadMax;

  //Bajamos la barra hasta dar con el final de carrera de ambos motores
  while (rpm1 != 0 || rpm2 != 0) {
    if (steps1 == 0) rpm1 = -velocidadLento;
    if (steps2 == 0) rpm2 = -velocidadLento;
    if (!digitalRead(PinFinalMotor1)) rpm1 = 0;
    if (!digitalRead(PinFinalMotor2)) rpm2 = 0;
    SetVelocidad(rpm1, rpm2, 0);
  }

  //Reinicio de la posicion 0
  steps1 = 0;
  steps2 = 0;

  //Subimos el primer motor para mover la bola
  rpm2 = velocidadMax;
  SetVelocidad(0, rpm2, 0);
  while (distancia2 < MotorZero1) {
    distancia2 = steps2 * mmPaso;
  }

  //Subimos ambos motores a la posicion cero
  rpm1 = velocidadMax;
  rpm2 = velocidadMax;
  while (rpm1 != 0 || rpm2 != 0) {
    distancia1 = steps1 * mmPaso;
    distancia2 = steps2 * mmPaso;
    if (distancia1 >= MotorZero2) rpm1 = 0;
    if (distancia2 >= MotorZero2) rpm2 = 0;
    SetVelocidad(rpm1, rpm2, 0);
  }

  //Posicion 0 a MotorZero3 mm por debajo
  steps1 = MotorZero3 / mmPaso;
  steps2 = MotorZero3 / mmPaso;
}

float LecturaJoystick(int a, int centro, int min, int max, int m) {
  float out = 0;
  if (a >= centro + m) out = map(a, centro + m, max, 0, velocidadMax);
  else if (a <= centro - m) out = map(a, centro - m, min, 0, -velocidadMax);
  return constrain(out, -velocidadMax, velocidadMax);
}

void SetVelocidad(float rpm1, float rpm2, boolean filtro) {
  static float rpm1_ant = 0;
  static float rpm2_ant = 0;

  static volatile boolean e1, e2 = 0; //Estado de los timers

  //Limite de velocidad entre los limites de velocidad maxima
  rpm1 = constrain(rpm1, -velocidadMax, velocidadMax);
  rpm2 = constrain(rpm2, -velocidadMax, velocidadMax);

  //Filtro de la velocidad para evitar cambios bruscos de aceleracion
  if (filtro) {
    rpm1 = (float)rpm1 * K + (float)rpm1_ant * (1 - K);
    rpm2 = (float)rpm2 * K + (float)rpm2_ant * (1 - K);
    rpm1_ant = rpm1;
    rpm2_ant = rpm2;
  }

  //Direccion motor 1
  if (rpm1 >= 0) {
    if (!dir1) {
      dir1 = 1;
      digitalWriteFast(PinDIR1, DIRECCION1);
    }
  }
  else if (dir1) {
    dir1 = 0;
    digitalWriteFast(PinDIR1, !DIRECCION1);
  }

  //Direccion motor 2
  if (rpm2 >= 0) {
    if (!dir2) {
      dir2 = 1;
      digitalWriteFast(PinDIR2, DIRECCION2);
    }
  }
  else if (dir2) {
    dir2 = 0;
    digitalWriteFast(PinDIR2, !DIRECCION2);
  }

  //Velocidad motor 1
  if (abs(rpm1) >= velocidadMin) {
    unsigned long T1 = 30000000 / (PasosVuelta * abs(rpm1));
    if (!e1) { //Inicia el timer
      Timer1.start();
      //digitalWriteFast(PinENABLE1, LOW); //Activa el motor
      e1 = 1;
    }
    Timer1.setPeriod(T1); //Periodo de la interrpucion
  }
  else if (e1) { //Desactiva el timer
    Timer1.stop();
    //digitalWriteFast(PinENABLE1, HIGH); //Desactiva el motor
    e1 = 0;
  }

  //Velocidad motor 2
  if (abs(rpm2) >= velocidadMin) {
    unsigned long T2 = 30000000 / (PasosVuelta * abs(rpm2));
    if (!e2) {  //Inicia el timer
      Timer3.start();
      //digitalWriteFast(PinENABLE2, LOW); //Activa el motor
      e2 = 1;
    }
    Timer3.setPeriod(T2); //Periodo de la interrpucion
  }
  else if (e2) { //Desactiva el timer
    Timer3.stop();
    //digitalWriteFast(PinENABLE2, HIGH); //Desactiva el motor
    e2 = 0;
  }

}

//STEP
void Step1() {
  static volatile boolean e = 0;
  if (e) {
    digitalWriteFast(PinSTEP1, LOW);
    e = 0;
  }
  else {
    digitalWriteFast(PinSTEP1, HIGH);
    e = 1;
    if (dir1) steps1++;
    else if (steps1 > 0) steps1--;
  }
}
void Step2() {
  static volatile boolean e = 0;
  if (e) {
    digitalWriteFast(PinSTEP2, LOW);
    e = 0;
  }
  else {
    digitalWriteFast(PinSTEP2, HIGH);
    e = 1;
    if (dir2) steps2++;
    else if (steps2 > 0) steps2--;
  }
}

void LedOn(int led) {
  switch (led) {
    case 0:
      digitalWrite(PinLed1, LOW);
      digitalWrite(PinLed2, LOW);
      digitalWrite(PinLed3, LOW);
      break;
    case 1:
      digitalWrite(PinLed1, HIGH);
      digitalWrite(PinLed2, LOW);
      digitalWrite(PinLed3, LOW);
      break;
    case 2:
      digitalWrite(PinLed1, LOW);
      digitalWrite(PinLed2, HIGH);
      digitalWrite(PinLed3, LOW);
      break;
    case 3:
      digitalWrite(PinLed1, LOW);
      digitalWrite(PinLed2, LOW);
      digitalWrite(PinLed3, HIGH);
      break;
  }
}

void Parpadeo(byte led, byte n) {
  for (byte i = 0; i < n; i++) {
    LedOn(led);
    delay(100);
    LedOn(0);
    delay(100);
  }
}

void EfectoLeds() {
  for (byte i = 0; i < 5; i++) {
    for (byte led = 1; led <= 3; led++) {
      LedOn(led);
      delay(100);
    }
  }
  LedOn(0);
}


