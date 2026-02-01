// Transmitter (Leonardo)
#include <SPI.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10
RF24 radio(CE_PIN, CSN_PIN);
const byte RX_PIPE[6] = "1Node"; // приемник (Nano)
const byte TX_PIPE[6] = "2Node";

struct CmdPacket { uint8_t start; uint8_t device_id; uint8_t cmd; uint8_t reserved; uint8_t crc; };
uint8_t crc8(const uint8_t *buf, size_t len){ uint8_t c=0; while(len--) c ^= *buf++; return c; }

const uint8_t TARGET_ID = 0x05; // id приёмника

void setup(){
  SPI.begin();
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.openWritingPipe(RX_PIPE);
  radio.openReadingPipe(1, TX_PIPE);
  radio.stopListening();
}

void loop(){
  CmdPacket p;
  p.start = 0xAA;
  p.device_id = TARGET_ID;
  p.cmd = 0x01; // старт последовательности
  p.reserved = 0;
  p.crc = crc8((uint8_t*)&p, sizeof(p)-1);
  bool ok = radio.write(&p, sizeof(p));
  // можно читать ответ с радио, если нужно
  delay(10000); // тест — отправлять раз в 10s
}
// Receiver (Nano)
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

#define CE_PIN 9
#define CSN_PIN 10
RF24 radio(CE_PIN, CSN_PIN);
const byte RX_PIPE[6] = "1Node";
const byte TX_PIPE[6] = "2Node";

const uint8_t DEVICE_ID = 0x05;

Servo servoX, servoY;
const int PIN_SERVO_X = 3; // ось X (вправо-влево)
const int PIN_SERVO_Y = 5; // ось Y (вверх-вниз)
const int PIN_LASER   = 4;

enum Mode : uint8_t {MODE_IDLE=0, MODE_SCAN=1};
struct CmdPacket { uint8_t start; uint8_t device_id; uint8_t cmd; uint8_t reserved; uint8_t crc; };
struct StatusPacket { uint8_t start; uint8_t device_id; int8_t tilt; int8_t pan; uint8_t mode; uint16_t seq; };

uint8_t crc8(const uint8_t *buf, size_t len){ uint8_t c=0; while(len--) c ^= *buf++; return c; }

int8_t curTilt=0, curPan=0;
const int STEP = 10, MIN_ANGLE = -40, MAX_ANGLE = 40;
const unsigned long MOVE_INTERVAL = 3000UL;
unsigned long lastMoveAt = 0;
uint16_t statusSeq = 0;

void setup(){
  pinMode(PIN_LASER, OUTPUT); digitalWrite(PIN_LASER, LOW);
  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);
  setAngles(0,0);
  SPI.begin();
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.openReadingPipe(1, RX_PIPE);
  radio.openWritingPipe(TX_PIPE);
  radio.startListening();
}

void loop(){
  if(radio.available()){
    CmdPacket cmd;
    radio.read(&cmd, sizeof(cmd));
    if(cmd.start==0xAA && cmd.device_id==DEVICE_ID && cmd.crc==crc8((uint8_t*)&cmd, sizeof(cmd)-1)){
      if(cmd.cmd==0x01){
        radio.stopListening();
        executeAllScans();
        radio.startListening();
      }
    }
  }
}

void executeAllScans(){
  lastMoveAt = millis() - MOVE_INTERVAL;
  // Горизонтальный (pan=0, tilt -40..40)
  runScan([](int idx){ int t = MIN_ANGLE + idx*STEP; setAngles(t, 0); }, (MAX_ANGLE-MIN_ANGLE)/STEP + 1);
  // Вертикальный (tilt=0, pan -40..40)
  runScan([](int idx){ int p = MIN_ANGLE + idx*STEP; setAngles(0, p); }, (MAX_ANGLE-MIN_ANGLE)/STEP + 1);
  // Диагональ1 (-40,-40 -> 40,40)
  runScan([](int idx){ int v = MIN_ANGLE + idx*STEP; setAngles(v, v); }, (MAX_ANGLE-MIN_ANGLE)/STEP + 1);
  // Диагональ2 (-40,40 -> 40,-40)
  runScan([](int idx){ int t = MIN_ANGLE + idx*STEP; int p = MAX_ANGLE - idx*STEP; setAngles(t, p); }, (MAX_ANGLE-MIN_ANGLE)/STEP + 1);
  setAngles(0,0);
}

template<typename F> void runScan(F setter, int steps){
  for(int i=0;i<steps;i++){
    while(millis() - lastMoveAt < MOVE_INTERVAL) { /* ждать */ }
    setter(i);
    sendStatusPacket(MODE_SCAN);
    lastMoveAt = millis();
  }
}

void setAngles(int8_t tilt, int8_t pan){
  tilt = constrain((int)tilt, MIN_ANGLE, MAX_ANGLE);
  pan  = constrain((int)pan, MIN_ANGLE, MAX_ANGLE);
  tilt = round((float)tilt/STEP)*STEP;
  pan  = round((float)pan/STEP)*STEP;
  curTilt = tilt; curPan = pan;
  int sTilt = map(tilt, MIN_ANGLE, MAX_ANGLE, 0, 180);
  int sPan  = map(pan,  MIN_ANGLE, MAX_ANGLE, 0, 180);
  servoX.write(sPan);   // X — pan
  servoY.write(sTilt);  // Y — tilt
  digitalWrite(PIN_LASER, HIGH); // включаем лазер при позиционировании (по необходимости)
}

void sendStatusPacket(uint8_t mode){
  StatusPacket s;
  s.start = 0xAB;
  s.device_id = DEVICE_ID;
  s.tilt = curTilt;
  s.pan = curPan;
  s.mode = mode;
  s.seq = ++statusSeq;
  radio.stopListening();
  radio.write(&s, sizeof(s));
  radio.startListening();
}
