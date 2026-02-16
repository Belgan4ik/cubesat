//CUBESAT - Модуль системы наведения

#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// ===== КОНСТАНТЫ КОНФИГУРАЦИИ =====

// Пины подключения
const int SERVO_X_PIN = 5;        // Ось наклона
const int SERVO_Y_PIN = 3;        // Ось поворота
const int LASER_PIN = 4;          // Лазерный модуль
const int RF24_CE_PIN = 9;
const int RF24_CSN_PIN = 10;

// Начальные позиции сервоприводов
const int INITIAL_SERVO_X = 96;
const int INITIAL_SERVO_Y = 95;

// Параметры сканирования
const int SCAN_MIN = -40;         // Минимальный угол сканирования
const int SCAN_MAX = 40;          // Максимальный угол сканирования
const int SCAN_STEP = 10;         // Шаг сканирования в градусах
const float SERVO_CORRECTION = 1.05; // Коэффициент коррекции углов

// Параметры передачи данных
const unsigned long TRANSMIT_DELAY = 3000;  // Задержка между передачами (мс)
const int MAX_RETRIES = 3;        // Максимальное количество попыток отправки
const int RETRY_DELAY = 100;      // Задержка между попытками (мс)
const int SERVO_STEP_DELAY = 20;  // Задержка для плавного движения серво (мс)

// Адреса для радиосвязи
const byte ADDRESS_STATION[6] = "STATN";  // Адрес наземной станции
const byte ADDRESS_CUBESAT[6] = "CBSAT";  // Адрес спутника

// Размер буферов передачи
const int BUFFER_SIZE = 32;

// ===== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ =====

RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);
Servo servoX;  // Сервопривод наклона
Servo servoY;  // Сервопривод поворота

// ===== ПЕРЕЧИСЛЕНИЯ =====

// Режимы работы системы
enum Mode {
  MODE_IDLE,      // Ожидание команд
  MODE_H_SCAN,    // Горизонтальное сканирование
  MODE_V_SCAN,    // Вертикальное сканирование
  MODE_D_SCAN1,   // Диагональное сканирование 1
  MODE_D_SCAN2    // Диагональное сканирование 2
};

// ===== СОСТОЯНИЕ СИСТЕМЫ =====

struct SystemState {
  int servoX;           // Текущая позиция серво X
  int servoY;           // Текущая позиция серво Y
  int angleX;           // Текущий угол X (относительный)
  int angleY;           // Текущий угол Y (относительный)
  Mode currentMode;     // Текущий режим работы
  bool laserEnabled;    // Состояние лазера
  char rxBuffer[BUFFER_SIZE];  // Буфер приема
  char txBuffer[BUFFER_SIZE];  // Буфер передачи
};

SystemState state = {
  INITIAL_SERVO_X, 
  INITIAL_SERVO_Y, 
  0, 
  0, 
  MODE_IDLE, 
  false,
  "",
  ""
};

// ===== ФУНКЦИИ ИНИЦИАЛИЗАЦИИ =====

// Инициализация серво и лазера
void initHardware() {
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  
  pinMode(LASER_PIN, OUTPUT);
  setLaser(false);
  
  // Установка начальной позиции
  servoX.write(state.servoX);
  servoY.write(state.servoY);
}

// Инициализация радиомодуля
bool initRadio() {
  if (!radio.begin()) {
    Serial.println(F("ОШИБКА: nRF24L01+ не обнаружен!"));
    return false;
  }
  
  // Настройка параметров радио
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setRetries(5, 15);
  
  // Настройка каналов связи
  radio.openWritingPipe(ADDRESS_STATION);    // Передача на станцию
  radio.openReadingPipe(1, ADDRESS_CUBESAT); // Прием на адрес спутника
  
  radio.startListening();
  
  return true;
}

// ===== УПРАВЛЕНИЕ ЛАЗЕРОМ =====

// Включение/выключение лазера
void setLaser(bool enable) {
  digitalWrite(LASER_PIN, enable ? HIGH : LOW);
  state.laserEnabled = enable;
  Serial.print(F("Лазер: "));
  Serial.println(enable ? F("ВКЛ") : F("ВЫКЛ"));
}

// ===== УПРАВЛЕНИЕ СЕРВОПРИВОДАМИ =====

// Плавное перемещение сервоприводов
void smoothMove(int targetX, int targetY) {
  int stepsX = abs(targetX - state.servoX);
  int stepsY = abs(targetY - state.servoY);
  int totalSteps = max(stepsX, stepsY);
  
  if (totalSteps == 0) return;
  
  // Интерполяция движения
  for (int i = 0; i <= totalSteps; i++) {
    int posX = map(i, 0, totalSteps, state.servoX, targetX);
    int posY = map(i, 0, totalSteps, state.servoY, targetY);
    
    servoX.write(posX);
    servoY.write(posY);
    Serial.println(posX);
    Serial.println(posY);
    delay(SERVO_STEP_DELAY);
  }
  
  state.servoX = targetX;
  state.servoY = targetY;
}

// Установка позиции по относительным углам
void setPosition(int x, int y) {
  state.angleX = x;
  state.angleY = y;
  
  // Преобразование относительных углов в абсолютные с коррекцией
  int targetX = INITIAL_SERVO_X + (int)(x * SERVO_CORRECTION);
  int targetY = INITIAL_SERVO_Y + (int)(y * SERVO_CORRECTION);
  
  smoothMove(targetX, targetY);
  
  Serial.print(F("Позиция: X="));
  Serial.print(x);
  Serial.print(F(" Y="));
  Serial.println(y);
}

// Возврат в начальную позицию
void returnToHome() {
  smoothMove(INITIAL_SERVO_X, INITIAL_SERVO_Y);
  state.servoX = INITIAL_SERVO_X;
  state.servoY = INITIAL_SERVO_Y;
  state.angleX = 0;
  state.angleY = 0;
}

// ===== ПЕРЕДАЧА ДАННЫХ =====

// Отправка данных с повторными попытками
bool sendWithRetry(const char* data) {
  bool success = false;
  
  for (int attempt = 1; attempt <= MAX_RETRIES && !success; attempt++) {
    radio.stopListening();
    success = radio.write(data, BUFFER_SIZE);
    radio.startListening();
    
    if (success) {
      Serial.print(F("Отправлено (попытка "));
      Serial.print(attempt);
      Serial.print(F("): "));
      Serial.println(data);
    } else {
      Serial.print(F("Ошибка (попытка "));
      Serial.print(attempt);
      Serial.print(F("/"));
      Serial.print(MAX_RETRIES);
      Serial.println(F(")"));
      
      if (attempt < MAX_RETRIES) {
        delay(RETRY_DELAY);
      }
    }
  }
  
  if (!success) {
    Serial.println(F("КРИТИЧЕСКАЯ ОШИБКА: передача не удалась!"));
  }
  
  return success;
}

// Отправка телеметрии
void transmitTelemetry() {
  snprintf(state.txBuffer, BUFFER_SIZE, "X:%d Y:%d", state.angleX, state.angleY);
  sendWithRetry(state.txBuffer);
}

// ===== РЕЖИМЫ СКАНИРОВАНИЯ =====

// Выполнение сканирования с задержкой и телеметрией
void scanPosition(int x, int y) {
  setPosition(x, y);
  delay(TRANSMIT_DELAY);
  transmitTelemetry();
}

// Горизонтальное сканирование
void horizontalScan() {
  state.currentMode = MODE_H_SCAN;
  Serial.println(F("Горизонтальное сканирование"));
  
  for (int y = SCAN_MIN; y <= SCAN_MAX; y += SCAN_STEP) {
    scanPosition(0, y);
  }
}

// Вертикальное сканирование
void verticalScan() {
  state.currentMode = MODE_V_SCAN;
  Serial.println(F("Вертикальное сканирование"));
  
  for (int x = SCAN_MIN; x <= SCAN_MAX; x += SCAN_STEP) {
    scanPosition(x, 0);
  }
}

// Диагональное сканирование (тип 1)
void diagonalScan1() {
  state.currentMode = MODE_D_SCAN1;
  Serial.println(F("Диагональное сканирование 1"));
  
  for (int angle = SCAN_MIN; angle <= SCAN_MAX; angle += SCAN_STEP) {
    scanPosition(angle, angle);
  }
}

// Диагональное сканирование (тип 2)
void diagonalScan2() {
  state.currentMode = MODE_D_SCAN2;
  Serial.println(F("Диагональное сканирование 2"));
  
  int steps = (SCAN_MAX - SCAN_MIN) / SCAN_STEP;
  for (int i = 0; i <= steps; i++) {
    int x = SCAN_MIN + i * SCAN_STEP;
    int y = SCAN_MAX - i * SCAN_STEP;
    scanPosition(x, y);
  }
}

// Полный цикл сканирования
void executeScanCycle() {
  Serial.println(F("===== НАЧАЛО ЦИКЛА СКАНИРОВАНИЯ ====="));
  
  horizontalScan();
  verticalScan();
  diagonalScan1();
  diagonalScan2();
  
  Serial.println(F("===== ЦИКЛ ЗАВЕРШЕН ====="));
}

// ===== ОБРАБОТКА КОМАНД =====

// Обработка команды START
void handleStartCommand() {
  Serial.println(F("Запуск автоматического сканирования"));
  
  setLaser(true);
  delay(1000);  // Пауза для стабилизации лазера
  
  executeScanCycle();
  
  setLaser(false);
  returnToHome();
  
  state.currentMode = MODE_IDLE;
  Serial.println(F("Возврат в режим ожидания"));
}

// Обработка команды управления лазером
void handleLaserCommand(bool enable) {
  setLaser(enable);
  snprintf(state.txBuffer, BUFFER_SIZE, "LASER:%s", enable ? "ON" : "OFF");
  sendWithRetry(state.txBuffer);
}

// Обработка ручного управления
void handleManualCommand(const char* cmd) {
  int x, y;
  
  if (sscanf(cmd, "%d %d", &x, &y) != 2) {
    Serial.println(F("ОШИБКА: неверный формат команды"));
    return;
  }
  
  // Проверка кратности шагу
  if (x % SCAN_STEP != 0 || y % SCAN_STEP != 0) {
    Serial.print(F("ОШИБКА: используйте шаг "));
    Serial.print(SCAN_STEP);
    Serial.println(F(" градусов"));
    return;
  }
  
  // Вычисление абсолютных углов
  int realX = INITIAL_SERVO_X + x;
  int realY = INITIAL_SERVO_Y + y;
  
  // Проверка допустимого диапазона
  if (realX < 0 || realX > 180 || realY < 0 || realY > 180) {
    Serial.println(F("ОШИБКА: выход за пределы 0-180°"));
    return;
  }
  
  smoothMove(realX, realY);
  
  Serial.print(F("Установлено: X="));
  Serial.print(realX);
  Serial.print(F("° Y="));
  Serial.print(realY);
  Serial.println(F("°"));
  
  // Отправка подтверждения
  snprintf(state.txBuffer, BUFFER_SIZE, "X:%d Y:%d", x, y);
  sendWithRetry(state.txBuffer);
}

// Обработка входящих команд
void processCommand() {
  if (!radio.available()) return;
  
  radio.read(state.rxBuffer, BUFFER_SIZE);
  
  Serial.print(F("← Команда: "));
  Serial.println(state.rxBuffer);
  
  // Разбор команд
  if (strcmp(state.rxBuffer, "START") == 0) {
    handleStartCommand();
  } 
  else if (strcmp(state.rxBuffer, "ON") == 0) {
    handleLaserCommand(true);
  } 
  else if (strcmp(state.rxBuffer, "OFF") == 0) {
    handleLaserCommand(false);
  } 
  else {
    handleManualCommand(state.rxBuffer);
  }
  
  // Очистка буфера
  memset(state.rxBuffer, 0, BUFFER_SIZE);
}

// ===== ОСНОВНЫЕ ФУНКЦИИ =====

void setup() {
  Serial.begin(9600);

  
  initHardware();
  
  if (!initRadio()) {
    Serial.println(F("КРИТИЧЕСКАЯ ОШИБКА: остановка системы"));
    while (1);
  }
  
  Serial.println(F("Система готова"));
  Serial.print(F("Начальная позиция: X="));
  Serial.print(state.servoX);
  Serial.print(F("° Y="));
  Serial.print(state.servoY);
  Serial.println(F("°"));
  Serial.println(F("Ожидание команд..."));
}

void loop() {
  processCommand();
}