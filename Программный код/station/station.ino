/*
 * СТАНЦИЯ УПРАВЛЕНИЯ
 * Отправляет команду запуска на CubeSat
 * Введите "START" для запуска цикла сканирования на CubeSat
 */

#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);

const byte address_station[6] = "STATN";  // Адрес станции
const byte address_cubesat[6] = "CBSAT";  // Адрес CubeSat

char txBuffer[32] = "";
char rxBuffer[32] = "";
int txIndex = 0;

void setup() {
  Serial.begin(9600);
  
  // проверяем подключен ли радиомодуль
  if (!radio.begin()) {
    Serial.println("Ошибка инициализации nRF24L01+!");
    while (1);
  }
  
  // настраиваем параметры радиомодуля
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setRetries(5, 15);
  
  // настраиваем куда отправлять и откуда читать
  radio.openWritingPipe(address_cubesat);    // Пишем на CubeSat
  radio.openReadingPipe(1, address_station);  // Читаем на адрес станции
  
  // начинаем слушать эфир
  radio.startListening();
  
  Serial.println("СТАНЦИЯ УПРАВЛЕНИЯ");
  Serial.println("Введите START для запуска");
  Serial.println("Или введите углы (шаг 10), например: 30 20");
}

void loop() {
  // принимаем ответы от CubeSat
  if (radio.available()) {
    // читаем данные из радиоканала
    radio.read(&rxBuffer, sizeof(rxBuffer));
    // выводим на экран что получили
    Serial.println(rxBuffer);
    // очищаем буфер приема
    memset(rxBuffer, 0, sizeof(rxBuffer));
  }
  
  // смотрим есть ли данные в сериал порту
  while (Serial.available() > 0) {
    // читаем по одному символу
    char c = Serial.read();
    
    // если нажали Enter
    if (c == '\n' || c == '\r') {
      // и если что-то ввели
      if (txIndex > 0) {
        // ставим конец строки
        txBuffer[txIndex] = '\0';
        
        // останавливаем прослушку чтобы отправить
        radio.stopListening();
        // отправляем команду
        bool success = radio.write(&txBuffer, sizeof(txBuffer));
        // возвращаемся к прослушке
        radio.startListening();
        
        // проверяем отправилось ли
        if (success) {
          Serial.print("Отправлено: ");
          Serial.println(txBuffer);
        } else {
          Serial.println("Ошибка отправки");
        }
        
        // очищаем буфер передачи
        memset(txBuffer, 0, sizeof(txBuffer));
        txIndex = 0;
      }
    } else {
      // добавляем символ в буфер если еще есть место
      if (txIndex < 31) {
        txBuffer[txIndex++] = c;
      }
    }
  }
}
