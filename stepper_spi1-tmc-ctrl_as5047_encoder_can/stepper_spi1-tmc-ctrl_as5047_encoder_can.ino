/*
Мотор вращается в одну сторону
57HS100-4204A-E1000  I=4.2A Angular step 1.8° Phase number: 2
encoder-wires: PB6:EA+  PC7:EB+  PA0:EA- (not used)  NC:EB- (not used)

  Важно! Драйвер управляет мотором по SPI_1
  TMC5160 - SPI1:
    NSS  PA4
    SCK  PA5
    MOSI PA7
    MISO PA6

  Датчик AS5047P подключен к SPI3
  AS5047P - SPI3:
    NSS  PA15
    SCK  PC10
    MOSI PC12
    MISO PC11

  Библиотека для работы с датчиком AS5047P https://github.com/VBCores/AS5047P 
*/

#include <SPI.h>
#include <VBCoreG4_arduino_system.h>
#include <AS5047P.h>

#define SPI1_SCK  PA5
#define SPI1_MOSI PA7
#define SPI1_MISO PA6

#define SPI3_SCK  PC10
#define SPI3_MOSI PC12
#define SPI3_MISO PC11

// Энкодер
#define Enc_A PB6
#define Enc_B PC7

// Режимы TMC5160
#define SD_MODE  PA12
#define SPI_MODE PA11

// Выводы драйвера
#define EN_PIN   PC5
#define DIR_PIN  PA9
#define STEP_PIN PA8

#define SPI1_NSS_PIN PA4

// Настройки SPI1 для TMC5160.
// 4 МГц частота SPI1, соответствует скорости передачи одной команды на TMC5160 16 мкс
static constexpr uint32_t MOTOR_SPI_FREQ = 4000000;
SPISettings motorSPISettings(MOTOR_SPI_FREQ, MSBFIRST, SPI_MODE3);

// SPI3 для датчика AS5047P
SPIClass SPI_3(SPI3_MOSI, SPI3_MISO, SPI3_SCK);

// Датчик AS5047P
#define AS5047P_CHIP_SELECT_PORT PA_15_ALT1
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

// Энкодер
constexpr int32_t IPR = 2000;
volatile int32_t encoderTicks = 0;

// CAN FD
uint8_t can_data_send[4] = {170, 171, 172, 173};

unsigned long t = 0;

FDCAN_HandleTypeDef* hfdcan1;
CanFD* canfd;
FDCAN_TxHeaderTypeDef TxHeader;


// ====================================
// Прерывание энкодера
// ====================================

void ISR_A() {
  if (digitalRead(Enc_A) == digitalRead(Enc_B)) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}


// ====================================
// Передача данных в TMC5160 по SPI1
// ====================================

void sendData(uint8_t address, uint32_t datagram) {

  // Настройка SPI перед транзакцией
  SPI.beginTransaction(motorSPISettings);

  // Выбор TMC5160
  digitalWrite(SPI1_NSS_PIN, LOW);
  delayMicroseconds(10);

  // Адрес регистра
  SPI.transfer(address);

  // 32 бита данных, старший байт первым
  SPI.transfer((datagram >> 24) & 0xFF);
  SPI.transfer((datagram >> 16) & 0xFF);
  SPI.transfer((datagram >> 8) & 0xFF);
  SPI.transfer(datagram & 0xFF);

  // Завершение передачи
  digitalWrite(SPI1_NSS_PIN, HIGH);

  SPI.endTransaction();
}


// ====================================
// Конфигурация двигателя
// ====================================

void motor_config() {

  // CS сначала переводим в неактивное состояние
  pinMode(SPI1_NSS_PIN, OUTPUT);
  digitalWrite(SPI1_NSS_PIN, HIGH);

  // TMC5160 подключен к SPI1
  SPI.setMOSI(SPI1_MOSI);
  SPI.setMISO(SPI1_MISO);
  SPI.setSCLK(SPI1_SCK);

  SPI.begin();

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(SD_MODE, OUTPUT);
  pinMode(SPI_MODE, OUTPUT);

  digitalWrite(SD_MODE, LOW);
  digitalWrite(SPI_MODE, HIGH);
  digitalWrite(EN_PIN, LOW);

  // Конфигурация TMC5160
  sendData(0x80, 0x00000000); // GCONF

  sendData(0xEC, 0x000101D5); // CHOPCONF
  sendData(0x90, 0x00070603); // IHOLD_IRUN
  sendData(0x91, 0x0000000A); // TPOWERDOWN

  sendData(0xF0, 0x00000000); // PWMCONF

  sendData(0xA4, 0x000003E8); // A1 = 1000
  sendData(0xA5, 0x000186A0); // V1 = 100000
  sendData(0xA6, 0x0000C350); // AMAX = 50000
  sendData(0xA7, 0x000186A0); // VMAX = 100000
  sendData(0xAA, 0x00000578); // D1 = 1400
  sendData(0xAB, 0x0000000A); // VSTOP = 10

  sendData(0xA0, 0x00000000); // RAMPMODE = 0

  sendData(0xA1, 0x00000000); // XACTUAL = 0
  sendData(0xAD, 0x00000000); // XTARGET = 0
}


// ====================================
// Конфигурация CAN FD
// ====================================

void can_config(int ID) {

  SystemClock_Config();

  canfd = new CanFD();

  canfd->init();
  canfd->write_default_params();
  canfd->apply_config();

  hfdcan1 = canfd->get_hfdcan();
  canfd->default_start();

  TxHeader.Identifier = ID;
  TxHeader.DataLength = FDCAN_DLC_BYTES_4;
  TxHeader.IdType = FDCAN_EXTENDED_ID;

  delay(10);
}


// ====================================
// Обмен данными по CAN FD
// ====================================

void can_send_recv() {

  // Отправка CAN FD
  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan1) != 0) {

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &TxHeader, can_data_send) != HAL_OK) {
      Error_Handler();
    }
    else {
      digitalToggle(PD2);
    }
  }

  // Получение CAN FD
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan1, FDCAN_RX_FIFO0) > 0) {

    FDCAN_RxHeaderTypeDef Header;
    uint8_t RxData[64] = {0};

    if (HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &Header, RxData) != HAL_OK) {
      Error_Handler();
    }
    else {
      Serial.print("ID ");
      Serial.print(Header.Identifier);

      Serial.print(" data: ");

      Serial.print(RxData[0]);
      Serial.print("  ");
      Serial.print(RxData[1]);
      Serial.print("  ");
      Serial.print(RxData[2]);
      Serial.print("  ");
      Serial.print(RxData[3]);
      Serial.println();
    }
  }
}


// ====================================
// Setup
// ====================================

void setup() {

  Serial.begin(115200);

  pinMode(PD2, OUTPUT);

  // Энкодер
  pinMode(Enc_B, INPUT);
  pinMode(Enc_A, INPUT);

  attachInterrupt(digitalPinToInterrupt(Enc_A), ISR_A, CHANGE);

  // CAN FD
  can_config(0x01);

  // TMC5160
  motor_config();

  // AS5047P подключен к SPI3
  while (!as5047p.initSPI(&SPI_3)) {
    Serial.println(F("Can't connect to the AS5047P sensor!"));
    delay(3000);
  }
}


// ====================================
// Loop
// ====================================

int32_t target = 200000;
int32_t lastTarget = 0;

void loop() {
  // Команда через Serial
  if (Serial.available() > 0) {
    target = Serial.readStringUntil('\n').toInt();
  }

  // Обновление целевой позиции TMC5160
  if (target != lastTarget) {
    sendData(0xAD, (uint32_t)target);
    lastTarget = target;
  }

  // CAN FD
  if (millis() - t >= 100) {
    can_send_recv();
    t = millis();
  }

  // Считывание энкодера
  noInterrupts();
  int32_t ticks = encoderTicks;
  interrupts();

  float revolutions = (float)ticks / IPR;

  // Диагностика
  Serial.print("Angle: ");
  Serial.print(as5047p.readAngleDegree());

  Serial.print(" Ticks: ");
  Serial.print(ticks);

  Serial.print(" Revolutions: ");
  Serial.println(revolutions, 4);

  delay(100);
}
