
/*
  57HSE82, step 1.8 degrees
  TMC5160 + STM32G474

  Управление:
  - После запуска постоянное вращение в отрицательную сторону
  - Число через Serial: движение к абсолютной позиции
  - После достижения позиции: остановка и ожидание
  - Новая позиция: новое движение
  - Команда R: постоянное вращение

  TMC5160 SPI1:
    NSS  PA4
    SCK  PA5
    MOSI PA7
    MISO PA6

  AS5047P SPI3:
    NSS  PA15
    SCK  PC10
    MOSI PC12
    MISO PC11
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

#define LED1 PD2

// Регистры TMC5160
#define REG_RAMPMODE  0x20
#define REG_XACTUAL   0x21
#define REG_XTARGET   0x2D
#define REG_RAMP_STAT 0x35

// Режимы TMC5160
#define RAMP_POSITION 0 //Движение к заданной позиции XTARGET
#define RAMP_VEL_NEG  2 //Постоянное вращение в отрицательном направлении

// SPI1 для TMC5160
static constexpr uint32_t MOTOR_SPI_FREQ = 4000000;
SPISettings motorSPISettings(MOTOR_SPI_FREQ, MSBFIRST, SPI_MODE3);

// SPI3 для AS5047P
SPIClass SPI_3(SPI3_MOSI, SPI3_MISO, SPI3_SCK);

#define AS5047P_CHIP_SELECT_PORT PA_15_ALT1
#define AS5047P_CUSTOM_SPI_BUS_SPEED 1000000

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
// Состояния двигателя
// ====================================

enum MotorState {
  MOTOR_ROTATING,
  MOTOR_POSITIONING,
  MOTOR_WAITING
};

MotorState motorState = MOTOR_ROTATING;
int32_t targetPosition = 0;
unsigned long positionCheckTime = 0;

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
// Запись регистра TMC5160
// ====================================

void sendData(uint8_t address, uint32_t datagram) {
  SPI.beginTransaction(motorSPISettings);

  digitalWrite(SPI1_NSS_PIN, LOW);
  delayMicroseconds(10);

  SPI.transfer(address);

  SPI.transfer((datagram >> 24) & 0xFF);
  SPI.transfer((datagram >> 16) & 0xFF);
  SPI.transfer((datagram >> 8) & 0xFF);
  SPI.transfer(datagram & 0xFF);

  digitalWrite(SPI1_NSS_PIN, HIGH);

  SPI.endTransaction();
}


// ====================================
// Чтение регистра TMC5160
// ====================================

uint32_t readData(uint8_t address) {
  uint32_t result = 0;

  // Ответ TMC5160 приходит на следующей SPI-транзакции
  for (int i = 0; i < 2; i++) {
    SPI.beginTransaction(motorSPISettings);

    digitalWrite(SPI1_NSS_PIN, LOW);
    delayMicroseconds(10);

    SPI.transfer(address & 0x7F);

    uint32_t value = 0;

    for (int j = 0; j < 4; j++) {
      value = (value << 8) | SPI.transfer(0x00);
    }

    digitalWrite(SPI1_NSS_PIN, HIGH);
    SPI.endTransaction();

    result = value;
    delayMicroseconds(10);
  }

  return result;
}


// ====================================
// Конфигурация двигателя
// ====================================

void motor_config() {
  pinMode(SPI1_NSS_PIN, OUTPUT);
  digitalWrite(SPI1_NSS_PIN, HIGH);

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

  // TMC5160
  sendData(0x80, 0x00000000); // GCONF

  sendData(0xEC, 0x000101D5); // CHOPCONF
  sendData(0x90, 0x00070603); // IHOLD_IRUN
  sendData(0x91, 0x0000000A); // TPOWERDOWN

  sendData(0xF0, 0x00000000); // PWMCONF

  // Параметры движения
  sendData(0xA4, 0x000003E8); // A1 = 1000
  sendData(0xA5, 0x000186A0); // V1 = 100000
  sendData(0xA6, 0x0000C350); // AMAX = 50000
  sendData(0xA7, 0x000186A0); // VMAX = 100000
  sendData(0xA8, 0x0000C350); // DMAX = 50000
  sendData(0xAA, 0x00000578); // D1 = 1400
  sendData(0xAB, 0x0000000A); // VSTOP = 10

  // Начальные координаты
  sendData(0xA0, RAMP_POSITION);
  sendData(0xA1, 0x00000000); // XACTUAL = 0
  sendData(0xAD, 0x00000000); // XTARGET = 0
}


// ====================================
// Постоянное вращение
// ====================================

void startContinuousRotation() {
  // Вращение к отрицательным координатам
  sendData(0xA0, RAMP_VEL_NEG);
  motorState = MOTOR_ROTATING;
  Serial.println("Continuous rotation started.");
}


// ====================================
// Движение к заданной позиции
// ====================================

void setTargetPosition(int32_t newTarget) {
  targetPosition = newTarget;

  sendData(0xAD, (uint32_t)targetPosition);
  sendData(0xA0, RAMP_POSITION);

  motorState = MOTOR_POSITIONING;

  positionCheckTime = millis();

  Serial.print("Moving to position: ");
  Serial.println(targetPosition);
}


// ====================================
// Проверка достижения позиции
// ====================================

void checkPositionReached() {
  if (motorState != MOTOR_POSITIONING) return;

  // Проверка каждые 50 мс
  if (millis() - positionCheckTime < 50) return;
  positionCheckTime = millis();

  // RAMP_STAT
  uint32_t rampStat = readData(REG_RAMP_STAT);

  // Bit 9: position_reached
  // Bit 10: vzero
  bool positionReached = (rampStat & (1UL << 9)) != 0;
  bool velocityZero = (rampStat & (1UL << 10)) != 0;

  // Дополнительно проверяем XACTUAL
  int32_t actualPosition = (int32_t)readData(REG_XACTUAL);

  if (positionReached && velocityZero && actualPosition == targetPosition) {
    motorState = MOTOR_WAITING;

    // RAMPMODE остаётся 0.
    // TMC5160 удерживает достигнутую позицию.

    Serial.print("Position reached: ");
    Serial.println(actualPosition);

    Serial.println("Motor stopped. Waiting for command.");
  }
}


// ====================================
// Неблокирующий приём Serial
// ====================================

void serialReceiveUserCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Команда постоянного вращения
    if (command.equalsIgnoreCase("R")) {
      startContinuousRotation();
    }
    // Команда целевой позиции
    else if (command.length() > 0) {
      setTargetPosition(command.toInt());
    }
  }
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
  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan1) != 0) {
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &TxHeader, can_data_send) != HAL_OK) {
      Error_Handler();
    } else {
      digitalToggle(LED1);
    }
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan1, FDCAN_RX_FIFO0) > 0) {
    FDCAN_RxHeaderTypeDef Header;
    uint8_t RxData[64] = {0};

    if (HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &Header, RxData) != HAL_OK) {
      Error_Handler();
    } else {
      Serial.print("ID ");
      Serial.print(Header.Identifier);
      Serial.print(" data: ");
      Serial.print(RxData[0]);
      Serial.print(" ");
      Serial.print(RxData[1]);
      Serial.print(" ");
      Serial.print(RxData[2]);
      Serial.print(" ");
      Serial.print(RxData[3]);
      Serial.println();
    }
  }
}


// ====================================
// Диагностика
// ====================================

void printDebugInfo() {
  static unsigned long print_t = 0;

  if (millis() - print_t < 200) return;
  print_t = millis();

  noInterrupts();
  int32_t ticks = encoderTicks;
  interrupts();

  float revolutions = (float)ticks / IPR;

  int32_t actualPosition = (int32_t)readData(REG_XACTUAL);

  Serial.print("Angle: ");
  Serial.print(as5047p.readAngleDegree());

  Serial.print(" | Ticks: ");
  Serial.print(ticks);

  Serial.print(" | Revolutions: ");
  Serial.print(revolutions, 4);

  Serial.print(" | XACTUAL: ");
  Serial.print(actualPosition);

  Serial.print(" | State: ");

  switch (motorState) {
    case MOTOR_ROTATING:
      Serial.println("ROTATING");
      break;

    case MOTOR_POSITIONING:
      Serial.println("POSITIONING");
      break;

    case MOTOR_WAITING:
      Serial.println("WAITING");
      break;
  }
}


// ====================================
// Setup
// ====================================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(LED1, OUTPUT);

  // Энкодер
  pinMode(Enc_B, INPUT);
  pinMode(Enc_A, INPUT);

  attachInterrupt(digitalPinToInterrupt(Enc_A), ISR_A, CHANGE);

  // CAN FD
  can_config(0x01);

  // TMC5160
  motor_config();

  // AS5047P на SPI3
  while (!as5047p.initSPI(&SPI_3)) {
    Serial.println(F("Can't connect to the AS5047P sensor!"));
    delay(3000);
  }

  Serial.println("AS5047P ready.");

  // Автоматически начинаем вращение
  startContinuousRotation();

  Serial.println();
  Serial.println("Commands:");
  Serial.println("  R       - continuous negative rotation");
  Serial.println("  -200000 - move to absolute position -200000");
  Serial.println("  50000   - move to absolute position 50000");
  Serial.println("  0       - move to position zero");
}


// ====================================
// Loop
// ====================================

void loop() {
  serialReceiveUserCommand();
  // Проверка достижения заданной позиции
  checkPositionReached();

  // CAN FD
  if (millis() - t >= 100) {
    can_send_recv();
    t = millis();
  }

  // Отладочная информация
  printDebugInfo();
}
