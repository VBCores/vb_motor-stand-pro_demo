/* BLDC - 57BL95S15-230 24V, 150W, I = 8.7A, 0.5N*m Pole pairs = 2 */

#include <VBCoreG4_arduino_system.h>
#include <SimpleFOC.h>
#include <AS5047P.h>

#define AS5047P_CHIP_SELECT_PORT PA_15_ALT1
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

//            MOSI  MISO  SCLK
SPIClass SPI_3(PC12, PC11, PC10);
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

BLDCMotor motor = BLDCMotor(2);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10);

// Hall sensors
HallSensor sensor = HallSensor( PB6, PC7, PC8, 2);

void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

float target_voltage = 2.0;

uint8_t data[4] = {170, 171, 172, 173}; // AA AB AC AD
unsigned long t = 0;

FDCAN_HandleTypeDef* hfdcan1;
CanFD* canfd;
FDCAN_TxHeaderTypeDef TxHeader;

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

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PD2, OUTPUT);

  pinMode(PB5, INPUT);
  pinMode(PB3, OUTPUT);

  pinMode(PB15, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB13, OUTPUT);

  digitalWrite(PB15, HIGH);
  digitalWrite(PB14, HIGH);
  digitalWrite(PB13, HIGH);

  digitalWrite(PB3, HIGH);

  can_config(0x04);

  /* ---------- Инициализация AS5047 ---------- */
  while (!as5047p.initSPI(&SPI_3)) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(3000);
  }

  Serial.println("AS5047P ready");

  /* ---------- Инициализация Hall sensor ---------- */
  // Если внешних подтяжек нет, можно попробовать:
  // sensor.pullup = Pullup::USE_INTERN;

  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);

  Serial.println("Hall sensor ready");

  /* ---------- SimpleFOC config ---------- */
  motor.linkSensor(&sensor);

  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 24;
  driver.voltage_limit = 18;
  driver.init();

  motor.linkDriver(&driver);

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  motor.voltage_limit = 24;
  motor.current_limit = 3;

  // Для Hall-сенсоров часто лучше, чем SinePWM/SpaceVectorPWM
 motor.foc_modulation = FOCModulationType::SinePWM;
  

  motor.init();

  Serial.println("Starting FOC alignment...");
  motor.initFOC();

  Serial.println(F("\nMotor ready."));
  Serial.println(F("Send target voltage through Serial, for example: 3 or -3"));
}

void can_send_recv() {
  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan1) != 0) {
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &TxHeader, data) != HAL_OK) {
      Error_Handler();
    } else {
      digitalToggle(PD2);
    }
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan1, FDCAN_RX_FIFO0) > 0) {
    FDCAN_RxHeaderTypeDef Header;
    uint8_t RxData[4];

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

void clearDrvFault() {
    motor.disable();
    digitalWrite(PB3, LOW);
    delayMicroseconds(1);   // 1-1.2 мкс по даташиту
    digitalWrite(PB3, HIGH);
    delay(10);              // ждём wake-up (typ 1-2 мс)
    motor.enable();
    Serial.println("DRV8328B fault cleared");
}

void loop() {
  sensor.update();
  motor.loopFOC();
  motor.move(target_voltage);

  serialReceiveUserCommand();

  if (millis() - t >= 100) {
    
    can_send_recv();
    t = millis();
  }

  static unsigned long print_t = 0;
  if (millis() - print_t >= 200) {
    Serial.print("Target voltage: ");
    Serial.print(target_voltage);
    Serial.print(" | Hall angle: ");
    Serial.print(sensor.getAngle());
    Serial.print(" | SPI angle: ");
    Serial.print(as5047p.readAngleDegree());
    Serial.print(" | NFAULT: ");
    Serial.println(digitalRead(PB5));
     Serial.print(" | Hall velocity: ");
    Serial.println(sensor.getVelocity());

    print_t = millis();
  }

  if (digitalRead(PB5) == LOW) {
    clearDrvFault();
  }
}

void serialReceiveUserCommand() {
  static String received_chars;

  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      if (received_chars.length() > 0) {
        target_voltage = received_chars.toFloat();
        if (target_voltage > 10.0)
          target_voltage = 10.0;
        else if (target_voltage < -10.0)
          target_voltage = -10.0;

        Serial.print("New target voltage: ");
        Serial.println(target_voltage);

        received_chars = "";
      }
    } else {
      received_chars += inChar;
    }
  }
}