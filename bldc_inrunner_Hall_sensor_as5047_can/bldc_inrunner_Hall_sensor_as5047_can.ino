
/* BLDC - 57BL115S21-230TF9 24V, 210W, I = 13A, 0.7N*m Pole pairs = 2 */
/*Для управления двигателем используется SimpleFOC версии 2.3.4 и stm32 mcu based boards версии 2.12.0*/

#include <VBCoreG4_arduino_system.h>
#include <SimpleFOC.h>
#include <AS5047P.h>

#define AS5047P_CHIP_SELECT_PORT PA_15_ALT1
#define AS5047P_CUSTOM_SPI_BUS_SPEED 1000000

#define NSLEEP PB3
#define NFAULT PB5
#define LED1 PD2

//            MOSI  MISO  SCLK
SPIClass SPI_3(PC12, PC11, PC10);
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

BLDCMotor motor = BLDCMotor(2);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PB13, PB14, PB15);

// Current sensors ACS711, 45 mV/A
//                                                           phA  phB  phC
InlineCurrentSense current_sense = InlineCurrentSense(45.0f, PC1, PC2, PC3);

// Hall sensors
HallSensor sensor = HallSensor(PB6, PC7, PC8, 2);

void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

float target_current = 0.6;

uint8_t data[4] = {170, 171, 172, 173}; // AA AB AC AD
unsigned long t = 0;

FDCAN_HandleTypeDef* hfdcan1;
CanFD* canfd;
FDCAN_TxHeaderTypeDef TxHeader;

// Таймер, по которому будет вызываться loopFOC()
HardwareTimer *timer = new HardwareTimer(TIM7);

//Функция настройки can
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


void FOC_func() {
  motor.loopFOC();
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED1, OUTPUT);

  pinMode(NFAULT, INPUT_PULLUP);
  pinMode(NSLEEP, OUTPUT);

  digitalWrite(NSLEEP, HIGH);

  can_config(0x04);

  /* ---------- Инициализация AS5047 ---------- */
  while (!as5047p.initSPI(&SPI_3)) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(3000);
  }
  Serial.println("AS5047P ready");

  /* ---------- Инициализация Hall sensor ---------- */
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  Serial.println("Hall sensor ready");

  /* ---------- SimpleFOC config ---------- */

  motor.linkSensor(&sensor);

  driver.pwm_frequency = 25000;
  driver.voltage_power_supply = 24.0f;
  driver.voltage_limit = 24.0f;

  if (!driver.init()) {
    digitalWrite(NSLEEP, LOW);
    Serial.println("Driver init failed!");
    while (true) delay(1000);
  }

  motor.linkDriver(&driver);

  // Настраиваем двигатель до motor.init()
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.voltage_limit = 3.0f;
  motor.voltage_sensor_align = 1.0f;

  // Начальные коэффициенты PID_current для последующей настройки
  motor.PID_current_q.P = 0.1f;
  motor.PID_current_q.I = 20.0f;
  motor.PID_current_q.D = 0.0f;

  motor.PID_current_d.P = 0.1f;
  motor.PID_current_d.I = 20.0f;
  motor.PID_current_d.D = 0.0f;

  motor.LPF_current_q.Tf = 0.005f;
  motor.LPF_current_d.Tf = 0.005f;

  analogReadResolution(10);

  // Связываем датчики тока с драйвером и мотором
  current_sense.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);

  Serial.println("Starting motor initialization...");

  if (!motor.init()) {
    motor.disable();
    digitalWrite(NSLEEP, LOW);
    Serial.println("Motor init failed!");
    while (true) delay(1000);
  }

  if (!current_sense.init()) {
    motor.disable();
    digitalWrite(NSLEEP, LOW);
    Serial.println("Current sense init failed!");
    while (true) delay(1000);
  }

  Serial.println("Starting FOC alignment...");

  if (!motor.initFOC()) {
    motor.disable();
    digitalWrite(NSLEEP, LOW);
    Serial.println("FOC init failed!");
    while (true) delay(1000);
  }

  Serial.println("FOC alignment successful!");

  digitalWrite(NSLEEP, HIGH);
  delay(10);

  if (digitalRead(NFAULT) == LOW) {
    Serial.println("DRV8328B fault!");
    motor.disable();
    digitalWrite(NSLEEP, LOW);
    while (true) delay(1000);
  }

  motor.enable();

  //motor.loopFOC() будет вызываться с частотой 1.5 кГц
  timer->pause();
  timer->setOverflow(1500, HERTZ_FORMAT);
  timer->attachInterrupt(FOC_func);
  timer->refresh();
  timer->resume();

  Serial.println("Current control enabled. Input target in A");
}


void can_send_recv() {
  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan1) != 0) {
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &TxHeader, data) != HAL_OK) {
      Error_Handler();
    } else {
      digitalToggle(LED1);
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


/* ---------- Debug information ---------- */

void printDebugInfo() {
  static unsigned long print_t = 0;

  if (millis() - print_t < 200) return;
  print_t = millis();

  Serial.print("Target Iq: ");
  Serial.print(target_current, 3);

  Serial.print(" | Current SP: ");
  Serial.print(motor.current_sp, 3);

  Serial.print(" | Iq: ");
  Serial.print(motor.current.q, 3);

  Serial.print(" | Id: ");
  Serial.print(motor.current.d, 3);

  Serial.print(" | Uq: ");
  Serial.print(motor.voltage.q, 3);

  Serial.print(" | Ud: ");
  Serial.print(motor.voltage.d, 3);

  Serial.print(" | SPI angle: ");
  Serial.print(as5047p.readAngleDegree(), 2);

  Serial.print(" | Hall velocity: ");
  Serial.println(sensor.getVelocity(), 2);

}



/* ---------- Main loop ---------- */

void loop() {
  sensor.update();
  motor.move(target_current);

  serialReceiveUserCommand();

  if (millis() - t >= 100) {
    can_send_recv();
    t = millis();
  }

  printDebugInfo();
}


/* ---------- Serial commands ---------- */

void serialReceiveUserCommand() {
  static String received_chars;

  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      if (received_chars.length() > 0) {
        target_current = received_chars.toFloat();
        received_chars = "";
      }
    } else {
      received_chars += inChar;
    }
  }
}
