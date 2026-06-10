/* BLDC - 57BL95S15-230 24V, 150W, I = 8.7A, 0.5N*m Pole pairs = 2 */

#include <VBCoreG4_arduino_system.h>
#include <SimpleFOC.h>

#define AS5047P_CHIP_SELECT_PORT PA_15_ALT1
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

//            MOSI  MISO  SCLK
<<<<<<< HEAD
SPIClass SPI_3(PC12, PC11, PC10);
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
=======
SPIClass SPI_3(PC12, PC11, PC10); 
>>>>>>> 73e46cede4de9acec182d1e92e22107205bcf9f9

BLDCMotor motor = BLDCMotor(2);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10);

<<<<<<< HEAD
// Hall sensors
HallSensor sensor = HallSensor( PB6, PC7, PC8, 2);
=======
// AS5047 по SPI3
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047P_CHIP_SELECT_PORT, 14, 0x3FFF);

static const float FIXED_ZERO_ELECTRIC_ANGLE = 2.132524f;
>>>>>>> 73e46cede4de9acec182d1e92e22107205bcf9f9

void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

<<<<<<< HEAD
float target_voltage = 2.0;

uint8_t data[4] = {170, 171, 172, 173}; // AA AB AC AD
unsigned long t = 0;

FDCAN_HandleTypeDef* hfdcan1;
CanFD* canfd;
FDCAN_TxHeaderTypeDef TxHeader;

void can_config(int ID) {
  SystemClock_Config();

=======

uint8_t data[4] = { 170, 171, 172, 173}; //AA AB AC AD
unsigned long t = 0;
unsigned long telemetry_t = 0;
FDCAN_HandleTypeDef*  hfdcan1;
CanFD* canfd;
FDCAN_TxHeaderTypeDef TxHeader;

static float normalizeAngle(float angle){
  while (angle < 0.0f) {
    angle += _2PI;
  }
  while (angle >= _2PI) {
    angle -= _2PI;
  }
  return angle;
}



void runInitFOC(){
  target_voltage = 0.0f;
  motor.move(0.0f);
  delay(200);
  Serial.println(F("Running SimpleFOC initFOC..."));
  motor.initFOC();
  Serial.print(F("SimpleFOC found zero electric angle: "));
  Serial.println(motor.zero_electric_angle, 6);
  motor.zero_electric_angle = normalizeAngle(FIXED_ZERO_ELECTRIC_ANGLE);
  Serial.print(F("Fixed zero electric angle: "));
  Serial.println(motor.zero_electric_angle, 6);
  Serial.print(F("Sensor direction: "));
  Serial.println(motor.sensor_direction == Direction::CW ? F("CW") : F("CCW"));
}

void can_config(int ID){
  SystemClock_Config();
>>>>>>> 73e46cede4de9acec182d1e92e22107205bcf9f9
  canfd = new CanFD();
  canfd->init();
  canfd->write_default_params();
  canfd->apply_config();
<<<<<<< HEAD

=======
>>>>>>> 73e46cede4de9acec182d1e92e22107205bcf9f9
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

<<<<<<< HEAD
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
=======
  /*----------Инициализация AS5047 для SimpleFOC---------*/
  sensor.init(&SPI_3);

  Serial.println("AS5047 sensor ready");
>>>>>>> 73e46cede4de9acec182d1e92e22107205bcf9f9

  /* ---------- SimpleFOC config ---------- */
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 24;
<<<<<<< HEAD
  driver.voltage_limit = 12;
  driver.init();

  motor.linkDriver(&driver);

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  motor.voltage_limit = 12;
  motor.current_limit = 6;

  // Для Hall-сенсоров часто лучше, чем SinePWM/SpaceVectorPWM
 motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  

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

void loop() {
  sensor.update();
  motor.loopFOC();

  serialReceiveUserCommand();

  if (millis() - t >= 100) {
    motor.move(target_voltage);
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
=======
  driver.pwm_frequency = 25000;
  driver.init();

  motor.linkDriver(&driver);
  motor.current_limit = 8.7;
  motor.voltage_sensor_align = 2.0;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit = 24;
  motor.init();
  runInitFOC();

  Serial.println(F("\n Motor ready."));
  Serial.println(F("Set target voltage with a number."));
  Serial.println(F("Commands: Z angle set zero, O delta add zero."));
  _delay(1000);
}

void can_send_recv(){
  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan1) != 0){
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &TxHeader, data) != HAL_OK){ 
      Error_Handler(); 
    } 
    else{
      digitalToggle(PD2);
    }
  }

  while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan1, FDCAN_RX_FIFO0) > 0 )
  {
    FDCAN_RxHeaderTypeDef Header;
    uint8_t RxData[4];

    if (HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &Header, RxData) != HAL_OK){ 
      Error_Handler(); 
    }  
    else{
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
      Serial.println("  ");
    }
  }
}

void loop() {
  sensor.update();
  motor.loopFOC();
  motor.move(target_voltage);

  if (millis() - t >= 100){
    can_send_recv();
    serialReceiveUserCommand();
    t = millis();
>>>>>>> 73e46cede4de9acec182d1e92e22107205bcf9f9
  }
}

void serialReceiveUserCommand() {
  static String received_chars;

  while (Serial.available()) {
    char inChar = (char)Serial.read();
<<<<<<< HEAD

    if (inChar == '\n' || inChar == '\r') {
      if (received_chars.length() > 0) {
        target_voltage = received_chars.toFloat();

        Serial.print("New target voltage: ");
        Serial.println(target_voltage);

        received_chars = "";
      }
    } else {
      received_chars += inChar;
=======
    received_chars += inChar;

    if (inChar == '\n') {
      received_chars.trim();
      char command = toupper(received_chars.charAt(0));
      if (command == 'Z') {
        motor.zero_electric_angle = normalizeAngle(received_chars.substring(1).toFloat());
        Serial.print(F("Zero electric angle set: "));
        Serial.println(motor.zero_electric_angle, 6);
      } else if (command == 'O') {
        motor.zero_electric_angle = normalizeAngle(motor.zero_electric_angle + received_chars.substring(1).toFloat());
        Serial.print(F("Zero electric angle adjusted: "));
        Serial.println(motor.zero_electric_angle, 6);
      } else {
        target_voltage = received_chars.toFloat();
      }
      received_chars = "";
>>>>>>> 73e46cede4de9acec182d1e92e22107205bcf9f9
    }
  }
}