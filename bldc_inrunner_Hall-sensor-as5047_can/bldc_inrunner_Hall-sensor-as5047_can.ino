/* BLDC - 57BL95S15-230 24V, 150W, I = 8.7A, 0.5N*m Pole pairs = 2 */

#include <VBCoreG4_arduino_system.h>
#include <SimpleFOC.h>

#define AS5047P_CHIP_SELECT_PORT PA_15_ALT1
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

//            MOSI  MISO  SCLK
SPIClass SPI_3(PC12, PC11, PC10); 

BLDCMotor motor = BLDCMotor(2);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10);

// AS5047 по SPI3
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047P_CHIP_SELECT_PORT, 14, 0x3FFF);

float target_voltage = 2;


uint8_t data[4] = { 170, 171, 172, 173}; //AA AB AC AD
unsigned long t = 0;
FDCAN_HandleTypeDef*  hfdcan1;
CanFD* canfd;
FDCAN_TxHeaderTypeDef TxHeader;

void can_config(int ID){
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

  /*----------Инициализация AS5047 для SimpleFOC---------*/
  sensor.init(&SPI_3);

  Serial.println("AS5047 sensor ready");

  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 24;
  driver.init();

  motor.linkDriver(&driver);
  motor.current_limit = 8.7;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit = 24;
  motor.init();
  motor.initFOC();

  Serial.println(F("\n Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
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
  if (millis() - t >= 100){
    can_send_recv();
    motor.move(target_voltage);
    Serial.println(target_voltage);
    serialReceiveUserCommand();
    t = millis();
  }

  sensor.update();
  motor.loopFOC();
  
  
}

void serialReceiveUserCommand() {
  static String received_chars;

  while (Serial.available()) {
    char inChar = (char)Serial.read();
    received_chars += inChar;

    if (inChar == '\n') {
      target_voltage = received_chars.toFloat();
      received_chars = "";
    }
  }
}