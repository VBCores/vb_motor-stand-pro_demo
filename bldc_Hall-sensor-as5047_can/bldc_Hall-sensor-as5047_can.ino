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
BLDCDriver3PWM driver = BLDCDriver3PWM(PA9, PA8, PA10);
// Hall sensor instance
// HallSensor(int hallA, int hallB , int cpr, int index)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
HallSensor sensor = HallSensor(PB6, PC7, PC8, 2);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

float target_voltage = 1;

uint8_t data[4] = { 170, 171, 172, 173}; //AA AB AC AD
unsigned long t = 0;
FDCAN_HandleTypeDef*  hfdcan1; // создаем переменную типа FDCAN_HandleTypeDef
CanFD* canfd;
FDCAN_TxHeaderTypeDef TxHeader; //FDCAN_TxHeaderTypeDef TxHeader;

void can_config(int ID){
  SystemClock_Config();  // Настройка тактирования
  canfd = new CanFD();  // Создаем управляющий класс
  canfd->init(); // Инициализация CAN
  canfd->write_default_params();  // Записываем дефолтные параметры для FDCAN (1000000 nominal / 8000000 data)
  canfd->apply_config();  // Применяем их
  hfdcan1 = canfd->get_hfdcan();  // Сохраняем конфиг
  canfd->default_start();
 

  TxHeader.Identifier = ID; 
  TxHeader.DataLength = FDCAN_DLC_BYTES_4;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  delay(10);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED2, OUTPUT);

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

/*----------Инициализация AS5047---------*/
  while (!as5047p.initSPI(& SPI_3)) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(3000);
  }

/*-----Simple FOC init config ------*/
  // check if you need internal pullups
 // sensor.pullup = Pullup::USE_EXTERN;
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);

  Serial.println("Sensor ready");

  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);
  motor.current_limit = 1;
  motor.init();
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.initFOC();

  Serial.println(F("\n Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
  _delay(1000);
}

void can_send_recv(){
//------Отправка сообщения в can------
  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan1) != 0){
      
      if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &TxHeader, data) != HAL_OK){ Error_Handler(); } 
      else{digitalWrite(LED2, !digitalRead(LED2));} //помигаем светодиодом, если все ок
    }

    // /*-------Получение сообщений из can-------*/ 

    while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan1, FDCAN_RX_FIFO0) > 0 )
      {
        FDCAN_RxHeaderTypeDef Header;  // хидер для входящего сообщения
        uint8_t RxData[4]; // длина входящего сообщения - 4 байта, вообще максимальная длина сообщения - 64 байта 
        if (HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &Header, RxData) != HAL_OK){ Error_Handler(); }  
        else{ // напечатаем первые 4 байта входящего сообщения, если все ок. Пример отправки сообщения cansend can0 00000123#DEADBEEF 
        Serial.print("ID ");
        Serial.print(Header.Identifier); // ID сообщения 
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
    t = millis();
  }

  sensor.update();
  Serial.print("Hall Angle: ");
  Serial.print(sensor.getAngle());
  Serial.print(" SPI sensor angle: ");
  Serial.print(as5047p.readAngleDegree());
  Serial.print(" Velocity from Hall sensor: ");
  Serial.println(sensor.getVelocity());

  motor.loopFOC();
  motor.move(target_voltage);
  serialReceiveUserCommand();

 
}

void serialReceiveUserCommand() {

 
  static String received_chars;

  while (Serial.available()) {

    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n') {

      target_voltage = received_chars.toFloat();
      Serial.print("Target voltage: ");
      Serial.println(target_voltage);

      received_chars = "";
    }
  }
}
