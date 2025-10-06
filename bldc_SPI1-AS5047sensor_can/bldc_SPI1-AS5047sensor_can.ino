/*GM 5208-12, датчик AS5047 к SPI1 */
#include <VBCoreG4_arduino_system.h> 
#include <SimpleFOC.h>


BLDCMotor motor = BLDCMotor(1);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA9, PA8, PA10);

SPIClass SPI_1(PA7, PA6, PA5); //NSS PA4
MagneticSensorSPI sensor = MagneticSensorSPI(PA4, 14, 0x3FFF);

uint8_t data[4] = { 170, 171, 172, 173}; //AA AB AC AD
unsigned long t;
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

  /* -----Настройка FD CAN  */
  can_config(0x03);

  /*----Simple FOC init config-----*/
 
  sensor.init(&SPI_1);
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);
  motor.init();

  Serial.println("Pole pairs (PP) estimator");
  Serial.println("-\n");

  float pp_search_voltage = 4; // maximum power_supply_voltage/2
  float pp_search_angle = 6*_PI; // search electrical angle to turn

  motor.controller = MotionControlType::angle_openloop;
  motor.voltage_limit=pp_search_voltage;
  motor.move(0);
  _delay(1000);
  sensor.update();
  float angle_begin = sensor.getAngle();
  _delay(50);

  float motor_angle = 0;
  while(motor_angle <= pp_search_angle){
    motor_angle += 0.01f;
    sensor.update(); // keep track of the overflow
    motor.move(motor_angle);
    _delay(1);
  }
  _delay(1000);
  sensor.update(); 
  float angle_end = sensor.getAngle();
  _delay(50);
  motor.move(0);
  _delay(1000);

  int pp = round((pp_search_angle)/(angle_end-angle_begin));

  Serial.print(F("Estimated PP : "));
  Serial.println(pp);
  Serial.println(F("PP = Electrical angle / Encoder angle "));
  Serial.print(pp_search_angle*180/_PI);
  Serial.print(F("/"));
  Serial.print((angle_end-angle_begin)*180/_PI);
  Serial.print(F(" = "));
  Serial.println((pp_search_angle)/(angle_end-angle_begin));
  Serial.println();


  if(pp <= 0 ){
    Serial.println(F("PP number cannot be negative"));
    Serial.println(F(" - Try changing the search_voltage value or motor/sensor configuration."));
    return;
  }else if(pp > 30){
    Serial.println(F("PP number very high, possible error."));
  }else{
    Serial.println(F("If PP is estimated well your motor should turn now!"));
    Serial.println(F(" - If it is not moving try to relaunch the program!"));
    Serial.println(F(" - You can also try to adjust the target voltage using serial terminal!"));
  }


  motor.controller = MotionControlType::torque;
  motor.pole_pairs = pp;
  motor.initFOC();
  _delay(1000);

  t = 0;
  Serial.println(F("\n Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
}

float target_voltage = 2;

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
  motor.loopFOC();
  motor.move(target_voltage);
  serialReceiveUserCommand();
  delay(1);
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
