/*DC motor 24V
CHP-36GP-555-ABHL
Gear ratio 1:51
ipr = 1734*/
#include <VBCoreG4_arduino_system.h>
#include <AS5047P.h>

#define AS5047P_CHIP_SELECT_PORT PA_15_ALT1
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

#define Enc_A PB6  
#define Enc_B PC7

#define IN1 PA8
#define IN2 PA9
#define SLEEPn PB3
#define VrefPin PA4
#define USR_BTN PC13


//            MOSI  MISO  SCLK
SPIClass SPI_3(PC12, PC11, PC10); 
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

int pwm = 500;
int count = 0;
float start_angle = 0;
int rotations = 0;
int ipr = 1734;

void ISR_A(){
  
  if (digitalRead(Enc_A) == digitalRead(Enc_B)){
    count += 1;
  }
  else {
    count -= 1;
  }

  if (count <= -ipr){ 
    rotations -= 1;
    count = 0;
  }
  else if (count >= ipr){
    rotations += 1;
    count = 0;
  }
  

}

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

  pinMode(Enc_B, INPUT);
  pinMode(Enc_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(Enc_A), ISR_A, CHANGE);

  pinMode(PB3, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(VrefPin, OUTPUT);
  pinMode(USR_BTN, INPUT_PULLUP);
 
  digitalWrite(SLEEPn, HIGH);
  analogWriteResolution(12);
  analogWrite(VrefPin, 2000);
  analogWriteFrequency(25000);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  /*--- Инициализация датчика ---*/ 
  while (!as5047p.initSPI(& SPI_3)) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(3000);
  }
  start_angle =as5047p.readAngleDegree(); 
  
  can_config(0x02);

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

//-----move motor----
  if (pwm>=0) {
      analogWrite(IN1, 4096);
      analogWrite(IN2, 4096-pwm);
    }
  else {
    analogWrite(IN2, 4096);
    analogWrite(IN1, 4096-abs(pwm)); 
  }
  /* ---------Подсчет ipr---------- */
  // if (pwm!=0 && abs(start_angle - as5047p.readAngleDegree())<0.05){
  //   ++rotations;
  //   Serial.print(as5047p.readAngleDegree());
  //   Serial.print(" - angle, start_angle: ");
  //   Serial.print(start_angle);
  //   Serial.print(" rotations: ");
  //   Serial.print(rotations);
  //   Serial.print(" impulse: ");
  //   Serial.println(count);
  // }
  if (Serial.available() > 0) {
    pwm = Serial.readString().toInt();    
  }

  if (millis() - t >= 100){
    can_send_recv();
    t = millis();
  }

  /*---- print data ----*/ 

  Serial.print(as5047p.readAngleDegree());
  Serial.print(" - angle, rotations: ");
  Serial.println(rotations);
  delay(100); 
  
}

  
