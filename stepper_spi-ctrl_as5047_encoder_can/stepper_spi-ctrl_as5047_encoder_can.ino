/*Мотор вращается 2 сек в одну сторону, 2 сек в другую
57HS100-4204A-E1000  I=4.2A Angular step 1.8° Phase number: 2
encoder-wires: PB6:EA+  PC7:EB+  PA0:EA- (not used)  NC:EB- (not used)
*/ 
#include <SPI.h>
#include <VBCoreG4_arduino_system.h>
#include <AS5047P.h>

#define AS5047P_CHIP_SELECT_PORT PA_15_ALT1
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

#define Enc_A PB6  
#define Enc_B PC7

#define SD_MODE          PA12 // Mode select
#define SPI_MODE         PA11 // Mode select

#define ENCB_DCEN_CFG4   PB0 // Encoder or CFG4
#define ENCA_DCIN_CFG5   PB1 // Encoder or CFG4
#define ENCN_DCO_CFG6    PB0 // Encoder or CFG4


#define EN_PIN           PC5  // Enable  (LOW = Driver is active)
#define DIR_PIN          PA9 // Direction
#define STEP_PIN         PA8  // Step
// #define SS           PA4 // Chip select
// #define MOSI          PA7 // Software Master Out Slave In (MOSI)
// #define MISO          PA6 // Software Master In Slave Out (MISO)
// #define SCK           PA5 // Software Slave Clock (SCK)

#define SPI1_NSS_PIN PA4

byte data[5];

//           MOSI  MISO  SCLK
SPIClass SPI_3(PC12, PC11, PC10); 
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

int ipr = 2000;
int count = 0;
int rotations = 0;
float start_angle = 0;


uint8_t can_data_send[4] = { 170, 171, 172, 173}; //AA AB AC AD
unsigned long t = 0;
FDCAN_HandleTypeDef*  hfdcan1; // создаем переменную типа FDCAN_HandleTypeDef
CanFD* canfd;
FDCAN_TxHeaderTypeDef TxHeader; //FDCAN_TxHeaderTypeDef TxHeader;

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

void motor_config(){
  //SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
  SPI.setBitOrder(MSBFIRST); 
  SPI.setDataMode(SPI_MODE3); 
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  pinMode(SPI1_NSS_PIN, OUTPUT);

  SPI.begin();

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(SD_MODE, OUTPUT);
  pinMode(SPI_MODE, OUTPUT);
 
  digitalWrite(SD_MODE, LOW);
  digitalWrite(SPI_MODE, HIGH);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(SPI1_NSS_PIN, HIGH);

  sendData(0x80,0x00000000);      //GCONF

  sendData(0xEC,0x000101D5);      //CHOPCONF: TOFF=5, HSTRT=5, HEND=3, TBL=2, CHM=0 (spreadcycle)
  sendData(0x90,0x00070603);      //IHOLD_IRUN: IHOLD=3, IRUN=10 (max.current), IHOLDDELAY=6
  sendData(0x91,0x0000000A);      //TPOWERDOWN=10

  sendData(0xF0,0x00000000);      // PWMCONF
  //sendData(0xF0,0x000401C8);      //PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amp limit=200, grad=1

  sendData(0xA4,0x000003E8);     //A1=1000
  sendData(0xA5,0x000186A0);     //V1=100000
  sendData(0xA6,0x0000C350);     //AMAX=50000
  sendData(0xA7,0x000186A0);     //VMAX=100000
  sendData(0xAA,0x00000578);     //D1=1400
  sendData(0xAB,0x0000000A);     //VSTOP=10

  sendData(0xA0,0x00000000);     //RAMPMODE=0

  sendData(0xA1,0x00000000);     //XACTUAL=0
  sendData(0xAD,0x00000000);     //XTARGET=0
}

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
void sendData(unsigned long address, unsigned long datagram) {
  //TMC5160 takes 40 bit data: 8 address and 32 data

  delay(100);
  unsigned long i_datagram;

  digitalWrite(SPI1_NSS_PIN, LOW);
  delayMicroseconds(10);

  SPI.transfer(address);

  i_datagram |= SPI.transfer((datagram >> 24) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 16) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 8) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram) & 0xff);
  digitalWrite(SPI1_NSS_PIN ,HIGH);

  Serial.print("Received: ");
  Serial.println(i_datagram, HEX);
  Serial.print(" from register: ");
  Serial.println(address,HEX);
}

void setup(){
  Serial.begin(115200);
  pinMode(LED2, OUTPUT);

  pinMode(Enc_B, INPUT);
  pinMode(Enc_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(Enc_A), ISR_A, CHANGE);
  
  can_config(0x01);
  motor_config();

  // /*--- Инициализация датчика ---*/ 
  while (!as5047p.initSPI(& SPI_3)) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(3000);
  }
  start_angle =as5047p.readAngleDegree(); 
  
}

void can_send_recv(){
//------Отправка сообщения в can------
  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan1) != 0){
      
      if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &TxHeader, can_data_send) != HAL_OK){ Error_Handler(); } 
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

int target = 51200;

void loop(){
  sendData(0xAD,target);
  delay(1);
  if (Serial.available() > 0) {
    target = Serial.readString().toInt();    
  }

  if (millis() - t >= 100){
    can_send_recv();
    t = millis();
  }

  Serial.print("Angle: ");
  Serial.print(as5047p.readAngleDegree());
  Serial.print(" Counts: ");
  Serial.print(count);
  Serial.print(" Rotation: ");
  Serial.println(rotations);

  target +=51200;
  delay(100);

  
}

