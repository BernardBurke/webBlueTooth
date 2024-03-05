#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>
#include <Adafruit_SSD1327.h>
#include <Adafruit_MLX90614.h>
#include <stdio.h> 
#include "driver/rtc_io.h"
#include "string.h"

//**************************** System Settings **********************************
#define DEVICE_NAME         "Examin_Breathalyzer_1"
#define verbose             1

//****************************** Flow Settings **********************************
#define THRESHOLD             400
#define BREATH_LENGTH         5000000 //How long the breath timer should be
#define PERIOD                20000
#define BREATH_SAMPLES        5  //How many samples are taken for each measurement
#define BREATH_PRESSURE       60 //Minimum pressure for the breath to be accepted
#define BREATH_SENSITIVITY    100
#define BREATH_END_THRESHOLD  4090 //What the capacitance has to drop to for the breath to be considered done (in pF)
#define BREATH_DELAY_1        5000000 //Time until first measurement after breath (in microseconds)
#define BREATH_DELAY_2        10000000    //Time after first measurement until second
#define BREATH_DELAY_3        30000000
#define SLEEP_TIME            600000000 //Time until the device falls asleep
#define SHUTDOWN_VOLTAGE      3000 //The battery voltage when the system will shutdown in mV

//**************************** Hardware Parameters ******************************
#define BIO_RESET_PIN       17   
#define BIO_MFIO_PIN        18

#define OLED_ADDR             0x3C
#define OLED_RESET_PIN      9
#define ENABLE_12V_PIN      12 

#define CAP_ADDR              0x48

#define MLX_ADDR            0x5A

#define PRESS_ADDR          0x5D

#define BUTTON_PIN          6
#define BUTTON_PIN_BITMASK  0x40

#define I2C1_SCL_PIN        5
#define I2C1_SDA_PIN        4

#define I2C2_SCL_PIN        38                
#define I2C2_SDA_PIN        21  

#define GREEN_LED_PIN       2
#define RED_LED_PIN         1

#define EEPROM_ADDR         0x50


//****************************** System States **********************************
#define IDLE                0
#define INITILIZE           1
#define WAIT_FOR_BREATH     2
#define BREATH              3
#define WAIT_AFTER_BREATH_1 4
#define MEASURE_1           5
#define WAIT_AFTER_BREATH_2 6  
#define MEASURE_2           7
#define WAIT_AFTER_BREATH_3 8
#define MEASURE_3           9
#define COMPLETE            10
#define MOUTHPIECE_MISSING  11
#define SHUTDOWN            12
#define LOW_VOLTAGE         13

//****************************** Battery Commands *******************************
#define ADR_GG              (0xAA >> 1)


#define GG_NVM_DATA(offset) (GG_CEXT_BLOCKDAT + offset)

/*** I2C Word Sizes ***/
#define WORD_SIZE_GG    (2)

/*** Standard Commands ***/
#define GG_CMD_CNTL         (0x00)
#define GG_CMD_TEMP         (0x02)
#define GG_CMD_VOLT         (0x04)
#define GG_CMD_FLAG         (0x06)
#define GG_CMD_NAC          (0x08)
#define GG_CMD_FAC          (0x0A)
#define GG_CMD_RM           (0x0C)
#define GG_CMD_FCC          (0x0E)
#define GG_CMD_AVGI         (0x10)
#define GG_CMD_AVGP         (0x18)
#define GG_CMD_SOC          (0x1C)
#define GG_CMD_ITEMP        (0x1E)
#define GG_CMD_SOH          (0x20)
#define GG_CMD_OPCFG        (0x3A)
#define GG_CMD_DCAP         (0x3C)

/*** Standard Commands ***/
#define GG_CEXT_DATCLASS    (0x3E)
#define GG_CEXT_DATBLOCK    (0x3F)
#define GG_CEXT_BLOCKDAT    (0x40)
#define GG_CEXT_BLDATCHKS   (0x60)
#define GG_CEXT_BLDATCTRL   (0x61)

/*** Control Subcommands ***/
#define GG_CTL_BATINSERT    (0x000C)
#define GG_CTL_SETCFGUPD    (0x0013)
#define GG_CTL_SOFTRESET    (0x0042)

/*** Subclasses ***/
#define GG_CLASS_STATE      (82)

/*** Data Offsets ***/
#define GG_OFS_OPCFG        (5)
#define GG_OFS_DESCAP       (12)
#define GG_OFS_DESENRGY     (14)

/*** Register Bit Positions ***/
#define BATLOWEN            (2)

//*************************** Bluetooth Characteristics *************************
#define SERVICE_UUID                "ba65b745-f9f7-4873-9289-bb93564909f3"

#define BODY_CHARACTERISTIC_UUID      "f87ef99e-a101-4518-958b-4def53dc4b8e" // HR, Oxygen
#define TMP_CHARACTERISTIC_UUID     "38bca6e3-05a4-487d-95b0-351a3e25eae1"   // temp, Hand Temp
#define MEAS_CHARACTERISTIC_UUID    "10762b73-87ad-4368-b9b9-a2ab27cfaad7"    // vir_init, vir_1, vir_2, vir3
#define PRES_CHARACTERISTIC_UUID      "2f63d787-50dc-42e6-ace0-a403958a02b6" // pressure
#define VS_CHARACTERISTIC_UUID      "ba65b745-f9f7-4873-9289-bb93564909f3"    // raw Virus
#define STAT_CHARACTERISTIC_UUID    "3e9c01e6-12de-11eb-adc1-0242ac120002"    // state
#define ID_CHARACTERISTIC_UUID    "7946624e-155d-11ee-be56-0242ac120002"      // id

BLECharacteristic *pCharacteristicVS;
BLECharacteristic *pCharacteristicBODY;
BLECharacteristic *pCharacteristicTMP;
BLECharacteristic *pCharacteristicID;
BLECharacteristic *pCharacteristicSTAT;
BLECharacteristic *pCharacteristicPRES;
BLECharacteristic *pCharacteristicMEAS;


//****************************** Screen Bitmaps *********************************
static const unsigned char PROGMEM ble_bitmap[] = {
  0b00011000,  0b00010100,
  0b00010010,  0b10010001,
  0b01010010,  0b00110100,
  0b00011000,  0b00011000,
  0b00110100,  0b01010010,
  0b10010001,  0b00010010,
  0b00010100,  0b00011000,
  0b00000000,  0b00000000
  };

static const unsigned char PROGMEM u_bitmap[] = {
  0b11111110, 0b01111111,
  0b11111110, 0b01111111,
  0b11111110, 0b01111111,
  0b11111110, 0b01111111,
  0b00111100, 0b00111100,
  0b00111100, 0b00111100,
  0b00111100, 0b00111100,
  0b00111100, 0b00111100,
  0b00111100, 0b00111100,
  0b00111100, 0b00111100,
  0b00111111, 0b11111100,
  0b00111111, 0b11111100,
  0b00011111, 0b11111000,
  0b00001111, 0b11110000,
  0b00000111, 0b11100000,
  0b00000000, 0b00000000,
  };  

//****************************** Hardware Declarations ****************************
TwoWire I2C2 = TwoWire(1);

SparkFun_Bio_Sensor_Hub bioHub(BIO_RESET_PIN, BIO_MFIO_PIN);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_SSD1327 display(128, 128, &Wire, OLED_RESET_PIN, 400000);

//**************************** Global Variables *********************************
bool deviceConnected = false;

char BLEbuf[50];

bioData body;
uint32_t temp;
float handTemp;
double pressure;
double max_pressure;
double init_pressure = 1;
double pressTotal;

int power_debounce = 0;

int debounce = 0;
int capReadings = 0;
unsigned long cap;
unsigned long capTotal = 0;
unsigned long capInitial = 0;
unsigned long capFinal_1 = 0;
unsigned long capFinal_2 = 0;
unsigned long capFinal_3 = 0;
static unsigned long dispUpdate = 1;
long refreshRate = 2000;

int state = IDLE;
static unsigned long breathStart = 0;
static unsigned long lastRead = 0;
static unsigned long endOfSpike = 0;
static unsigned long inactive_time = 0;
static unsigned long lv_time = 0;
bool connection = 0;

uint8_t date [3] = {0, 0, 0}; 
char id [10];

uint16_t voltage = 9999;

uint8_t charge = 0;

//********************************* Initialization *******************************
void setup() {
  Serial.begin(115200); 

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(ENABLE_12V_PIN, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, 1);
  digitalWrite(GREEN_LED_PIN, 0);
  digitalWrite(RED_LED_PIN, 0);
  digitalWrite(ENABLE_12V_PIN, 0); 

  Wire.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 400000);
  Wire.setTimeout(100);

  I2C2.begin(I2C2_SDA_PIN,I2C2_SCL_PIN, 100000);
  I2C2.setTimeout(100); 

  displayInit();

  capInit();

  //Initialize the pulse oximeter
  bodyInit();

  mlx.begin(MLX_ADDR, &I2C2);

  bleInit();

  BQ27425Init();

}


//******************************** Main Controll Loop ***************************
void loop() {
  if(micros() >= lastRead + PERIOD){
    lastRead = micros();

    
    
    if((state != IDLE)|(state != MOUTHPIECE_MISSING)|(state != SHUTDOWN)){
      pressureInit();
      sensorsRead();
      dataLog();
      bleUpdate();
    }else{
      body.confidence = 0;
      body.heartRate = 0;
      body.oxygen = 0;
      handTemp = 0;  
      temp = 0;
      cap = 0; 
      pressure = 0;
    }

    

    double capVal = (((double)(cap))/8388608)*4096;
    double valInit = (((double)(capInitial))/8388608)*4096;
    double valFinal_1 = (((double)(capFinal_1))/8388608)*4096;
    double valFinal_2 = (((double)(capFinal_2))/8388608)*4096;
    double valFinal_3 = (((double)(capFinal_3))/8388608)*4096;


    switch(state){
      case SHUTDOWN:
        digitalWrite(ENABLE_12V_PIN, 0);
        digitalWrite(GREEN_LED_PIN, 0);
        digitalWrite(RED_LED_PIN, 0);
        bioHub.max30101Control(0);

        delay(500);

        rtc_gpio_pullup_en(GPIO_NUM_6);
        rtc_gpio_pulldown_dis(GPIO_NUM_6);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)GPIO_NUM_6, 0);
        esp_deep_sleep_start();
        break;

      case IDLE:
        if(voltage < SHUTDOWN_VOLTAGE){
          state = LOW_VOLTAGE;
          lv_time = micros();
        }


        state = INITILIZE;

        digitalWrite(ENABLE_12V_PIN, 1);
        digitalWrite(GREEN_LED_PIN, 1);



        bioHub.max30101Control(1);
        break;

      //Initialize the device, take the initial capacitance measurements before the breath 
      case INITILIZE:
        if(voltage < SHUTDOWN_VOLTAGE){
          state = LOW_VOLTAGE;
          lv_time = micros();
        }
        //Check if the button has been pressed
        if(!digitalRead(BUTTON_PIN)){
          state = SHUTDOWN;
        }
        //Check if the mouthpiece is connected
        if(!detectMouthpiece()){
          state = MOUTHPIECE_MISSING;
          inactive_time = micros();
          break;
        }

        if(date[0] == 0){
          date[0] = readEEPROM(1);
          date[1] = readEEPROM(2);
          date[2] = readEEPROM(3);

          for(int i = 0; i < 9; i++){
            id[i] = readEEPROM(i + 4);
          }

        }


        //Update the display
        if(dispUpdate + refreshRate < micros() ){
          display.clearDisplay();
          display.setTextSize(2);
          display.setTextColor(0xF);
          display.setCursor(16, 50);
          display.print("Initilize");
          display.setCursor(45, 70);
          display.print("...");
          drawUI();
          display.display();
          dispUpdate = micros();
        }
        //Set the LEDs
        digitalWrite(GREEN_LED_PIN, 1);
        digitalWrite(RED_LED_PIN, 0);

        //Update initial capacitance measurements
        capTotal += cap;
        capReadings += 1;

        pressTotal += pressure;

        //Once enough capacitance samples have been taken move onto wait for breath
        if(capReadings >= BREATH_SAMPLES){
          capInitial = capTotal/capReadings;
          init_pressure = pressTotal/capReadings;
          state = WAIT_FOR_BREATH;
          capTotal = 0;
          capReadings = 0;
          inactive_time = micros();
        }
        break;

      //Wait until the mouthpiece is breathed into
      case WAIT_FOR_BREATH:
        if(voltage < SHUTDOWN_VOLTAGE){
          state = LOW_VOLTAGE;
          lv_time = micros();
        }
        //Check if the button has been pressed
        if((!digitalRead(BUTTON_PIN))|(inactive_time + SLEEP_TIME < micros())){
          state = SHUTDOWN;
        }
        //Check if the mouthpiece is connected
        if(!detectMouthpiece()){
          state = MOUTHPIECE_MISSING;
          inactive_time = micros();
          break;
        }




        //Update the display
        if(dispUpdate + refreshRate < micros() ){
          display.clearDisplay();
          display.setTextSize(2);
          display.setTextColor(0xF);
          display.setCursor(25, 50);
          display.print("Waiting");
          display.setCursor(45, 70);
          display.print("For");
          display.setCursor(15, 90);
          display.print("Breath...");
          drawUI();
          display.display();
          dispUpdate = micros();
        }
        
        //If the capacitance of the sensor increases move onto next state (after debouncing)
        if(capVal >= valInit + BREATH_SENSITIVITY){
          debounce += 1;
        }
        if(debounce >= 3){
          state = BREATH;
          debounce = 0;
          breathStart = micros();
        }         
        break;

      //Waits until the breath has been finished
      case BREATH:
        if(voltage < SHUTDOWN_VOLTAGE){
          state = LOW_VOLTAGE;
          lv_time = micros();
        }
        //Checks if the button has been pressed
        if(!digitalRead(BUTTON_PIN)){
          state = SHUTDOWN;
        }
        //Checks if the mouthpiece is connected
        if(!detectMouthpiece()){
          state = MOUTHPIECE_MISSING;
          inactive_time = micros();
          break;
        }

        if(pressure - init_pressure > max_pressure){
          max_pressure = pressure - init_pressure;
        }

        //Updates display
        if(dispUpdate + refreshRate < micros() ){
          display.clearDisplay();
          display.setTextSize(2);
          display.setTextColor(0xF);

          if(breathStart + BREATH_LENGTH > micros()){
            display.setCursor(25, 50);
            display.print("Breathe");
            display.setCursor(40, 70);
            display.setTextSize(3);
            itoa((breathStart + BREATH_LENGTH - micros())/1000000, BLEbuf, 10);
            display.print(BLEbuf);
            display.print(".");
            itoa(((breathStart + BREATH_LENGTH - micros())/100000)%10, BLEbuf, 10);
            display.print(BLEbuf);  
          }else if(max_pressure < BREATH_PRESSURE){
            display.setTextSize(2);
            display.setCursor(10, 50);
            display.print("Breath");
            display.setCursor(10, 70);
            display.print("Again");
          } else {
            display.setTextSize(2);
            display.setCursor(10, 50);
            display.print("Waiting");
            display.setCursor(45, 70);
            display.print("...");
          }
  
          drawUI();
          display.display();
          dispUpdate = micros();
        }

        //Checks if the breath has ended
        if((capVal <= BREATH_END_THRESHOLD)&(breathStart + BREATH_LENGTH < micros())&(max_pressure >= BREATH_PRESSURE)){
          debounce += 1;
        }

        //Debounce for detecting when the breath has ended
        if(debounce >= 3){
          state = WAIT_AFTER_BREATH_1;
          endOfSpike = micros();
          capReadings = 0;
        }
        break;

      //Waiting for the moisture to evaporate after the breath
      case WAIT_AFTER_BREATH_1:
        if(voltage < SHUTDOWN_VOLTAGE){
          state = LOW_VOLTAGE;
          lv_time = micros();
        }
        //Check if the button has been pressed
        if(!digitalRead(BUTTON_PIN)){
          state = SHUTDOWN;
        }

        //Check if the mouthpiece is there
        if(!detectMouthpiece()){
          state = MOUTHPIECE_MISSING;
          inactive_time = micros();
          break;
        }

        //Update the diplay
        if(dispUpdate + refreshRate < micros() ){
          display.clearDisplay();
          display.setTextColor(0xF);
          display.setTextSize(2);
          display.setCursor(0, 50);
          display.print("Processing");
          display.setCursor(45, 70);
          display.print("...");
          drawUI();
          display.display();
          dispUpdate = micros();
        }

        //Check if enough time has passed
        if(micros() >= endOfSpike + BREATH_DELAY_1){
          capTotal = 0;
          capReadings = 0;
          state = MEASURE_1;
        }
        break;

      //Meaure the final capacitance value of the sensor
      case MEASURE_1:
        if(voltage < SHUTDOWN_VOLTAGE){
          state = LOW_VOLTAGE;
          lv_time = micros();
        }
        //Check if the button has been pressed
        if(!digitalRead(BUTTON_PIN)){
          state = SHUTDOWN;
        }
        //Check if the mouthpiece is connected
        if(!detectMouthpiece()){
          state = MOUTHPIECE_MISSING;
          inactive_time = micros();
          break;
        }

        //Update the display
        if(dispUpdate + refreshRate < micros() ){
          display.clearDisplay();
          display.setTextColor(0xF);
          display.setTextSize(2);
          display.setCursor(0, 50);
          display.print("Processing");
          display.setCursor(45, 70);
          display.print("...");
          drawUI();
          display.display();
          dispUpdate = micros();
        }

        //Update the capacitance readings
        capTotal += cap;
        capReadings += 1;
        if(capReadings >= BREATH_SAMPLES){
          //Find the average of the readings
          capFinal_1 = capTotal/capReadings;
          state = WAIT_AFTER_BREATH_2;
        }
        break;

      case WAIT_AFTER_BREATH_2:
        if(voltage < SHUTDOWN_VOLTAGE){
          state = LOW_VOLTAGE;
          lv_time = micros();
        }
        //Check if the button has been pressed
        if(!digitalRead(BUTTON_PIN)){
          state = SHUTDOWN;
        }

        //Check if the mouthpiece is there
        if(!detectMouthpiece()){
          state = MOUTHPIECE_MISSING;
          inactive_time = micros();
          break;
        }

        //Update the diplay
        if(dispUpdate + refreshRate < micros() ){
          display.clearDisplay();
          display.setTextColor(0xF);
          display.setTextSize(2);
          display.setCursor(0, 50);
          display.print("Processing");
          display.setCursor(45, 70);
          display.print("...");
          drawUI();
          display.display();
          dispUpdate = micros();
        }

        //Check if enough time has passed
        if(micros() >= endOfSpike + BREATH_DELAY_2){
          capTotal = 0;
          capReadings = 0;
          state = MEASURE_2;
        }
        break;

      //Meaure the final capacitance value of the sensor
      case MEASURE_2:
        if(voltage < SHUTDOWN_VOLTAGE){
          state = LOW_VOLTAGE;
          lv_time = micros();
        }
        //Check if the button has been pressed
        if(!digitalRead(BUTTON_PIN)){
          state = SHUTDOWN;
        }
        //Check if the mouthpiece is connected
        if(!detectMouthpiece()){
          state = MOUTHPIECE_MISSING;
          inactive_time = micros();
          break;
        }

        //Update the display
        if(dispUpdate + refreshRate < micros() ){
          display.clearDisplay();
          display.setTextColor(0xF);
          display.setTextSize(2);
          display.setCursor(0, 50);
          display.print("Processing");
          display.setCursor(45, 70);
          display.print("...");
          drawUI();
          display.display();
          dispUpdate = micros();
        }

        //Update the capacitance readings
        capTotal += cap;
        capReadings += 1;
        if(capReadings >= BREATH_SAMPLES){
          //Find the average of the readings
          capFinal_2 = capTotal/capReadings;
          state = WAIT_AFTER_BREATH_3;

        }
        break;

      case WAIT_AFTER_BREATH_3:
        if(voltage < SHUTDOWN_VOLTAGE){
          state = LOW_VOLTAGE;
          lv_time = micros();
        }
        //Check if the button has been pressed
        if(!digitalRead(BUTTON_PIN)){
          state = SHUTDOWN;
        }

        //Check if the mouthpiece is there
        if(!detectMouthpiece()){
          state = MOUTHPIECE_MISSING;
          inactive_time = micros();
          break;
        }

        //Update the diplay
        if(dispUpdate + refreshRate < micros() ){
          display.clearDisplay();
          display.setTextColor(0xF);
          display.setTextSize(2);
          display.setCursor(0, 50);
          display.print("Processing");
          display.setCursor(45, 70);
          display.print("...");
          drawUI();
          display.display();
          dispUpdate = micros();
        }

        //Check if enough time has passed
        if(micros() >= endOfSpike + BREATH_DELAY_3){
          capTotal = 0;
          capReadings = 0;
          state = MEASURE_3;
        }
        break;

      //Meaure the final capacitance value of the sensor
      case MEASURE_3:
        if(voltage < SHUTDOWN_VOLTAGE){
          state = LOW_VOLTAGE;
          lv_time = micros();
        }
        //Check if the button has been pressed
        if(!digitalRead(BUTTON_PIN)){
          state = SHUTDOWN;
        }
        //Check if the mouthpiece is connected
        if(!detectMouthpiece()){
          state = MOUTHPIECE_MISSING;
          inactive_time = micros();
          break;
        }

        //Update the display
        if(dispUpdate + refreshRate < micros() ){
          display.clearDisplay();
          display.setTextColor(0xF);
          display.setTextSize(2);
          display.setCursor(0, 50);
          display.print("Processing");
          display.setCursor(45, 70);
          display.print("...");
          drawUI();
          display.display();
          dispUpdate = micros();
        }

        //Update the capacitance readings
        capTotal += cap;
        capReadings += 1;
        if(capReadings >= BREATH_SAMPLES){
          //Find the average of the readings
          capFinal_3 = capTotal/capReadings;
          state = COMPLETE;
          inactive_time = micros();
        }
        break;



      //Display the final results
      case COMPLETE:
        if(voltage < SHUTDOWN_VOLTAGE){
          state = LOW_VOLTAGE;
          lv_time = micros();
        }
        //If the virus is detected display that, else display that the result is negative
        if(dispUpdate + refreshRate < micros() ){
          display.clearDisplay();
          display.setTextColor(0xF);
          display.setTextSize(2);
//          display.setCursor(0, 50);
//          display.print("Measurements");
          display.setCursor(20, 70);
          display.print("Complete");
          drawUI();
          display.display();
          dispUpdate = micros();
        }

        if(!detectMouthpiece()){
          state = MOUTHPIECE_MISSING;
          inactive_time = micros();
          break;
        }

        if(inactive_time + SLEEP_TIME < micros()){
          state = SHUTDOWN;
        }

        //Check if the button has been pressed
        if(!digitalRead(BUTTON_PIN)){
          state = INITILIZE;
        }

        break;

      //If the mouthpiece is disconnected wait until it is connected again
      case MOUTHPIECE_MISSING:
        if(voltage < SHUTDOWN_VOLTAGE){
          state = LOW_VOLTAGE;
          lv_time = micros();
        }

          pressTotal = 0;
          capTotal = 0;
          capInitial = 0;
          capFinal_1 = 0;
          capFinal_2 = 0;
          capFinal_3 = 0;
          debounce = 0;
          capReadings = 0;
          max_pressure = 0;
          date[0] = 0;
          date[1] = 0;
          date[2] = 0;

          if(dispUpdate + refreshRate < micros() ){
            if(handTemp == 1){
              display.clearDisplay();
              display.setTextColor(0xF); 
              display.setTextSize(2);
              display.setCursor(20, 50);
              display.print("Connect");
              display.setCursor(45,65);
              display.print("Cable");
              drawUI();
              display.display();
              dispUpdate = micros();
            }else{
              display.clearDisplay();
              display.setTextColor(0xF); 
              display.setTextSize(2);
              display.setCursor(20, 50);
              display.print("Connect");
              display.setCursor(0,65);
              display.print("Mouthpiece");
              drawUI();
              display.display();
              dispUpdate = micros();
            }
          }

          delay(1);
          //Check if the mouthpiece has been disconnected
          if(detectMouthpiece()){
            state = INITILIZE;
          }

          //Check if the button has been pressed
          if((!digitalRead(BUTTON_PIN))|(inactive_time + SLEEP_TIME < micros())){
            state = SHUTDOWN;
          }
        break;
      
      case LOW_VOLTAGE:
        if(lv_time + 10000000 < micros()){
          state = SHUTDOWN;
        }

        if(voltage > SHUTDOWN_VOLTAGE + 200){
          state = INITILIZE;
          lv_time = 0;
        }
        
        if(dispUpdate + refreshRate < micros() ){
            
              display.clearDisplay();
              display.setTextColor(0xF); 
              display.setTextSize(2);
              display.setCursor(20, 50);
              display.print("BATTERY");
              display.setCursor(45,65);
              display.print("LOW");
              display.display();
              dispUpdate = micros();
          }
        
        
        break;
      
    }
 
    //charge = BQ27425GetSOC();
    uint16_t temp_volt = BQ27425GetVolt();
    if(temp_volt > 500) {
      voltage = temp_volt;
    }
  }  
}





//******************************* Bluetooth Functions **************************
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      connection = 1;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      connection = 0;
      
      //BLEAdvertising *pAdvertising = pServer->getAdvertising();
      //pAdvertising->addServiceUUID(SERVICE_UUID);
      //pAdvertising->setMinPreferred(0x00);
      //pAdvertising->setScanResponse(true);
      //pAdvertising->setMinPreferred(0x06);
      //pAdvertising->setMinPreferred(0x12);
      //pAdvertising->start();
      bleInit();

    }
};

void bleInit(){
  BLEDevice::init(DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristicVS = pService->createCharacteristic(
                        VS_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

  pCharacteristicVS->setValue("0");



  pCharacteristicBODY = pService->createCharacteristic(
                        BODY_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

  pCharacteristicBODY->setValue("0");



  pCharacteristicTMP = pService->createCharacteristic(
                         TMP_CHARACTERISTIC_UUID,
                         BLECharacteristic::PROPERTY_READ |
                         BLECharacteristic::PROPERTY_WRITE |
                         BLECharacteristic::PROPERTY_NOTIFY
                       );

  pCharacteristicTMP->setValue("0");


  pCharacteristicMEAS = pService->createCharacteristic(
                        MEAS_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

  pCharacteristicMEAS->setValue("0");

  pCharacteristicSTAT = pService->createCharacteristic(
                        STAT_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  
  itoa(THRESHOLD, BLEbuf, 16); 

  pCharacteristicSTAT->setValue(BLEbuf);

  pCharacteristicPRES = pService->createCharacteristic(
                        PRES_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristicPRES->setValue("0");

  pCharacteristicID = pService->createCharacteristic(
                        ID_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristicID->setValue("0");

  // begin by starting the service, then starting to advertize the service
  // so devices can discover it.
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setMinPreferred(0x00);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();  
}

void bleUpdate(){

  String hr;
  String ox;

  if (body.confidence > 0) {
    hr = String(body.heartRate);
  }else{
    hr = String("0");
  }  

  if (body.oxygen > 0) {
    ox = String(body.oxygen);
  }else{
    ox = String("0");
  }

  String transmit = String(hr + "," + ox);
  transmit.toCharArray(BLEbuf, 50);
  pCharacteristicBODY->setValue(BLEbuf);


  String tempHigh = String(temp);
  String tempLow = String(handTemp);
  transmit = String(tempHigh + "," + tempLow);

  transmit.toCharArray(BLEbuf, 50);
  pCharacteristicTMP->setValue(BLEbuf);


  double valInit = (((double)(capInitial))/8388608)*4096;
  double valFinal_1 = (((double)(capFinal_1))/8388608)*4096;
  double valFinal_2 = (((double)(capFinal_2))/8388608)*4096;
  double valFinal_3 = (((double)(capFinal_3))/8388608)*4096;

  String init = String(valInit);
  String fin1 = String(valFinal_1);
  String fin2 = String(valFinal_2);
  String fin3 = String(valFinal_3);

  transmit = String(init + "," + fin1 + "," + fin2 + "," + fin3);
  transmit.toCharArray(BLEbuf, 50);
  pCharacteristicMEAS->setValue(BLEbuf);


  if(init_pressure != 1){
    itoa(pressure - init_pressure, BLEbuf, 10);
    pCharacteristicPRES->setValue(BLEbuf);
  }else{
    itoa(0, BLEbuf, 10);
    pCharacteristicPRES->setValue(BLEbuf);    
  }

  double capVal = (((double)(cap))/8388608)*4096;
  itoa(capVal, BLEbuf, 10);
  pCharacteristicVS->setValue(BLEbuf);


  transmit = String(state);
  transmit.toCharArray(BLEbuf, 50);
  pCharacteristicSTAT->setValue(BLEbuf);


  String date_1 = String(date[0]);
  String date_2 = String(date[1]);
  String date_3 = String(date[2]);
  String id_s = String(id);
  transmit = String(date_1 + "/" + date_2 + "/" +date_3 + "," + id_s);
  transmit.toCharArray(BLEbuf, 50);
  pCharacteristicID->setValue(BLEbuf);


}

//************************** AD7745 Capacitance to Digital  *********************
void capInit(){
  Wire.beginTransmission(CAP_ADDR);
  Wire.write(byte(0x07));
  Wire.write(byte(0x80));
  Wire.endTransmission();

  Wire.beginTransmission(CAP_ADDR);
  Wire.write(byte(0x09));
  Wire.write(byte(0x23));
  Wire.endTransmission();

  Wire.beginTransmission(CAP_ADDR);
  Wire.write(byte(0x0A));
  Wire.write(byte(0x21));
  Wire.endTransmission();
}

unsigned long capRead(){
  int capStatus = 0;
  unsigned long capData = 0;

  Wire.requestFrom(CAP_ADDR, 4);

  if(4 <= Wire.available()){
    capStatus = Wire.read();
    byte capDataH = Wire.read();
    byte capDataM = Wire.read();
    byte capDataL = Wire.read();
    capData = (capDataH << 16)|(capDataM << 8)|capDataL;
    
    if((capData <= 8388608)&(verbose == 2)){
      Serial.println("[ERROR]Negative capacitance!"); 
      return 0;
    }
    
    capData = capData - 8388608;
    
    return capData;

  }else{
    if(verbose == 2){
      Serial.println("I2C Failure!");
    }
    return 0;
  }

}

//*********************************** Display *********************************
void displayInit(){
  display.begin(OLED_ADDR);
  display.clearDisplay();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(0xF);
  display.setCursor(16, 50);
  display.print("Initilize");
  display.setCursor(45, 70);
  display.print("...");
  drawUI();
  display.display();
  dispUpdate = micros();  
  
  
  //display.display();
}

void drawUI(){
  if(connection){
    display.drawBitmap(0, 0, ble_bitmap, 8, 16, 0xF);
  } else {
    display.drawBitmap(0, 0, ble_bitmap, 8, 16, 0x6);
  }
  
  display.drawBitmap(70, 0, u_bitmap, 16, 16, 0xF);

  display.setTextSize(1);
  display.setTextColor(0xF);
  display.setCursor(20, 4);
  display.print("EXAMIN");

  itoa(charge, BLEbuf, 10);
  
  uint8_t charge_voltage = (voltage - SHUTDOWN_VOLTAGE)/11;

  if(charge_voltage > 100){
    charge_voltage = 100;
  }

  

  if(charge_voltage == 100){
    display.setCursor(100, 4);  
  }else if(charge_voltage >= 10){
    display.setCursor(105, 4);
  }else{
    display.setCursor(112, 4);
  }
  ////display.setCursor(90, 4); //////////
  itoa(charge_voltage, BLEbuf, 10);
  display.print(BLEbuf);
  display.setCursor(120, 4);
  display.print("%");

  display.setCursor(0, 20);
  display.print("Pres:");
  if(pressure - init_pressure < 9999){
    
    itoa(pressure - init_pressure, BLEbuf, 10);
  } else {
    itoa(0, BLEbuf, 10);
  }
  display.print(BLEbuf);  
  display.print("Pa");

  display.setCursor(60, 20);
  if (body.confidence > 0) {
    display.setTextColor(0xF);
  } else {
    display.setTextColor(0x6);
  }
  display.print("HR:");
  itoa(body.heartRate, BLEbuf, 10);
  display.print(BLEbuf);
  display.setCursor(98, 20);
  display.print("O2:");
  itoa(body.oxygen, BLEbuf, 10);
  display.print(BLEbuf);

  display.setTextColor(0xF);
  display.setCursor(0, 31);
  display.print("Temp:");
  itoa(handTemp, BLEbuf, 10);
  display.print(BLEbuf);
  display.print("C");

  display.setCursor(66, 31);
  display.print("Virus:");
  double capVal1 = (((double)(cap))/8388608)*4.096;
  itoa(capVal1, BLEbuf, 10);
  display.print(BLEbuf);
  display.print(".");  
  double capVal2 = (int)(capVal1*100) - (int)(capVal1)*100;
  itoa(capVal2, BLEbuf, 10);
  display.print(BLEbuf);

  display.setTextSize(0.25);
  display.setCursor(0, 120);
  
  double valFinal_1 = (((double)(capFinal_1))/8388608)*4096;
  double valFinal_2 = (((double)(capFinal_2))/8388608)*4096;
  double valFinal_3 = (((double)(capFinal_3))/8388608)*4096;

  display.setTextColor(0x6);
  itoa(valFinal_1, BLEbuf, 10);
  display.print("1:");
  display.print(BLEbuf);

  itoa(valFinal_2, BLEbuf, 10);
  display.print(" 2:");
  display.print(BLEbuf);

  itoa(valFinal_3, BLEbuf, 10);
  display.print(" 3:");
  display.print(BLEbuf);

  if(date[0] != 0){
    display.setCursor(0, 110);
    itoa(date[2], BLEbuf, 10);
    display.print(BLEbuf);
    display.print("/");
    itoa(date[0], BLEbuf, 10);
    display.print(BLEbuf);
    display.print("/");
    itoa(date[1], BLEbuf, 10);
    display.print(BLEbuf);
    display.print(" ID:");
    display.print(id);
  }

  display.drawLine(0, 18, 128, 18, 0xF);
  display.drawLine(0, 29, 128, 29, 0xF);
  display.drawLine(0, 40, 128, 40, 0xF);
}

//******************************** Pressure Sensor ****************************
void pressureInit(){
  Wire.beginTransmission(PRESS_ADDR);
  Wire.write(byte(0x0F));
  Wire.endTransmission();
  Wire.requestFrom(PRESS_ADDR, 1);
  Wire.read();

  Wire.beginTransmission(PRESS_ADDR);
  Wire.write(byte(0x10));
  Wire.write(byte(0b01110000));
  Wire.endTransmission();
}

double pressureRead(){
  Wire.beginTransmission(PRESS_ADDR);
  Wire.write(byte(0x28));
  Wire.endTransmission();
  Wire.requestFrom(PRESS_ADDR,3);
  uint32_t data[3];
  data[0] = Wire.read();
  data[1] = Wire.read();
  data[2] = Wire.read();
  uint32_t ret = data[0];
  ret |= data[1] << 8;
  ret |= data[2] << 16;

  return (ret * 1000)/40960;
}


//************************************ Other Sensors ***************************
void bodyInit(){
  int result = bioHub.begin(I2C2);
  if (result == 0){
    if(verbose == 2){
      Serial.println("Sensor started!");
    }
  }else{
    if(verbose == 2){
      Serial.println("Could not communicate with the sensor!!!");
    }
  }
  
  //Serial.println("Configuring Sensor....");
  int error = bioHub.configBpm(MODE_ONE); // MODE_ONE
  if (error == 0) {
    if(verbose == 2){
      Serial.println("Sensor configured.");
    }
  }
  else {
    if(verbose == 2){
      Serial.println("Error configuring sensor.");
      Serial.print("Error: ");
      Serial.println(error);
    }
  }

  //bioHub.setPulseWidth(69); //////////////

}

uint32_t tempRead(){
  Wire.beginTransmission(PRESS_ADDR);
  Wire.write(byte(0x2b));
  Wire.endTransmission();
  Wire.requestFrom(PRESS_ADDR,2);
  uint32_t data[2];
  data[0] = Wire.read();
  data[1] = Wire.read();
  uint32_t ret = data[0];
  ret |= data[1] << 8;

  return ret;
}

//****************************** System Functions ******************************
bool detectMouthpiece(){
  
  if(handTemp == 1){
    bodyInit();
    mlx.begin(MLX_ADDR, &I2C2); 
    return 0;
  }
  

  Wire.beginTransmission(PRESS_ADDR);
  Wire.write(byte(0x0F));
  Wire.endTransmission();
  Wire.requestFrom(PRESS_ADDR,1);
  if(Wire.read() == 0xb3){
    return 1;
  }else{
    return 0;
  }

  


}

void sensorsRead(){
  body = bioHub.readBpm();

///  handTemp = 50;//////////////////////
  handTemp = mlx.readObjectTempC();  
  if(isnan(handTemp)){
    handTemp = 1;
  }

  temp = tempRead();
  

  
  cap = capRead(); 
  pressure = pressureRead();

  pressureInit();
}

void dataLog(){
  if(verbose == 1){
    double capVal = (((double)(cap))/8388608)*4096;
    double valInit = (((double)(capInitial))/8388608)*4096;
    double valFinal = (((double)(capFinal_1))/8388608)*4096;
    double valFinal_1 = (((double)(capFinal_1))/8388608)*4096;
    double valFinal_2 = (((double)(capFinal_2))/8388608)*4096;
    double valFinal_3 = (((double)(capFinal_3))/8388608)*4096;

    Serial.print("start,");
    Serial.print(capVal);
    Serial.print(",");
    Serial.print(body.heartRate); 
    Serial.print(",");
    Serial.print(body.confidence);
    Serial.print(",");
    Serial.print(body.oxygen);
    Serial.print(",");
    Serial.print(temp);
    Serial.print(",");
    Serial.print(handTemp);
    Serial.print(",");
    Serial.print(pressure);
    Serial.print(",");
    Serial.print(valInit);
    Serial.print(",");
    Serial.print(valFinal_1);
    Serial.print(",");
    Serial.print(valFinal_2);
    Serial.print(",");
    Serial.print(valFinal_3);
    Serial.print(",");
    Serial.print(id);
    Serial.print(",");
    Serial.print((int) date[0]);
    Serial.print((int) date[1]);
    Serial.print((int) date[2]);
    Serial.print(",");
    Serial.println(state);
  }

  if(verbose == 2) {
    double capVal = (((double)(cap))/8388608)*4096;
    Serial.print("Virus: ");
    Serial.println(capVal);
    Serial.print("Heartrate: ");
    Serial.println(body.heartRate);  
    Serial.print("Confidence: ");
    Serial.println(body.confidence);
    Serial.print("Oxygen: ");
    Serial.println(body.oxygen); 
    Serial.print("Breath Temp: ");
    Serial.println(temp);
    Serial.print("Hand Temp: ");
    Serial.println(handTemp); 
    Serial.print("Pressure: ");
    Serial.println(pressure);
    Serial.print("State: ");
    Serial.println(state);

    //Serial.print("PULSE STATUS-----: ");
    //Serial.println(body.extStatus);

    //Serial.print("FINGER STATUS-----: ");
    //Serial.println(body.extStatus);

    Serial.print("Battery Voltage: ");
    Serial.println(voltage);

  }
}


//***************************** Battery Manegement ******************************
void I2CReadBuffer(uint8_t adr, uint8_t reg, uint8_t *buffer, uint8_t length){
  Wire.beginTransmission(adr);
  Wire.write(reg);
  
  int ret = Wire.endTransmission(false);

  if(ret != 0){
    Serial.print("Error transmiting!! code: ");
    Serial.println(ret);
  }

  Wire.requestFrom(adr, length);

  for(int i = 0; i<length; i++){
    buffer[i] = Wire.read();
  }
}

void I2CWriteBuffer(uint8_t adr, uint8_t reg, uint8_t *buffer, uint8_t length){
  uint8_t payload[33];

  payload[0] = reg;

  for(int i = 0; i < length; i++){
    payload[i + 1] = buffer[i];
  }

  Wire.beginTransmission(adr);
  Wire.write(payload, length + 1);
  int ret = Wire.endTransmission(true);

  if(ret != 0){
    Serial.print("Error transmiting!! code: ");
    Serial.println(ret);
  }
}

uint8_t BQ27425GetSOC(){

  uint8_t SOC = 0;
  I2CReadBuffer(ADR_GG, GG_CMD_SOC, &SOC, 1);

  return SOC;

}

uint16_t BQ27425GetVolt(){

  uint8_t volt[2] = {0,0};
  I2CReadBuffer(ADR_GG, GG_CMD_VOLT, &volt[0], 2);
  delay(10);
  I2CReadBuffer(ADR_GG, GG_CMD_VOLT + 1, &volt[1], 2);

  uint16_t voltage = volt[0];
  voltage |= volt[1] << 8;
  return voltage;


   
}



void BQ27425Init(){
  uint8_t val[32] = {0,0};

  I2CReadBuffer(ADR_GG, GG_CMD_DCAP, val, WORD_SIZE_GG);
  int16_t designCap = val[0] | (val[1] << 8);

  Serial.print("\nDesign capacity: ");
  Serial.println(designCap);

  if(designCap != 3350){
        
    // Config update mode
    val[0] = GG_CTL_SETCFGUPD & 0xFF;
    val[1] = (GG_CTL_SETCFGUPD >> 8);
    I2CWriteBuffer(ADR_GG, GG_CMD_CNTL, val, WORD_SIZE_GG);

    // Enable access to NVM
    val[0] = 0;
    val[1] = 0;
    I2CWriteBuffer(ADR_GG, GG_CEXT_BLDATCTRL, val, WORD_SIZE_GG);
        
    // Set the data block to 0th 32-byte block
    I2CWriteBuffer(ADR_GG, GG_CEXT_DATBLOCK, val, WORD_SIZE_GG);
        
    // Set the DataClass being accessed to 'State [NVM]'
    val[0] = GG_CLASS_STATE & 0xFF;
    val[1] = (GG_CLASS_STATE >> 8);
    I2CWriteBuffer(ADR_GG, GG_CEXT_DATCLASS, val, WORD_SIZE_GG);
        
    // Wait for the data to be ready
    delayMicroseconds(400); //200

    // Read first 32-byte block of 'State [NVM]'
    I2CReadBuffer(ADR_GG, GG_NVM_DATA(0), val, 32);
        
    // Modify OpConfig - Set BATLOWEN
    val[GG_OFS_OPCFG+1] |= (1 << BATLOWEN);
        
    // Modify design capacity
    val[GG_OFS_DESCAP] = (3350 >> 8);
    val[GG_OFS_DESCAP+1] = 3350 & 0xFF;

    // Modify design energy
    val[GG_OFS_DESENRGY] = (12060 >> 8);
    val[GG_OFS_DESENRGY+1] = 12060 & 0xFF;

    // Calculate checksum
    uint8_t i = 0;
    uint8_t checksum = 0;

    for (i = 0; i < 32; i++) checksum += val[i];
        
    checksum = ~checksum;

    // Save changes to NVM scratch area
    I2CWriteBuffer(ADR_GG, GG_NVM_DATA(0), val, 32);
        
    // Write checksum to trigger write from scratch to NVM
    val[0] = checksum;
    val[1] = 0;
    I2CWriteBuffer(ADR_GG, GG_CEXT_BLDATCHKS, val, WORD_SIZE_GG);

    // Send BAT_INSERT
    val[0] = GG_CTL_BATINSERT & 0xFF;
    val[1] = (GG_CTL_BATINSERT >> 8);
    I2CWriteBuffer(ADR_GG, GG_CMD_CNTL, val, WORD_SIZE_GG);

    // Send BAT_INSERT
    val[0] = GG_CTL_SOFTRESET & 0xFF;
    val[1] = (GG_CTL_SOFTRESET >> 8);
    I2CWriteBuffer(ADR_GG, GG_CMD_CNTL, val, WORD_SIZE_GG);    
  }
}
 

 //****************************** EEPROM ****************************************
uint8_t readEEPROM(uint8_t addr){
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(byte(addr));
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDR, 1);
  return Wire.read();
}

void writeEEPROM(uint8_t addr, uint8_t data){
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(byte(addr));
  Wire.write(byte(data));
  Wire.endTransmission();
}


