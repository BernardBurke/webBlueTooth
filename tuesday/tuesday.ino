/*
   Emulate the Examin Handset to Anthony's HTML5 app
   created 6 Feb 2024
   by 
   Credit to: Tom Igoe for the example code
*/
#include <ArduinoBLE.h>
#include <RTCZero.h>
// this define is a macro. It says "take whatever is in parentheses
// and add it in the middle of this string. This way you don't have
// to write out the whole string in each characteristic:
#define MY_UUID(val) ("555a0002-" val "-467a-9538-01f0652c74e8");

// fill in your name here. It will be combined with
// the Arduino's MAC address to set the peripheral BLE name:
const char myName[] = "Examin-Handset";

// BB replace the set up the service and the characteristics:
// BLEService                     service                 (MY_UUID("0000"));
BLEService                        service                 ("ba65b745-f9f7-4873-9289-bb93564909f3");
// Will make all the characteristics Int for now
//BLEUnsignedLongCharacteristic  epochCharacteristic     (MY_UUID("0001"), BLERead | BLEWrite | BLENotify);
//BLEIntCharacteristic           timeZoneCharacteristic  (MY_UUID("0002"), BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic             HeartRateOxygen   ("f87ef99e-a101-4518-958b-4def53dc4b8e", BLERead | BLEWrite | BLENotify ); // HR, Oxygen
//BLEIntCharacteristic             TMP_CHARACTERISTIC_UUID     ("38bca6e3-05a4-487d-95b0-351a3e25eae1");   // temp, Hand Temp
//BLEIntCharacteristic             MEAS_CHARACTERISTIC_UUID    ("10762b73-87ad-4368-b9b9-a2ab27cfaad7");    // vir_init, vir_1, vir_2, vir3
//BLEIntCharacteristic             PRES_CHARACTERISTIC_UUID     ("2f63d787-50dc-42e6-ace0-a403958a02b6"); // pressure
//BLEIntCharacteristic             VS_CHARACTERISTIC_UUID      ("ba65b745-f9f7-4873-9289-bb93564909f3");   // raw Virus
//BLEIntCharacteristic             STAT_CHARACTERISTIC_UUID    ("3e9c01e6-12de-11eb-adc1-0242ac120002");    // state
//BLEIntCharacteristic             ID_CHARACTERISTIC_UUID    ("7946624e-155d-11ee-be56-0242ac120002");      // id

// static data
long randomNumber;
float floatrandNumber;

void setup() {
  Serial.begin(9600);
  // wait 3 seconds if serial port not open:
  if (!Serial) delay(3000);

  // start the realtime clock:
  //rtc.begin();

  // start the BLE radio:
  if (!BLE.begin()) {
    Serial.println("Failed to start BLE");
  }
  // set the peripheral's local name:
  BLE.setLocalName(myName);

  // set the advertised service:
  BLE.setAdvertisedService(service);
  // add the characteristics:
  service.addCharacteristic(HeartRateOxygen);
  //service.addCharacteristic(TMP_CHARACTERISTIC_UUID);
  
  // set initial values for the characteristics:
  //epochCharacteristic.writeValue(0);
  HeartRateOxygen.writeValue(10);
  //TMP_CHARACTERISTIC_UUID.writeValue(11);

  // add the service to the peripheral and advertise it:
  BLE.addService(service);
  BLE.advertise();
  
  randomSeed(analogRead(0));
}

void loop() {
  // poll for connections:
  randomNumber = random(80,90);
  BLE.poll();
  // if you're connected to a central:
  if (BLE.connected()) {
    //Serial.print("Connected");
    if (millis() % 10000 < 3) {
      Serial.print(randomNumber);
      Serial.println(" Changing value"  );
      HeartRateOxygen.writeValue(randomNumber);
      }
  }
}
