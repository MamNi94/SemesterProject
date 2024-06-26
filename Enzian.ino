#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

BLEService customService("250A");


BLEStringCharacteristic IMU_Characteristic("2ABA", BLERead| BLENotify, 30);
String IMU_Data = "";
BLEStringCharacteristic RSSI_Characteristic_Eiger("2CCC", BLERead| BLENotify, 30);
String RSSI_Data_Eiger = "";
BLEStringCharacteristic RSSI_Characteristic_Jungfrau("2DDD", BLERead| BLENotify, 30);
String RSSI_Data_Jungfrau = "";
BLEStringCharacteristic RSSI_Characteristic_Breithorn("2BBB", BLERead| BLENotify, 30);
String RSSI_Data_Breithorn = "";


unsigned long previousAdvertiseTime = 50;
const unsigned long AdvertiseInterval = 0; 


int rssi_eiger = 0;
int rssi_jungfrau = 0;
int rssi_breithorn = 0;

const char* targetAddressJungfrau = "74:4d:bd:81:37:99";
const char* targetAddressEiger = "74:4d:bd:81:26:dd";
const char* targetAddressBreithorn = "74:4d:bd:81:18:75";


int eiger = 0;
int jung = 0;
int bh = 0;
int total = 0;


void setup() {
    
    //Serial.begin(9600);
    //while (!Serial);
    //Serial.println("Start");

    BLE.begin();
    IMU.begin();
    
    // Create BLE Service
    BLE.setLocalName("Enzian");
    
    BLE.setAdvertisedService(customService);


    customService.addCharacteristic(IMU_Characteristic);
    customService.addCharacteristic(RSSI_Characteristic_Eiger);
    customService.addCharacteristic(RSSI_Characteristic_Jungfrau);
     customService.addCharacteristic(RSSI_Characteristic_Breithorn);

    BLE.addService(customService);

    BLE.advertise();
    
}

void loop() {

    BLE.scan();
    delay(20);
     
    BLEDevice peripheral = BLE.available();

    while(peripheral){
        //Serial.println(peripheral.address());
     
        if( peripheral.address() == targetAddressJungfrau){
          rssi_jungfrau = peripheral.rssi();
          jung +=1;
        }
        if( peripheral.address() == targetAddressEiger){
          rssi_eiger = peripheral.rssi();
          eiger+=1;
        }
        if( peripheral.address() == targetAddressBreithorn){
          rssi_breithorn = peripheral.rssi();
          bh+=1;
        }
        peripheral = BLE.available();
   
     }
     BLE.stopScan();
     //Serial.println("Next Iteration");
      total+=1;
    

      //RSSI_Data_Breithorn =  String(bh)+"/"+String(total)  +"Breit Direct "+ String(rssi_breithorn)  ;
      //RSSI_Data_Jungfrau =  String(jung)+"/"+String(total)  +"Jung Direct " + String(rssi_jungfrau);
      //RSSI_Data_Eiger =  String(eiger)+"/"+String(total) +"Eiger Direct " + String(rssi_eiger);

      RSSI_Data_Breithorn =  "Breit Direct "+ String(rssi_breithorn)  ;
      RSSI_Data_Jungfrau =  "Jung Direct " + String(rssi_jungfrau);
      RSSI_Data_Eiger =  "Eiger Direct " + String(rssi_eiger);

      float accelx, accely, accelz;
      IMU.readAcceleration(accelx, accely, accelz);
      IMU_Data = "X" + String(accelx) + " Y"+ String(accely) +" Z"+ String(accelz);

      IMU_Characteristic.writeValue(IMU_Data);
      RSSI_Characteristic_Eiger.writeValue(RSSI_Data_Eiger);
      RSSI_Characteristic_Jungfrau.writeValue(RSSI_Data_Jungfrau);
      RSSI_Characteristic_Breithorn.writeValue(RSSI_Data_Breithorn);
      

      
      
      BLE.advertise();
      if(!BLE.central()){
          delay(100);
      }
      

   
}
