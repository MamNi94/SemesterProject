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
const unsigned long AdvertiseInterval = 100; 


//unsigned long previousScanTime = 50;
//const unsigned long ScanInterval = 100; 

int i = 0;
int rssi_eiger = 0;
int rssi_jungfrau = 0;
int rssi_breithorn = 0;


void setup() {
    
    
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

   

    IMU_Characteristic.writeValue(IMU_Data);
    RSSI_Characteristic_Eiger.writeValue(RSSI_Data_Eiger);
    RSSI_Characteristic_Eiger.writeValue(RSSI_Data_Jungfrau);
    RSSI_Characteristic_Eiger.writeValue(RSSI_Data_Breithorn);

    BLE.advertise();

}

void loop() {

  while(BLE.central()){
  
  unsigned long currentMillis = millis();

  //Für Notificatoin Test
  if (currentMillis - previousAdvertiseTime >= AdvertiseInterval) {
    i +=1;
    if(i == 50){
       i =0;
    }
    
    BLE.scanForUuid("182c");
    BLEDevice eiger = BLE.available();
    if(eiger){
      rssi_eiger = eiger.rssi();
    }
    

    BLE.scanForUuid("180c");
    BLEDevice jungfrau = BLE.available();
    if(jungfrau){
      rssi_jungfrau = jungfrau.rssi();
    }

    BLE.scanForUuid("181c");
    BLEDevice breithorn = BLE.available();
    if(breithorn){
      rssi_breithorn = breithorn.rssi();
    }

    BLE.stopScan();
 
 
  
      RSSI_Data_Eiger =  "Eiger Direct " + String(rssi_eiger) + " " + String(i);
      RSSI_Data_Jungfrau =  "Jungfrau Direct " + String(rssi_jungfrau) + " " + String(i);
      RSSI_Data_Breithorn =  "Breithorn Direct " + String(rssi_breithorn) + " " + String(i);


      float accelx, accely, accelz;
      IMU.readAcceleration(accelx, accely, accelz);
      IMU_Data = "X" + String(accelx) + " Y"+ String(accely) +" Z"+ String(accelz);

      IMU_Characteristic.writeValue(IMU_Data);
      RSSI_Characteristic_Eiger.writeValue(RSSI_Data_Eiger);
      RSSI_Characteristic_Jungfrau.writeValue(RSSI_Data_Jungfrau);
      RSSI_Characteristic_Breithorn.writeValue(RSSI_Data_Breithorn);
      
      previousAdvertiseTime = currentMillis;
      
    
      BLE.advertise();
  } 
     
  }
  
}