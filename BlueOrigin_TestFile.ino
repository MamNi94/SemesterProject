#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

BLEService customService("250A");

BLEStringCharacteristic IMU_Characteristic("2A58", BLERead| BLENotify, 30);
String IMU_Data = "";

unsigned long previousScanTime = 50;
const unsigned long scanInterval = 100; 

void setup() {
    Serial.begin(9600);
    while(!Serial)
      // Initialize BLE
    if (!BLE.begin()) {
        //Serial.println("Starting BLE failed!");
        while (1);
    }

      if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
    
    
    // Create BLE Service
    BLE.setLocalName("Blue Origin");
    
    BLE.setAdvertisedService(customService);
    customService.addCharacteristic(IMU_Characteristic);

    BLE.addService(customService);

    BLE.advertise();

    IMU_Characteristic.writeValue(IMU_Data);
    
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousScanTime >= scanInterval) {
        

      float accelx, accely, accelz;
      IMU.readAcceleration(accelx, accely, accelz);
      IMU_Data = "X " + String(accelx) + "  Y "+ String(accely) +"  Z "+ String(accelz);

      IMU_Characteristic.writeValue(IMU_Data);
      previousScanTime = currentMillis;
      BLE.advertise();
        
    }
}
