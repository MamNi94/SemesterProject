
#include <ArduinoBLE.h>


BLEService customService("192a");

BLEStringCharacteristic RSSI_value("2A58", BLERead| BLENotify, 30);
String RSSI_Eiger = "";

unsigned long previousScanTime = 50 ;
const unsigned long scanInterval = 10; 

void setup() {

    BLE.begin();
    
    // Create BLE Service
    BLE.setLocalName("iBeacon Breithorn");
    
    BLE.setAdvertisedService(customService);
    customService.addCharacteristic(RSSI_value);

    BLE.addService(customService);
 
    // Start advertising
    BLE.advertise();

    RSSI_value.writeValue(RSSI_Eiger);
    
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousScanTime >= scanInterval) {

      BLE.scan();
      delay(40);

      BLEDevice peripheral = BLE.available();
        while (peripheral){


            
            int rssi = peripheral.rssi();
    
            if( peripheral.address() == "74:4d:bd:81:26:dd"){
            String RSSI_Eiger = "RSSI Eiger: "+String(rssi);
            RSSI_value.writeValue(RSSI_Eiger);
                  break;
             }
             peripheral = BLE.available();
        }
        previousScanTime = currentMillis;
        BLE.advertise();
      
        
    }
}