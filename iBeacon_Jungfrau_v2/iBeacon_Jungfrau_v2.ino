#include <ArduinoBLE.h>
//#include <vector>


BLEService customService("180c");

BLEStringCharacteristic RSSI_value("2A57", BLERead| BLENotify, 30);
String RSSI_Breithorn = "";

unsigned long previousScanTime = 0;
const unsigned long scanInterval = 10; 

void setup() {

    BLE.begin();
    
    // Create BLE Service
    BLE.setLocalName("iBeacon Jungfrau");
    
    BLE.setAdvertisedService(customService);
    customService.addCharacteristic(RSSI_value);

    BLE.addService(customService);
 
    // Start advertising
    BLE.advertise();

    RSSI_value.writeValue(RSSI_Breithorn);
    
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousScanTime >= scanInterval) {
       
        BLE.scan();
        delay(40);
        BLEDevice peripheral = BLE.available();
        while (peripheral){
          if( peripheral.address() == "74:4d:bd:81:18:75"){
          int rssi = peripheral.rssi();
          String RSSI_Breithorn = "RSSI Breithorn: "+String(rssi);
          RSSI_value.writeValue(RSSI_Breithorn);
          break;
          }

          peripheral = BLE.available();  
        }
       
        previousScanTime = currentMillis;

        BLE.advertise();
    
        
    }
    BLE.advertise();

}