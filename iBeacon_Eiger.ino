
#include <ArduinoBLE.h>
#include <vector>

// Function prototype
float calculateMedian(const std::vector<int>& vec);
std::vector<int> rssiValues; 

BLEService customService("173d");

BLEStringCharacteristic RSSI_value("2A59", BLERead| BLENotify, 30);
String RSSI_Jungfrau = "";

unsigned long previousScanTime = 50;
const unsigned long scanInterval = 10; 


void setup() {



    BLE.begin();

    
    // Create BLE Service
    BLE.setLocalName("iBeacon Eiger");
    
    BLE.setAdvertisedService(customService);
    customService.addCharacteristic(RSSI_value);

    BLE.addService(customService);
 
    // Start advertising
    BLE.advertise();

    RSSI_value.writeValue(RSSI_Jungfrau);
    
}

void loop() {

  unsigned long currentMillis = millis();
  
  if (currentMillis - previousScanTime >= scanInterval) {
            
    
    BLE.scanForUuid("180c");
    BLEDevice peripheral = BLE.available();
     
    if(peripheral){

      int rssi = peripheral.rssi();
      RSSI_Jungfrau = "RSSI Jungfrau: "+String(rssi) + " T = "+String(currentMillis);
      RSSI_value.writeValue(RSSI_Jungfrau);
    }
    previousScanTime = currentMillis;
    BLE.advertise();
    if(!BLE.central()){
          delay(100);
      }
    
  }
  }

     
