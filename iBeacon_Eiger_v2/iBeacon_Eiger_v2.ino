
#include <ArduinoBLE.h>
#include <vector>

// Function prototype
float calculateMedian(const std::vector<int>& vec);
std::vector<int> rssiValues; 

BLEService customService("173d");

BLEStringCharacteristic RSSI_value("2A59", BLERead| BLENotify, 30);
String RSSI_Jungfrau = "";

unsigned long previousScanTime = 0;
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
  BLE.poll();
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousScanTime >= scanInterval) {
            
    
    BLE.scan();
    delay(40);
    BLEDevice peripheral = BLE.available();
     
    while(peripheral){
       if( peripheral.address() == "74:4d:bd:81:37:99"){
      int rssi = peripheral.rssi();
      RSSI_Jungfrau = "RSSI Jungfrau: "+String(rssi) ;
      RSSI_value.writeValue(RSSI_Jungfrau);
      break;
       }

      peripheral = BLE.available();
    }
    previousScanTime = currentMillis;
    BLE.advertise();

  }
   
  }
