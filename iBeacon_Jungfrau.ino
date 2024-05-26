#include <ArduinoBLE.h>
//#include <vector>

// Function prototype
//float calculateMedian(const std::vector<int>& vec);
//std::vector<int> rssiValues; 

BLEService customService("180c");

BLEStringCharacteristic RSSI_value("2A57", BLERead| BLENotify, 30);
String RSSI_Breithorn = "";

unsigned long previousScanTime = 50;
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

        BLE.scanForUuid("192a");
        BLEDevice peripheral = BLE.available();
        if (peripheral){
          int rssi = peripheral.rssi();
            //rssiValues.insert(rssiValues.begin(), rssi);

             //Check Vector Size
            //if (rssiValues.size() > 10){
            //  rssiValues.pop_back(); 
           // }

            // Calculate median
            //float medianValue = calculateMedian(rssiValues);

            String RSSI_Breithorn = "RSSI Breithorn: "+String(rssi);
            RSSI_value.writeValue(RSSI_Breithorn);
        }
        previousScanTime = currentMillis;
        BLE.advertise();
         if(!BLE.central()){
          delay(100);
      }
        
    }
    
}

/*
float calculateMedian(const std::vector<int>& vec) {
  // Make a copy of the vector since we need to sort it
  std::vector<int> sortedVec = vec;
  
  // Sort the vector
  insertionSort(sortedVec);

  // Calculate the median
  size_t size = sortedVec.size();
  if (size == 0) {
    // No elements in the vector
    return 0;
  } else if (size % 2 == 0) {
    // Vector has even number of elements
    return (sortedVec[size / 2 - 1] + sortedVec[size / 2]) / 2.0;
  } else {
    // Vector has odd number of elements
    return sortedVec[size / 2];
  }
}


void insertionSort(std::vector<int>& vec) {
  for (size_t i = 1; i < vec.size(); ++i) {
    int key = vec[i];
    int j = i - 1;
    while (j >= 0 && vec[j] > key) {
      vec[j + 1] = vec[j];
      j = j - 1;
    }
    vec[j + 1] = key;
  }
}
*/
