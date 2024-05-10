#include <ArduinoBLE.h>
#include <vector>

// Function prototype
float calculateMedian(const std::vector<int>& vec);
std::vector<int> rssiValues; 

BLEService customService("181c");

BLEStringCharacteristic RSSI_value("2A58", BLERead| BLENotify, 30);
String RSSI_Eiger = "";

unsigned long previousScanTime = 50;
const unsigned long scanInterval = 100; 

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

        BLE.scanForUuid("182c");
        BLEDevice peripheral = BLE.available();
        if (peripheral){

            int rssi = peripheral.rssi();
            rssiValues.insert(rssiValues.begin(), rssi);

             //Check Vector Size
            if (rssiValues.size() > 10){
              rssiValues.pop_back(); 
            }

            // Calculate median
            float medianValue = calculateMedian(rssiValues);  

            String RSSI_Eiger = "RSSI Eiger: "+String(medianValue)+ " T = "+String(currentMillis);
            RSSI_value.writeValue(RSSI_Eiger);
        }
        previousScanTime = currentMillis;
        BLE.advertise();
        
    }
}


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