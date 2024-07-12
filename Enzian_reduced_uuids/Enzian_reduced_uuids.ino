#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

BLEService customService("250A");


BLEStringCharacteristic IMU_Characteristic("2ABA", BLERead| BLENotify, 30);
String IMU_Data = "";
BLEStringCharacteristic GYRO_Characteristic("3ABA", BLERead| BLENotify, 30);
String Gyro_Data = "";

BLEStringCharacteristic RSSI_Characteristic("2CCC", BLERead| BLENotify, 30);
String RSSI_Data = "";


int all_connected = 0;
int rssi_eiger = 0;
int rssi_jungfrau = 0;
int rssi_breithorn = 0;
float accelx, accely, accelz;
float roll, pitch, yaw;

const char* targetAddressJungfrau = "74:4d:bd:81:37:99";
const char* targetAddressEiger = "74:4d:bd:81:26:dd";
const char* targetAddressBreithorn = "74:4d:bd:81:18:75";


int eiger_counter = 0;
int jungfrau_counter = 0;
int breithorn_counter = 0;
int counter = 0;



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
    customService.addCharacteristic(GYRO_Characteristic);
    customService.addCharacteristic(RSSI_Characteristic);



    BLE.addService(customService);

    BLE.advertise();
    
}

void loop() {
    while(BLE.connected()){
      BLE.poll();
      BLE.scan();
      delay(40);

      BLEDevice peripheral = BLE.available();
      all_connected = 0;
      while(peripheral){

      
          if( peripheral.address() == targetAddressJungfrau){
            rssi_jungfrau = peripheral.rssi();
            //Serial.println(rssi_jungfrau);
            jungfrau_counter +=1;
            all_connected +=1;
          }
          if( peripheral.address() == targetAddressEiger){
            rssi_eiger = peripheral.rssi();
            eiger_counter+=1;
            all_connected +=1;
          }
          
          if( peripheral.address() == targetAddressBreithorn){
            rssi_breithorn = peripheral.rssi();
            breithorn_counter+=1;
            all_connected +=1;
          }
          if( all_connected ==3){
            break;
          }
          
          peripheral = BLE.available();

      }
        BLE.stopScan();
        //Serial.println(all_connected);

        RSSI_Data=  "E"+ String(rssi_eiger) + " J"+ String(rssi_jungfrau)  + " B"+ String(rssi_breithorn)  ;
    


        IMU.readAcceleration(accelx, accely, accelz);
        IMU_Data = "X" + String(accelx) + " Y"+ String(accely) +" Z"+ String(accelz);
        IMU.readGyroscope(roll, pitch, yaw);
        roll = round(roll*10)/10;
        pitch = round(pitch*10)/10;
        yaw = round(yaw*10)/10;
        
        Gyro_Data =   "R" + String(roll)+"P"+ String(pitch) +"Y"+ String(yaw);

        IMU_Characteristic.writeValue(IMU_Data);
        GYRO_Characteristic.writeValue(Gyro_Data);
        RSSI_Characteristic.writeValue(RSSI_Data);



        BLE.advertise();
        
        }

}