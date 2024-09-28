BLE Datalake:
Python Script to run on a BLE capable laptop
Connects to Central Device Enzian, Beacon Eiger, Beacon Jungfrau and Beacon Breithorn using their Mac Adress
Recieves RSSI Values via notifications --> each BLE device has a service with a characteristic that contain the RSSI values
Calculates Distance and Position from the recieved RSSI values using different interpolation functions

There is the possibility to collect data for plotting 360 degree plots of RSSI, RSSI fitting functions or Position and saving the values to input files for the plotting scripts

This can be done by setting "collect_data" = True and then using the respective key presses

BLE_Parameter_Estimation:
Takes as input RSSI data values and gives the parameters for different interpolation functions and also plots the interpolated function

BLE_Project_Visualize_RSSI_data:
Takes 360 degree RSSI Data and creates equipotential line plots

Enzian:
Arduino code for the Handheld device Arduino Nano BLE
Searches for MAC adress of surrounding BLE devices and reads RSSI values of the beacon devices
Writes RSSI values of beacon devices into string Characteristic that is then broadcasted to connected central device

Ibeacon Eiger, Jungfrau, Breithorn:
Arduino code for Beacon Devices Xiao ESP32 S3
Scans for MAC adress of one neighboring beacon device
Reas RSSI of beacon device and sends it to connected central device
