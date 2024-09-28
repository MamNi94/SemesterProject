from bluepy.btle import Scanner, Peripheral, ADDR_TYPE_PUBLIC, DefaultDelegate
import threading
import time
import numpy as np
import keyboard
import re
import os
import sys
from scipy.optimize import least_squares
from filterpy.kalman import KalmanFilter
import joblib
import yaml

def enable_notifications(peripheral,uuids, handle_to_uuid):
    for service in peripheral.getServices():
            print("\nService: {}".format(service))
            # Print characteristics
            
            for char in service.getCharacteristics():
                print("    Characteristic: {}".format(char))

                for uuid in uuids:
                    if char.uuid == uuid: 
                            # Read the value of the characteristic
                            value = char.read()
                        
                            print("        Value:", value.hex())  # Assuming the value is in hexadecimal format
                            
                            notification = False
                            while notification == False:           
                            #peripheral.withDelegate(notification_delegate)
                                try:
                                    handle_to_uuid[char.getHandle()] = str(uuid)
                                    peripheral.withDelegate(NotificationDelegate(handle_to_uuid))
                                    peripheral.writeCharacteristic(char.getHandle() + 1, b"\x01\x00", withResponse=True)
                                    notification = True
                                except:
                                    notification = False
                            print(f'{uuid} connected, starting notifications...')

    return peripheral








    
#Define Globals
stop_threads = False
test_array = []
connected_global = {}
peripherals_global = []
CamPos = False

#load model
quadratic_distance_model =  joblib.load('quadratic_distance_model_modified.pkl')

#Define Beacon Locations
A = np.array([0.0, 0, 0])
B = np.array([0, 1, 0])
C = np.array([0.9, 0.5,  -0.8])

if CamPos == True:
    #load KameraPositions
    # Load the YAML file
    with open('transforms.yaml', 'r') as file:
        data = yaml.safe_load(file)


    # Accessing individual transformations
    T_0S = data['transforms']['T_0S']
    H1 = data['transforms']['H1']
    H2 = data['transforms']['H2']
    H3 = data['transforms']['H3']

    x0 = np.ones([4,1])
    #Assume Eiger at Robot / Midpoint of Robot CAD Model = x0
    A = x0[0:3] + np.array([0,0.085,0])
    #Jungfrau at H1
    B = x0@H1@T_0S + np.array([0, -0.06, -0.015])
    #Breithorn at H2
    C = x0@H2@T_0S + np.array([0, -0.06, -0.015])






UUID_to_Sensor = {

    '2ABA': 'IMU',
    '3ABA': 'Gyro',
    '2CCC':'Direct',
    '2A59':'Jungfrau TXPower',
    '2A58':'Eiger TXPower',
    '2A57':'Breithorn TXPower'

}

Sensor_Data = {
    'IMU': {'X':0,'Y':0,'Z':0},
    'Gyro': {'R':0,'P':0,'Y':0},
    'Direct': {'E': -20, 'J':-20, 'B':-20},
    'Jungfrau TXPower':-10,
    'Eiger TXPower':-10,
    'Breithorn TXPower':-10
    
}
#Extract RSSI from Strings
rssi_pattern = r'-?\d+'
imu_pattern = re.compile(r'([XYZ])([-+]?\d*\.\d+|\d+)')
gyro_pattern = re.compile(r'([RPY])([-+]?\d+)')
rssi_pattern_enzian =re.compile(r'([EJB])([-+]?\d+)')


# A function to listen for the key press to stop threads
def listen_for_stop_key():
    global stop_threads
    global peripherals_global
    keyboard.wait('q')  # Press 'q' to stop threads
    stop_threads = True
    print("Key pressed, stopping threads...")
    for peripheral in peripherals_global:
        peripheral.disconnect()
    sys.exit()
##########################

def get_system_of_equations(p,d1,d2,d3):
    global A,B,C

    x,y,z = p
    return[
        np.sqrt((x-A[0])**2 +(y-A[1])**2 + (z-A[2])**2 ) - d1,
        np.sqrt((x-B[0])**2 +(y-B[1])**2 + (z-B[2])**2 ) - d2,
        np.sqrt((x-C[0])**2 +(y-C[1])**2 + (z-C[2])**2 ) - d3
    ]




def print_dict(d):
    """Print each key-value pair of the dictionary on a new line."""
    for key, value in d.items():
        print(f"{key}: {value}")

def quad_model_predict(RSSI, TxPower):
    a = 0.91
    b = 0.07
    c = -0.23
  
    x = RSSI / TxPower
    coefficients = [a, b, c - x]
    d = max(np.roots(coefficients))
    return round(d,3)

def lin_model_predict(RSSI, TxPower):
    a = 1.51
    b = -0.77
    x = RSSI / TxPower
    d= (x-b)/a - 0.4


    return round(d,3)

def exponential_model_predict(RSSI,TxPower):

    a = 0.57
    b=0.42
    c=0.45

    x = RSSI / TxPower
    
    d = np.power((x - c) / a, 1 / b)
    return round(d,3)

def exp_model_predict(RSSI, TxPower):

    x = RSSI / TxPower
    a = 0.53
    b=4.19
    c=0.46
    d = -np.log(1 - (x - c) / a) / b

    if np.isnan(d):
        d = 0.2
    
    return round(d,3)

def log_model_predict(RSSI,TxPower):

    a = 0.54
    b = 0.75
    c = 1.3
    offset = 0.2
    x = RSSI / TxPower


    d = (np.exp((x-c)/a)/b)-offset

    if np.isnan(d):
        d = 0.2

    return round(d,3)

def calculate_position():
    print('Calculating position started....')
    global stop_threads
    global Sensor_Data
    global poly_model

    k =0
    RSSI_array = []
    RSSI_Eiger = []
    TX_power_Eiger = []

    distance = 0
    collect_data = True
   

    t = 0
    roll, pitch, yaw = 0,0,0
    last_time = time.time()

    starting_time = time.time()

    if collect_data == True:
    
            position_lin, position_poly, position_exp, position_exponential, position_log = {},{},{},{},{}
            distance_lin, distance_poly, distance_exp,distance_exponential, distance_log = {},{},{},{},{}

   
    while not stop_threads:
       
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        rssi_eiger = Sensor_Data['Direct']['E']
        rssi_jungfrau = Sensor_Data['Direct']['J']
        rssi_breithorn = Sensor_Data['Direct']['B']

        TxPower_Eiger = Sensor_Data['Eiger TXPower']
        TxPower_Jungfrau = Sensor_Data['Jungfrau TXPower']
        TxPower_Breithorn = Sensor_Data['Breithorn TXPower']
    
        k+=1


        
        initial_guess = np.array([0,0,0])
        #calculate lin model based distanc
        d_model_eiger_lin = lin_model_predict(rssi_eiger,TxPower_Eiger)
        d_model_jungfrau_lin =  lin_model_predict(rssi_jungfrau,TxPower_Jungfrau) 
        d_model_breithorn_lin = lin_model_predict(rssi_breithorn,TxPower_Breithorn) 

        if np.isnan(d_model_eiger_lin) or np.isnan(d_model_jungfrau_lin) or np.isnan(d_model_breithorn_lin):
            estimated_position_model = ['NAN']
        else:
            estimated_position_model_out = least_squares(get_system_of_equations, initial_guess, args = (d_model_eiger_lin, d_model_jungfrau_lin,d_model_breithorn_lin))
            estimated_position_model_lin = estimated_position_model_out.x
  
      

        #calculate quad model based distance
        d_model_eiger = quad_model_predict(rssi_eiger,TxPower_Eiger) 
        d_model_jungfrau =  quad_model_predict(rssi_jungfrau,TxPower_Jungfrau) 
        d_model_breithorn = quad_model_predict(rssi_breithorn,TxPower_Breithorn) 

        if np.isnan(d_model_eiger) or np.isnan(d_model_jungfrau) or np.isnan(d_model_breithorn):
            estimated_position_model = ['NAN']
        else:
            estimated_position_model_out = least_squares(get_system_of_equations, initial_guess, args = (d_model_eiger, d_model_jungfrau,d_model_breithorn))
            estimated_position_model = estimated_position_model_out.x

         #calculate quad model based distance
        d_model_eiger_exponential = exponential_model_predict(rssi_eiger,TxPower_Eiger) 
        d_model_jungfrau_exponential =  exponential_model_predict(rssi_jungfrau,TxPower_Jungfrau) 
        d_model_breithorn_exponential = exponential_model_predict(rssi_breithorn,TxPower_Breithorn)

        if np.isnan(d_model_eiger_exponential) or np.isnan(d_model_jungfrau_exponential) or np.isnan(d_model_breithorn_exponential):
            estimated_position_model_exp = ['NAN']
        else:
            estimated_position_model_out = least_squares(get_system_of_equations, initial_guess, args = (d_model_eiger_exponential, d_model_jungfrau_exponential,d_model_breithorn_exponential))
            estimated_position_model_exponential = estimated_position_model_out.x

      
        d_model_eiger_exp = exp_model_predict(rssi_eiger,TxPower_Eiger) 
       
        d_model_jungfrau_exp =  exp_model_predict(rssi_jungfrau,TxPower_Jungfrau) 
    
        d_model_breithorn_exp = exp_model_predict(rssi_breithorn,TxPower_Breithorn)

        if np.isnan(d_model_eiger_exp) or np.isnan(d_model_jungfrau_exp) or np.isnan(d_model_breithorn_exp):
            estimated_position_model_exp = ['NAN']
        else:
            estimated_position_model_out = least_squares(get_system_of_equations, initial_guess, args = (d_model_eiger_exp, d_model_jungfrau_exp,d_model_breithorn_exp))
            estimated_position_model_exp = estimated_position_model_out.x
        
        
        d_model_eiger_log = log_model_predict(rssi_eiger,TxPower_Eiger) 
       
        d_model_jungfrau_log =  log_model_predict(rssi_jungfrau,TxPower_Jungfrau) 
    
        d_model_breithorn_log = log_model_predict(rssi_breithorn,TxPower_Breithorn)

        if np.isnan(d_model_eiger_log) or np.isnan(d_model_jungfrau_log) or np.isnan(d_model_breithorn_log):
            estimated_position_model_exp = ['NAN']
        else:
            estimated_position_model_out = least_squares(get_system_of_equations, initial_guess, args = (d_model_eiger_log, d_model_jungfrau_log,d_model_breithorn_log))
            estimated_position_model_log= estimated_position_model_out.x

    
        if collect_data ==True:
            
            angle = 0
            roll_collection = 0
            pitch_collection  = 0
            yaw_colleciton = 0


            if k%1 ==0:
                #RSSI Plot Data Collection
                if keyboard.is_pressed('g'):
                    RSSI_Eiger.append(rssi_eiger)
                    TX_power_Eiger.append(Sensor_Data['Eiger TXPower'])
                if keyboard.is_pressed('d'):
                    RSSI_array.append([1+((distance/100) * np.cos(angle/180 *np.pi)),1 + ((distance/100) * np.sin(angle/180 * np.pi)),np.mean(RSSI_Eiger), np.mean(TX_power_Eiger)])
                    RSSI_Eiger = []
                    TX_power_Eiger = []
                    
                if keyboard.is_pressed('b'):
                    distance = distance-10
                if keyboard.is_pressed('f'):
                    distance = distance+10

                if  keyboard.is_pressed('s'):
                    with open(f'RSSI_Data/rssi_list_{angle}_interpolation_data_XIAO_4.txt', 'w') as f:
                        for item in RSSI_array:
                            f.write(f"{item}\n")
                if keyboard.is_pressed('p'):
                    actual_pos = [0, 0.5,0]
                    t = time.time() - starting_time

                    position_lin[t] = [actual_pos, [estimated_position_model_lin[0],estimated_position_model_lin[1],estimated_position_model_lin[2]]]
                    distance_lin[t] = [d_model_eiger_lin,d_model_jungfrau_lin,d_model_breithorn_lin]

                    position_poly[t] = [actual_pos, [estimated_position_model[0],estimated_position_model[1],estimated_position_model[2]]]
                    distance_poly[t] = [d_model_eiger,d_model_jungfrau,d_model_breithorn]

                    position_exponential[t] = [actual_pos, [estimated_position_model_exponential[0],estimated_position_model_exponential[1],estimated_position_model_exponential[2]]]
                    distance_exponential[t] = [d_model_eiger_exponential,d_model_jungfrau_exponential,d_model_breithorn_exponential]

                    position_exp[t] = [actual_pos, [estimated_position_model_exp[0],estimated_position_model_exp[1],estimated_position_model_exp[2]]]
                    distance_exp[t] = [d_model_eiger_exp,d_model_jungfrau_exp,d_model_breithorn_exp]

                    position_log[t] = [actual_pos, [estimated_position_model_log[0],estimated_position_model_log[1],estimated_position_model_log[2]]]
                    distance_log[t] = [d_model_eiger_log,d_model_jungfrau_log,d_model_breithorn_log]

                position_number = 1
                if keyboard.is_pressed('i'):
                    with open(f'position_data/measured_positions_lin_pos{position_number}.txt', 'w') as f:
                            for key, value in position_lin.items():
                                f.write(f"{key}: {value}\n")

                    with open(f'position_data/measured_distances_lin_pos{position_number}.txt', 'w') as f:
                            for key, value in distance_lin.items():
                                f.write(f"{key}: {value}\n")

                    with open(f'position_data/measured_positions_poly_pos{position_number}.txt', 'w') as f:
                            for key, value in position_poly.items():
                                f.write(f"{key}: {value}\n")

                    with open(f'position_data/measured_distances_poly_pos{position_number}.txt', 'w') as f:
                            for key, value in distance_poly.items():
                                f.write(f"{key}: {value}\n")

                    with open(f'position_data/measured_positions_exponential_pos{position_number}.txt', 'w') as f:
                            for key, value in position_exponential.items():
                                f.write(f"{key}: {value}\n")

                    with open(f'position_data/measured_distances_exponential_pos{position_number}.txt', 'w') as f:
                            for key, value in distance_exponential.items():
                                f.write(f"{key}: {value}\n")

                    with open(f'position_data/measured_positions_exp_pos{position_number}.txt', 'w') as f:
                            for key, value in position_exp.items():
                                f.write(f"{key}: {value}\n")

                    with open(f'position_data/measured_distances_exp_pos{position_number}.txt', 'w') as f:
                            for key, value in distance_exp.items():
                                f.write(f"{key}: {value}\n")
                    
                    with open(f'position_data/measured_positions_log_pos{position_number}.txt', 'w') as f:
                            for key, value in position_log.items():
                                f.write(f"{key}: {value}\n")

                    with open(f'position_data/measured_distances_log_pos{position_number}.txt', 'w') as f:
                            for key, value in distance_log.items():
                                f.write(f"{key}: {value}\n")

                

        if keyboard.is_pressed('a'):
                roll,pitch,yaw = 0,0,0     

        k = 1
        os.system('clear')
        
        print_dict(Sensor_Data)

        print('3D position POLY', estimated_position_model)
        print('3D position LINEAR', estimated_position_model_lin)
        print('3D position EXPONENTIAL', estimated_position_model_exponential)
        print('3D position EXP', estimated_position_model_exp)
        print('3D position LOG', estimated_position_model_log)
 
    
        print('Distance Poly  Eiger: ',d_model_eiger, ' Jungfrau: ', d_model_jungfrau, '  Breithorn: ', d_model_breithorn)
        print('Distance Lin Eiger: ',d_model_eiger_lin, ' Jungfrau: ', d_model_jungfrau_lin, '  Breithorn: ', d_model_breithorn_lin)
        print('Distance Exponential Eiger: ',d_model_eiger_exponential, ' Jungfrau: ', d_model_jungfrau_exponential, '  Breithorn: ', d_model_breithorn_exponential)
        print('Distance EXP Eiger: ',d_model_eiger_exp, ' Jungfrau: ', d_model_jungfrau_exp, '  Breithorn: ', d_model_breithorn_exp)
        print('Distance LOG Eiger: ',d_model_eiger_log, ' Jungfrau: ', d_model_jungfrau_log, '  Breithorn: ', d_model_breithorn_log)
       


        if collect_data == True:
            print('RSSI Data Eiger: ', RSSI_Eiger)
            print('TX_power Eiger ',TX_power_Eiger )
            print('current distance ', distance, ' For Data Collection')
   
            
        
       
       
        



# Custom delegate class to handle notifications
class NotificationDelegate(DefaultDelegate):

    def __init__(self, handle_to_uuid):
        DefaultDelegate.__init__(self)
        self.handle_to_uuid = handle_to_uuid        

    def handleNotification(self, cHandle, data):
        # Handle received notification data here
        global test_array
        global rssi_pattern
        global imu_pattern
        global Sensor_Data
        global UUID_to_Sensor
        global rssi_pattern_enzian

        decoded_data = data.decode('utf-8')
        test_array.append(decoded_data)
        uuid = self.handle_to_uuid[cHandle]

        if uuid == '2ABA' or uuid == '3ABA' or uuid == '2CCC':
            sensor =UUID_to_Sensor[uuid]
            if uuid == '2ABA':
                matches = imu_pattern.findall(decoded_data)
            if uuid == '3ABA':
                #print(decoded_data)
                matches = gyro_pattern.findall(decoded_data)
            if uuid == '2CCC':
                matches = rssi_pattern_enzian.findall(decoded_data)
                
            # Convert matches to a dictionary
            decoded_data_dict = {key: float(value) for key, value in matches}
            Sensor_Data[sensor] = decoded_data_dict
   
        else:
          
            match = re.search(rssi_pattern,decoded_data)
            if match:
                rssi = match.group()
                
                sensor =UUID_to_Sensor[uuid]
                Sensor_Data[sensor]    = float(rssi)
           
        
      
        #print(Sensor_Data)
        #print("Notification received on uuid {}: {}".format(self.handle_to_uuid[cHandle], decoded_data))
        
        
       
        
        
        

def print_services_and_characteristics(device, uuids):
    global stop_threads
    global connected_global
    global peripherals_global
   
    print("Services and Characteristics of Device {}:".format(device.addr))
    connected = False
    peripheral = None
    
    while not connected and not stop_threads:
        try:
            # Attempt to connect to the device
            peripheral = Peripheral(device.addr, device.addrType)
            if peripheral:
                connected = True
                connected_global[device.getValueText(9)] = True
        except Exception as e:
    
            print("Error connecting to device:", e)
            print(f'Retrying connection to device {device.getValueText(9)}...')
          

    peripherals_global.append(peripheral)
    try:
        # Print servicesa
        
        handle_to_uuid = {}
        peripheral = enable_notifications(peripheral, uuids,handle_to_uuid)

            
        while True and not stop_threads:
            try:
                peripheral.waitForNotifications(2)      
                    
            except:
                print('problem', uuids)
                #print_services_and_characteristics(device, uuids)
                handle_to_uuid = {}
                peripheral = enable_notifications(peripheral, uuids,handle_to_uuid)
                
                

                

    except Exception as e:
        print("Error:", e)
        print('Reconnecting...')
        print_services_and_characteristics(device, uuids)
        

device_names = ['iBeacon Eiger', 'iBeacon Jungfrau', 'iBeacon Breithorn', 'Enzian']
#device_names = ['Enzian','iBeacon Breithorn']
#device_names = [ 'iBeacon Jungfrau','iBeacon Breithorn','iBeacon Eiger', 'Enzian']
uuids = {
    #'Enzian': ['2ABA', '3ABA','2CCC'],
    'Enzian': ['2CCC'],
    'iBeacon Eiger': ['2A59'],
    'iBeacon Jungfrau': ['2A57'],
    'iBeacon Breithorn': ['2A58']}


for device_name in device_names:
    connected_global[device_name] = True

threads = []

# Create a scanner object

scanner = Scanner()


print('scanning for devices...')
# Scan for devices (you can specify a timeout in seconds)
devices = scanner.scan(timeout=4.0, passive = False)  # Scans for 10 seconds

print('finished scanning')
print(len(devices),' devices detected')

# Iterate through the scanned devices
for dev in devices:
    # Access device data

    if dev.getValueText(9) is not None and  dev.getValueText(9) in device_names:
        print("  Device Name:", dev.getValueText(9)) 
        name = dev.getValueText(9)
        uuid = uuids[name]
      
        print(name, uuid)
        threading.Thread(target = print_services_and_characteristics, args =  (dev, uuid,)).start()
        
        

threading.Thread(target = listen_for_stop_key).start()

while True:
    
    all_true = all(connected_global.values())
    
    if all_true == True:
        
        print('All Devices Connected')
        break

threading.Thread(target = calculate_position).start()


if stop_threads == True:
    print('stored data = ', test_array)
 

#Docu
'''
Start VsCode in Superuser mode: sudo code --no-sandbox --user-data-dir=/home/nicolas/VSCODE/.vscode_root

in case of error 13 (Issues with scanning): sudo service bluetooth restart

'''