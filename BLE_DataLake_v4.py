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


def remove_gravity(acc, orientation):
  
    corrected_acc = acc

    roll = orientation[0]#roll
    pitch = orientation[1]#pitch
    yaw = orientation[2]#yaw
    
    roll = np.pi * roll / 180
    pitch = np.pi * pitch / 180
    yaw = np.pi * yaw / 180
 
    # Compute the rotation matrix for roll
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    # Compute the rotation matrix for pitch
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Compute the rotation matrix for yaw
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    

    R = R_z @ R_y @ R_x
    # Combine the rotations
 

    g = np.array([0,0,1])

    g_imu = R @ g
    # Correct the accelerometer data by subtracting the gravity vector
    corrected_acc['X'] = acc['X'] - g_imu[0]
    corrected_acc['Y'] = acc['Y'] - g_imu[1]
    corrected_acc['Z'] = acc['Z'] - g_imu[2]



    return corrected_acc


def Initiate_Kalman_Filter(initial_state):
    kf = KalmanFilter(dim_x = 6, dim_z = 6)
    #Measurement Funciton H
    kf.H = np.array([[1,0,0,0,0,0],
                     [0,1,0,0,0,0],
                     [0,0,1,0,0,0],
                     [0,0,0,1,0,0],
                     [0,0,0,0,1,0],
                     [0,0,0,0,0,1]])
    
    #Measurement Noise covariance
    kf.R = np.eye(6) * 0.01
    
    #process noise covariance
    q = 0.1
    kf.Q = np.array([[q,0,0,0,0,0],
                     [0,q,0,0,0,0],
                     [0,0,q,0,0,0],
                     [0,0,0,q,0,0],
                     [0,0,0,0,q,0],
                     [0,0,0,0,0,q]])
    
    kf.x = initial_state

    return kf
    

    



#Define Globals
stop_threads = False
test_array = []
connected_global = {}
peripherals_global = []
CamPos = False

#load model
quadratic_distance_model =  joblib.load('quadratic_distance_model_modified.pkl')

#Define Beacon Locations
A = np.array([0, 0, 0])
B = np.array([1, 0, 0])
C = np.array([0.5, -0.85,  0])

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
    'Direct': {'E': -2, 'J':-2, 'B':-2},
    'Jungfrau TXPower':-1,
    'Eiger TXPower':-1,
    'Breithorn TXPower':-1
    
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

def get_linear_distance(rssi_0 = -1, rssi_1 = -3, rssi_current = -1, distance_0 = -3, distance_1 = 1):
    try:
        m = (distance_1 - distance_0) / (rssi_1 - rssi_0)
    except:
        m = 1
    q = 0#distance_1 - m * rssi_1
    distance = m * rssi_current + q
    return distance


def print_dict(d):
    """Print each key-value pair of the dictionary on a new line."""
    for key, value in d.items():
        print(f"{key}: {value}")

def model_predict(RSSI, TxPower):
    a = -0.63
    b = 1.38
    c = 0.51
  
    x = RSSI / TxPower
    d = a*x**2 + b * x + c
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
    collect_data = False
   

    t = 0
    roll, pitch, yaw = 0,0,0
    last_time = time.time()
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
        #Distance Linear

        d_eiger_linear = get_linear_distance(rssi_0= -17, rssi_1=Sensor_Data['Eiger TXPower'], rssi_current= rssi_eiger , distance_0=0, distance_1=1)
        d_jungfrau_linear = get_linear_distance(rssi_0= -17, rssi_1=Sensor_Data['Jungfrau TXPower'], rssi_current=rssi_jungfrau, distance_0=0, distance_1=1)
        d_breithorn_linear = get_linear_distance(rssi_0= -17, rssi_1=Sensor_Data['Breithorn TXPower'], rssi_current=rssi_breithorn , distance_0=0, distance_1=1)
        d_eiger_linear = round(d_eiger_linear, 3)
        d_jungfrau_linear = round(d_jungfrau_linear,3)
        d_breithorn_linear = round(d_breithorn_linear,3)

        A,B,C = 0.89976,7.7095,0.111
        A,B,C = 0.74,0.17, 0.3
        #Distance Exponential
        d_Eiger = A * (np.power(rssi_eiger  / Sensor_Data['Eiger TXPower'],B)) + C
        d_Eiger = round(d_Eiger,3)
        d_Jungfrau = A * (np.power(rssi_jungfrau / Sensor_Data['Jungfrau TXPower'],B)) + C
        d_Jungfrau = round(d_Jungfrau,3)
        d_Breithorn =A * (np.power(rssi_breithorn / Sensor_Data['Breithorn TXPower'],B)) + C
        d_Breithorn = round(d_Breithorn,3)

       
        #Linear Fit
        #d_linear_fit = (Sensor_Data['Eiger Direct'] +22.356) / -41.333
        initial_guess = np.array([0,0,0])
        if np.isnan(d_Eiger) or np.isnan(d_Breithorn) or np.isnan(d_Jungfrau):
            estimated_position_exp = ['NAN']
        else:
            estimated_position_exp_out = least_squares(get_system_of_equations, initial_guess, args = (d_Eiger, d_Jungfrau,d_Breithorn))
            estimated_position_exp = estimated_position_exp_out.x
        estimated_position_lin = least_squares(get_system_of_equations, initial_guess, args = (d_eiger_linear, d_jungfrau_linear,d_breithorn_linear)).x
        
        #Apply Kalman Filter
        if t ==0:
            initial_state = np.array([estimated_position_exp[0],estimated_position_exp[1],estimated_position_exp[2],0,0,0])
            kf = Initiate_Kalman_Filter(initial_state)
        imu_data = Sensor_Data['IMU'].copy()
        gyro_data = Sensor_Data['Gyro']
        delta_roll = gyro_data['R']
        delta_pitch = gyro_data['P']
        delta_yaw = gyro_data['Y']
        roll = roll +delta_roll*dt
        pitch = pitch + delta_pitch*dt
        yaw = yaw + delta_yaw*dt
  
      

        #calculate model based distance
        adjustment = 0
        d_model_eiger = model_predict(rssi_eiger,TxPower_Eiger) -adjustment
        d_model_jungfrau =  model_predict(rssi_jungfrau,TxPower_Jungfrau) -adjustment
        d_model_breithorn = model_predict(rssi_breithorn,TxPower_Breithorn) -adjustment

        if np.isnan(d_model_eiger) or np.isnan(d_model_jungfrau) or np.isnan(d_model_breithorn):
            estimated_position_model = ['NAN']
        else:
            estimated_position_model_out = least_squares(get_system_of_equations, initial_guess, args = (d_model_eiger, d_model_jungfrau,d_model_breithorn))
            estimated_position_model = estimated_position_model_out.x
        
        adjusted_imu_data = remove_gravity(imu_data.copy(), [roll,pitch,yaw])
        state=np.array([estimated_position_lin[0], estimated_position_lin[1],estimated_position_lin[0] ,adjusted_imu_data['X'], adjusted_imu_data['Y'], adjusted_imu_data['Z']])
        
        #qquatratic model:
        d_quad_model_eiger  = quadratic_distance_model.predict([[rssi_eiger / TxPower_Eiger]])
        d_quad_model_jungfrau = quadratic_distance_model.predict([[rssi_jungfrau / TxPower_Jungfrau]])
        d_quad_model_breithorn = quadratic_distance_model.predict([[rssi_breithorn/TxPower_Breithorn]])


        #Update State Transition Matrix
        
        kf.F = np.array([[1,0,0,dt,0,0],
                        [0,1,0,0,dt,0],
                        [0,0,1,0,0,dt],
                        [0,0,0,1,0,0],
                        [0,0,0,0,1,0],
                        [0,0,0,0,0,1]])

        kf.predict()
        kf.update(state)

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
                    distance = distance-5
                if keyboard.is_pressed('f'):
                    distance = distance+5

                if  keyboard.is_pressed('s'):
                    with open(f'RSSI_Data/rssi_list_{angle}_interpolation_data_big_data_v2.txt', 'w') as f:
                        for item in RSSI_array:
                            f.write(f"{item}\n")
                

        if keyboard.is_pressed('a'):
                roll,pitch,yaw = 0,0,0     

        k = 1
        os.system('clear')
        
        print_dict(Sensor_Data)
        print('3D Position LIN: ', estimated_position_lin)
        print('3D Position EXP: ', estimated_position_exp)
        print('3D position Model', estimated_position_model)
 
        

        print('Distance Eiger Linear ',d_eiger_linear,'  Distance Jungfrau Linear: ', d_jungfrau_linear,'  Distance Breithorn Linear: ',d_breithorn_linear)
        print('Distance Eiger EXP:', d_Eiger,  '  Distance Jungfrau EXP: ', d_Jungfrau, '  Distance Breithorn EXP:', d_Breithorn)
        print('Distance Poly Model  Eiger: ',d_model_eiger, ' Jungfrau: ', d_model_jungfrau, '  Breithorn: ', d_model_breithorn)
        print(f'Quad nodel Distance: Eiger = {d_quad_model_eiger}, Jungfrau = {d_quad_model_jungfrau}, Breithorn = {d_quad_model_breithorn}')

       
        print('adjusted IMU Data', adjusted_imu_data)
        print('roll:',roll,' pitch: ', pitch,' yaw: ', yaw)
        print('KF State', state)


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
        # Print services
        
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
    'Enzian': ['2ABA', '3ABA','2CCC'],
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
devices = scanner.scan(timeout=2.0, passive = False)  # Scans for 10 seconds

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