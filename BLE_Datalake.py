from bluepy.btle import Scanner, Peripheral, ADDR_TYPE_PUBLIC, DefaultDelegate
import threading
import time

# Custom delegate class to handle notifications
class NotificationDelegate(DefaultDelegate):
    def __init__(self, uuids):
        DefaultDelegate.__init__(self)
        self.time = time.time()
        self.number_of_uuids = uuids
        self.count = 1

    def handleNotification(self, cHandle, data):
        # Handle received notification data here
       
        decoded_data = data.decode('utf-8')
        
        if time.time() - self.time > 2:
            print("Notification received on handle {}: {}".format(cHandle, decoded_data))
            self.count +=1
            if self.count > self.number_of_uuids:
                self.time = time.time()
                self.count = 1
    
       

def print_services_and_characteristics(device, uuids):
    print("Services and Characteristics of Device {}:".format(device.addr))
    connected = False
    peripheral = None
    
    while not connected:
        try:
            # Attempt to connect to the device
            peripheral = Peripheral(device.addr, device.addrType)
            connected = True
        except Exception as e:
       
            print("Error connecting to device:", e)
            print(f'Retrying connection to drvice {device.getValueText(9)} in 5 seconds...')
            time.sleep(1)  # Wait for 5 seconds before retrying
    

    try:
        # Print services
        notification_delegate = NotificationDelegate(len(uuids))

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
                           
                            
                            peripheral.withDelegate(notification_delegate)
                            peripheral.writeCharacteristic(char.getHandle() + 1, b"\x01\x00", withResponse=True)
            
        while True:
                if peripheral.waitForNotifications(1.0):
                             
                    continue

    except Exception as e:
        print("Error:", e)
        print('Reconnecting...')
        print_services_and_characteristics(device, uuids)
        

device_names = ['Enzian','iBeacon Eiger', 'iBeacon Jungfrau', 'iBeacon Breithorn']
device_names = ['Enzian', 'iBeacon Eiger']
#device_names = ['iBeacon Jungfrau']
uuids = {
    'Enzian': ['2ABA','2CCC','2DDD', '2BBB'],
    #'Enzian':['0815'],
    'iBeacon Eiger': ['2A59'],
    'iBeacon Jungfrau': ['2A57'],
    'iBeacon Breithorn': ['2A58']}

threads = []

# Create a scanner object
scanner = Scanner()

# Scan for devices (you can specify a timeout in seconds)
devices = scanner.scan(timeout=20, passive = False)  # Scans for 10 seconds



# Iterate through the scanned devices
for dev in devices:
    # Access device data

    if dev.getValueText(9) is not None and  dev.getValueText(9) in device_names:
        print("  Device Name:", dev.getValueText(9)) 
        name = dev.getValueText(9)
        
        uuid = uuids[name]
        print(name, uuid)
        threading.Thread(target = print_services_and_characteristics, args =  (dev, uuid,)).start()
     
    





#Docu
'''
Start VsCode in Superuser mode: sudo code --no-sandbox --user-data-dir=/home/nicolas/VSCODE/.vscode_root

in case of error 13 (Issues with scanning): sudo service bluetooth restart

'''
