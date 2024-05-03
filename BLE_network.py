import asyncio
from bleak import BleakScanner
from bleak import BleakClient
from bleak import  BleakError



async def scan_ibeacons():
    scanner = BleakScanner()
    devices = await scanner.discover()

   
    ibeacon_devices = [device for device in devices if device.name and (device.name.startswith("XIAO_ESP_iBeacon") or device.name.startswith("Blue Origin"))  ]
    ibeacon_devices = [device for device in devices if device.name and  device.name.startswith("Blue Origin") ]
    print(ibeacon_devices)

        
    await asyncio.gather(*[connect_and_enable_notifications(device) for device in ibeacon_devices])
 



async def connect_and_enable_notifications(device, attempt = 1):
        async def notification_handler(sender, data):
            print("Notification received from:" + device.name, data)
            

        try:
            async with BleakClient(device.address,timeout = 5) as client:
                    
                    UUID = "00002a58-0000-1000-8000-00805f9b34fb"
                    # Enable notifications on the characteristic 
                    if device.name == "XIAO_ESP_iBeacon_Nautilus":
                            UUID = '0000AAA1-0000-1000-8000-00805F9B34FB'
                    if device.name == "XIAO_ESP_iBeacon_Taurus":
                            UUID = '0000B7BB-0000-1000-8000-00805F9B34FB'
                   
                    if device.name == "Blue Origin":
                            services = await client.get_services()
                            print(f"Characteristics for {device.name}:")
                            for service in services:
                                print(f'service: {service.uuid}')
                                for characteristic in service.characteristics:
                                    print(characteristic.uuid)
                                    UUID = '00002CCC-0000-1000-8000-00805F9B34FB'
                                    
                    try:
                     await client.start_notify(UUID, notification_handler)  
                    except BleakError as notify_error :
                        print(f"Failed to start notifications for {device.address}: {notify_error}")
                        await connect_and_enable_notifications(device, attempt = 1)
                # Wait for a while to receive notifications
                    while True:
                        
                        await asyncio.sleep(1)


        except BleakError as e:
            print(f"Failed to connect to {device.name}: {e}")
            if attempt < 3:
                await asyncio.sleep(2)
                print(f'attempting reconnection for {device.name}....')
                #asyncio.create_task(connect_and_enable_notifications(device, attempt = 1))
                await connect_and_enable_notifications(device, attempt = 1)
            else:
                print(f"Max attempts reached for {device.name}.")
        except asyncio.TimeoutError:
                print(f"Timeout error occurred while connecting to {device.name}. Retrying...")
                if attempt < 3:
                    await asyncio.sleep(2)
                    await connect_and_enable_notifications(device, attempt = 1)
                else:
                    print(f"Max attempts reached for {device.name}.")


try:
    asyncio.run(scan_ibeacons())
except KeyboardInterrupt:
    print("Program terminated by user.")
