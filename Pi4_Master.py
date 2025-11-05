from pymodbus.client import AsyncModbusTcpClient
#import paho.mqtt.client as mqtt
import smbus2
import asyncio
import serial
import select
import time

# GROUP 1
#sending PICO sensor data to:
OUTGOING_ADDRESS_1 = 24
DATA_OUT_1 = 0
#sending PICO activity bit:
ACTIVITY_ADDRESS_1 = 24592
ACTIVITY_BIT_1 = 0
#reading from to PICO:
INCOMING_ADDRESS_1 = 1
DATA_IN_1 = 0
#sending PICO's read data to:
READ_DATA_ADDRESS_1 = 28
'''___'''

# GROUP 2
#sending PICO sensor data to:
OUTGOING_ADDRESS_2 = 25
DATA_OUT_2 = 0
#sending ESP32's activity bit:
ACTIVITY_ADDRESS_2 = 24593
ACTIVITY_BIT_2 = 0
#reading from to ESP32:
INCOMING_ADDRESS_2 = 0
DATA_IN_2 = 0
#sending PICO's read data to:
READ_DATA_ADDRESS_2 = 29
'''___'''

# GROUP 3 
#sending PICO sensor data to:
OUTGOING_ADDRESS_1 = 26
DATA_OUT_1 = 0
#sending PICO activity bit:
ACTIVITY_ADDRESS_1 = 24594
ACTIVITY_BIT_1 = 0
#reading from to PICO:
INCOMING_ADDRESS_1 = 1
DATA_IN_1 = 0
#sending PICO's read data to:
READ_DATA_ADDRESS_1 = 30
'''___'''

# GROUP 4
#sending PICO sensor data to:
OUTGOING_ADDRESS_1 = 27
DATA_OUT_1 = 0
#sending PICO activity bit:
ACTIVITY_ADDRESS_1 = 24595
ACTIVITY_BIT_1 = 0
#reading from to PICO:
INCOMING_ADDRESS_1 = 1
DATA_IN_1 = 0
#sending PICO's read data to:
READ_DATA_ADDRESS_1 = 31
'''___'''

#PLC info
PLC_IP = "192.168.1.10"
PLC_PORT = 502

# COMMS FOR GROUP1 - UART
ser = serial.Serial(
	port = '/dev/serial0',
	baudrate=9600,
	timeout=1
	)

ser2 = serial.Serial(
	port = '/dev/ttyAMA3',
	baudrate=9600,
	timeout=1
)

# COMMS FOR GROUP2 - MQTT
"""
def on_connect(client,user_data,flags,rc):
	print(f"Connected with result code {rc}")
	client.subscribe("test/topic")

		
def on_message(client,user_data,msg):
	try:
		# SCANNING GROUP2
		payload = msg.payload.decode() # payload from ESP32
		parts = payload.split(",") # splitting payload into data and ac bit 
		if len(parts) == 2:
			DATA_OUT_2 = parts[0] # data
			ACTIVITY_BIT_2 = parts[1] # ac 
			DATA_OUT_2 = int(float(DATA_OUT_2))
			ACTIVITY_BIT_2 = int(float(ACTIVITY_BIT_2))
			#print(f"data out 2 {DATA_OUT_2} ac out 2 {ACTIVITY_BIT_2}")
			try:
				asyncio.run(write_to_plc(OUTGOING_ADDRESS_2, DATA_OUT_2)) # sending GROUP2 data to PLC
				asyncio.run(write_to_plc(ACTIVITY_ADDRESS_2, ACTIVITY_BIT_2)) # sending GROUP2 ac to PLC
			except Error as e:
				pass
		else:
			DATA_OUT_2 = 0
			
	except ValueError:
		pass
"""

# READING DATA FROM PLC
async def read_from_plc(address):
	client = AsyncModbusTcpClient(PLC_IP,port=PLC_PORT)
	time.sleep(0.02)
	await client.connect()
	value = 0
	if client.connected:
		result = await client.read_holding_registers(address)
		if not result.isError():
			value = result.registers[0]
			if value:
				#print(f"data from port {address} = {value}")
				value = value
			else:
				#print(f"coil {address} is not active")
				value = value
		
		else:
			print(f"modbus error reading {address} : {value}")
			value = value
	else:
		print("Connection failed")
	
	client.close()
	return value

# WRITING DATA TO PLC
async def write_to_plc(address, send_data):
	try:
		async with AsyncModbusTcpClient(PLC_IP,port=PLC_PORT) as client:
			result = await client.write_register(address,send_data)
			
			if result.isError():
				print(f"Error writing to coil {address} : {result}")
			else:
				#print(f"Successfully wrote {send_data} to coil {address}")
				i = 0

	except Exception as e:
		print(f"Unexpected eror {e}")

# STARTING CONNECTIONS FOR MQTT
client = mqtt.Client("PiSubscriber")
client.on_connect = on_connect
client.on_message = on_message
client.connect("localhost",1883,60)
client.loop_start()

# MAIN LOOP
try:
	while True:
		# UPDATING GROUP2
		DATA_IN_2 = asyncio.run(read_from_plc(INCOMING_ADDRESS_2))	# reading data from PLC
		asyncio.run(write_to_plc(READ_DATA_ADDRESS_2, int(DATA_IN_2)))   # sending data back to PLC
		client.publish("pi/commands",int(DATA_IN_2)) # writing data to GROUP2
		
		# SCANNING GROUP1
		data = ser.readline().decode('utf-8').strip()
		parts = data.split(',')
		DATA_OUT_1 = ''.join(c for c in parts[0] if c.isdigit()) if len(parts) > 0 else None
		ACTIVITY_BIT_1 = ''.join(c for c in parts[1] if c.isdigit()) if len(parts) > 1 else None
		if len(DATA_OUT_1) > 0: 
			DATA_OUT_1 = int(float(DATA_OUT_1))
			ACTIVITY_BIT_1 = int(ACTIVITY_BIT_1)
		else:
			DATA_OUT_1 = 0
			ACTIVITY_BIT_1 = 0
		print(f"data out 1 {DATA_OUT_1} ac out 1 {ACTIVITY_BIT_1}")
		asyncio.run(write_to_plc(OUTGOING_ADDRESS_1, DATA_OUT_1)) # sending GROUP1 data to PLC
		asyncio.run(write_to_plc(ACTIVITY_ADDRESS_1, ACTIVITY_BIT_1)) # sending GROUP1 ac to PLC		
		# UPDATING GROUP1
		DATA_IN_1 = asyncio.run(read_from_plc(INCOMING_ADDRESS_1))	# reading data from PLC
		asyncio.run(write_to_plc(READ_DATA_ADDRESS_1, DATA_IN_1)) # sending data back to PLC
		ser.write(str(DATA_IN_1).encode())	# writing data to GROUP1
		
except KeyboardInterrupt:
	print("Exiting...")

finally:
	ser.close()
	'''
	for port in ports:
		port.close()
	'''
