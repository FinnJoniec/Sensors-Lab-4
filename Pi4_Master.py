from pymodbus.client import AsyncModbusTcpClient
#import paho.mqtt.client as mqtt
import smbus2
import asyncio
import serial
import select
import time


# LARGE TEAM
ACTIVITY_ADDRESS = 3
TEST_ADDRESS = 51


# GROUP A - I2C
#bus = smbus2.SMBus(1)
PICO_SLAVE_ADDRESS_A = 0x08
#example code
#bus.write_byte(PICO_SLAVE_ADDRESS_A, 0x12)
#data = bus.read_byte(PICO_SLAVE_ADDRESS_A)
#print(f"Got data: {data}")
#sending PICO sensor data to:
OUTGOING_ADDRESS_A = 24
DATA_OUT_A = 0
#sending PICO activity bit:
ACTIVITY_ADDRESS_A = 24591
ACTIVITY_BIT_A = 0
#reading from to PICO:
INCOMING_ADDRESS_A = 1
DATA_IN_A = 0
#sending PICO's read data to:
READ_DATA_ADDRESS_A = 28
'''___'''

# GROUP B - SPI
#sending PICO sensor data to:
OUTGOING_ADDRESS_B = 25
DATA_OUT_B = 0
#sending ESP32's activity bit:
ACTIVITY_ADDRESS_B = 24592
ACTIVITY_BIT_B = 0
#reading from to ESP32:
INCOMING_ADDRESS_B = 0
DATA_IN_B = 0
#sending PICO's read data to:
READ_DATA_ADDRESS_B = 29
'''___'''

# GROUP C - Custom GPIO Clock/Data
#sending PICO sensor data to:
OUTGOING_ADDRESS_C = 26
DATA_OUT_C = 0
#sending PICO activity bit:
ACTIVITY_ADDRESS_C = 24593
ACTIVITY_BIT_C = 0
#reading from to PICO:
INCOMING_ADDRESS_C = 1
DATA_IN_C = 0
#sending PICO's read data to:
READ_DATA_ADDRESS_C = 30
'''___'''

# GROUP D - UART
#sending PICO sensor data to:
OUTGOING_ADDRESS_D = 27
DATA_OUT_D = 0
#sending PICO activity bit:
ACTIVITY_ADDRESS_D = 24594
ACTIVITY_BIT_D = 0
#reading from to PICO:
INCOMING_ADDRESS_D = 1
DATA_IN_D = 0
#sending PICO's read data to:
READ_DATA_ADDRESS_D = 31
'''___'''

#PLC info
PLC_IP = "192.168.1.10"
PLC_PORT = 502

# COMMS FOR GROUPD - UART
ser = serial.Serial(
	port = '/dev/serial0',        
#NameError: name 'READ_DATA_ADDRESS_D'
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
"""
client = mqtt.Client("PiSubscriber")
client.on_connect = on_connect
client.on_message = on_message
client.connect("localhost",1883,60)
client.loop_start()
"""

# MAIN LOOP
try:
	while True:
		# UPDATING GROUPB
		DATA_IN_B = asyncio.run(read_from_plc(INCOMING_ADDRESS_B))	# reading data from PLC
		asyncio.run(write_to_plc(READ_DATA_ADDRESS_B, int(DATA_IN_B)))   # sending data back to PLC
		#client.publish("pi/commands",int(DATA_IN_2)) # writing data to GROUP2
		
		# SCANNING GROUPB
		data = ser.readline().decode('utf-8').strip()
		parts = data.split(',')
		DATA_OUT_B = ''.join(c for c in parts[0] if c.isdigit()) if len(parts) > 0 else None
		ACTIVITY_BIT_B = ''.join(c for c in parts[1] if c.isdigit()) if len(parts) > 1 else None
		if len(DATA_OUT_B) > 0: 
			DATA_OUT_B = int(float(DATA_OUT_B))
			ACTIVITY_BIT_B = int(ACTIVITY_BIT_B)
		else:
			DATA_OUT_B = 0
			ACTIVITY_BIT_B = 0
		print(f"data out 1 {DATA_OUT_B} ac out 1 {ACTIVITY_BIT_B}")
		asyncio.run(write_to_plc(OUTGOING_ADDRESS_B, DATA_OUT_B)) # sending GROUP1 data to PLC
		asyncio.run(write_to_plc(ACTIVITY_ADDRESS_B, ACTIVITY_BIT_B)) # sending GROUP1 ac to PLC		
		'''# UPDATING GROUP1
		DATA_IN_B = asyncio.run(read_from_plc(INCOMING_ADDRESS_B))	# reading data from PLC
		asyncio.run(write_to_plc(READ_DATA_ADDRESS_1, DATA_IN_1)) # sending data back to PLC
		ser.write(str(DATA_IN_1).encode())	# writing data to GROUP1
		'''
except KeyboardInterrupt:
	print("Exiting...")

finally:
	ser.close()
	'''
	for port in ports:
		port.close()
	'''
