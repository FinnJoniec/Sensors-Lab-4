from pymodbus.client import AsyncModbusTcpClient 
import RPi.GPIO as GPIO
import spidev as spidev

#import paho.mqtt.client as mqtt
from smbus2 import SMBus
import asyncio
import serial
import select
import time
from collections import deque


# LARGE TEAM
ACTIVITY_ADDRESS = 3
TEST_ADDRESS = 51

SMBus
# GROUP A - I2C
#bus = smbus2.SMBus(1)
I2C_ADDR = 0x08
bus= SMBus(1)
#sending PICO sensor data to:
OUTGOING_ADDRESS_A = 24
DATA_OUT_A = 0
#sending PICO activity bit:
ACTIVITY_ADDRESS_A = 24596
ACTIVITY_BIT_A = 0
#reading from to PICO:
INCOMING_ADDRESS_A = 1
DATA_IN_A = 0
#sending PICO's read data to:
READ_DATA_ADDRESS_A = 28



# GROUP B - Pulse-Width Encoding
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

# COMMS FOR GROUPB - UART
sar = serial.Serial(
	port = '/dev/ttyAMA5',        
#NameError: name 'READ_DATA_ADDRESS_B'
	baudrate=9600,
	timeout=1
	)
	

#PLC info
PLC_IP = "192.168.1.22"
PLC_PORT = 502



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
                                                                     
# Group C fixed pin variables
CLOCK_PIN = 18  
DATA_PIN  = 23   

GPIO.setmode(GPIO.BCM)
GPIO.setup(CLOCK_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(DATA_PIN,  GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.setwarnings(False)

def recieve_byte():
	value = 0
	for i in range(8):
		while GPIO.input(CLOCK_PIN) == 0:
			pass
		bit = GPIO.input(DATA_PIN)
		value = (value << 1) | bit
		while GPIO.input(CLOCK_PIN) == 1:
			pass
	return value

def send_byte(value):
	time.sleep(.001)
	GPIO.setup(DATA_PIN, GPIO.OUT)
	for i in range(7, -1, -1):
		while GPIO.input(CLOCK_PIN) == 0:
			pass  # wait for rising edge start
		GPIO.output(DATA_PIN, (value >> i) & 1)
		while GPIO.input(CLOCK_PIN) == 1:
			pass
	time.sleep(.001)
	GPIO.setup(DATA_PIN, GPIO.IN)


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
READ_DATA_ADDRESS_D = 51


#PLC info
PLC_IP = "192.168.1.22"
PLC_PORT = 502

# COMMS FOR GROUPD - UART
ser = serial.Serial(
	port = '/dev/serial0',        
#NameError: name 'READ_DATA_ADDRESS_D'
	baudrate=9600,
	timeout=1
	)

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



# MAIN LOOP
try:
	while True:
		
		# UPDATING GROUPB
		DATA_IN_B = asyncio.run(read_from_plc(INCOMING_ADDRESS_B))	# reading data from PLC
		asyncio.run(write_to_plc(READ_DATA_ADDRESS_B, int(DATA_IN_B)))   # sending data back to PLC
		#client.publish("pi/commands",int(DATA_IN_2)) # writing data to GROUP2
		
		# SCANNING GROUPB
		data = sar.readline().decode('utf-8').strip()
		parts = data.split(',')
		DATA_OUT_B = ''.join(c for c in parts[0] if c.isdigit()) if len(parts) > 0 else None
		ACTIVITY_BIT_B = ''.join(c for c in parts[1] if c.isdigit()) if len(parts) > 1 else None
		if len(DATA_OUT_B) > 0:
			DATA_OUT_B = int(float(DATA_OUT_B))
			ACTIVITY_BIT_B = int(ACTIVITY_BIT_B)
		else:
			DATA_OUT_B = 0
			ACTIVITY_BIT_B = 0
		print("Group B")
		print(f"angle: {DATA_OUT_B} activity bit:{ACTIVITY_BIT_B}")
		asyncio.run(write_to_plc(OUTGOING_ADDRESS_B, DATA_OUT_B)) # sending GROUP1 data to PLC
		asyncio.run(write_to_plc(ACTIVITY_ADDRESS_B, ACTIVITY_BIT_B)) # sending GROUP1 ac to PLC		
		
		#  Lab 4B Extension – Control Within Table (Random Pairing)
        # --------------------------------------------------------------

        # 1 Random partner selection every ~30 seconds
		if "pair_change_time" not in locals():
			pair_change_time = time.time()
			current_partner = "A"  # initial partner
		if time.time() - pair_change_time > 30:
			import random
			current_partner = random.choice(["A", "C", "D"])
			pair_change_time = time.time()
			print(f"\n[Lab4B] Now paired with Group {current_partner}")

        # 2 Map partner → address
		partner_address_map = {"A": OUTGOING_ADDRESS_A,
								"C": OUTGOING_ADDRESS_C,
                               "D": OUTGOING_ADDRESS_D}
		partner_address = partner_address_map[current_partner]

        # 3️ Read partner's potentiometer value from PLC
		partner_val = 25
        #asyncio.run(read_from_plc(partner_address))

        # 4 Send that value to our ESP32 via UART
        #    (so our servo mimics the partner’s potentiometer)
		if partner_val is not None:
			msg = f"{partner_val}\n"
			sar.write(msg.encode())
			print(f"[Lab4B] Sent partner value {partner_val} → ESP32")

        # 5 Scale to 0–5000 and write to our own Read Address for HMI display
		scaled_val = max(0, min(5000, int(partner_val * (5000 / 180))))
		asyncio.run(write_to_plc(READ_DATA_ADDRESS_B, scaled_val))

		time.sleep(.1)
		
		
		############################################################################################
		###      GROUP A      ######################################################################
		############################################################################################
		
		DATA_IN_A = asyncio.run(read_from_plc(INCOMING_ADDRESS_A))	# reading data from PLC
		asyncio.run(write_to_plc(READ_DATA_ADDRESS_A, int(DATA_IN_A)))   # sending data back to PLC
		# Scanning Group A
		data = ser.readline().decode('utf-8').strip()
		parts = data
		'''
		DATA_OUT_A = ''.join(c for c in parts[0] if c.isdigit()) if len(parts) > 0 else None
		ACTIVITY_BIT_A = ''.join(c for c in parts[1] if c.isdigit()) if len(parts) > 1 else None
		if len(DATA_OUT_A) > 0:
			DATA_OUT_A = int(float(DATA_OUT_A)for c in parts[1] if c.isdigit()) if len(parts) > 1 else None
			ACTIVITY_BIT_A = int(ACTIVITY_BIT_A)ocument
		else:
			DATA_OUT_A = 0
			ACTIVITY_BIT_A = 0
			'''
		asyncio.run(write_to_plc(OUTGOING_ADDRESS_A, DATA_OUT_A)) # sending GROUP1 data to PLC
		asyncio.run(write_to_plc(ACTIVITY_ADDRESS_A, ACTIVITY_BIT_A)) # sending GROUP1 ac to PLC
		
		data_to_send = 223
		bus.write_byte(I2C_ADDR, data_to_send)
		
		angle = bus.read_byte(I2C_ADDR)
		
		print(f"GROUP A | ANGLE IN: {angle} | OUTGOING DATA: {data_to_send}")
			
		############################################################################################
		############################################################################################
		############################################################################################
		
		
		#Group C
		DATA_IN_C = asyncio.run(read_from_plc(INCOMING_ADDRESS_C))	# reading data from PLC
		asyncio.run(write_to_plc(READ_DATA_ADDRESS_C, int(DATA_IN_C)))   # sending data back to PLC
		#Scanning Group C
		data = ser.readline().decode('utf-8').strip()
		parts = data.split(',')
		DATA_OUT_C = ''.join(c for c in parts[0] if c.isdigit()) if len(parts) > 0 else None
		ACTIVITY_BIT_C = ''.join(c for c in parts[1] if c.isdigit()) if len(parts) > 1 else None
		if len(DATA_OUT_C) > 0:
			DATA_OUT_C = int(float(DATA_OUT_C))
			ACTIVITY_BIT_C = int(ACTIVITY_BIT_C)
		else:
			DATA_OUT_C = 0
			ACTIVITY_BIT_C = 0
		angle = recieve_byte()
		
		# Prepare a response (e.g. angle + 5)
		response = (angle + 5) & 0xFF
		time.sleep(.001)
		send_byte(response)
		print(f"Group C Angle-> Recieved: {angle}, Sent {response}")
		asyncio.run(write_to_plc(OUTGOING_ADDRESS_C, DATA_OUT_C)) # sending GROUP1 data to PLC
		asyncio.run(write_to_plc(ACTIVITY_ADDRESS_C, ACTIVITY_BIT_C)) # sending GROUP1 ac to PLC		
		
		

	
	############################################################################################
		#Updating Group D
		DATA_IN_D = asyncio.run(read_from_plc(INCOMING_ADDRESS_D))	# reading data from PLC
		asyncio.run(write_to_plc(READ_DATA_ADDRESS_D, int(DATA_IN_D)))   # sending data back to PLC
		#Scanning Group D
		data = ser.readline().decode('utf-8').strip()
		parts = data.split(',')
		DATA_OUT_D = ''.join(c for c in parts[0] if c.isdigit()) if len(parts) > 0 else None
		ACTIVITY_BIT_D = ''.join(c for c in parts[1] if c.isdigit()) if len(parts) > 1 else None
		if len(DATA_OUT_D) > 0:
			DATA_OUT_D = int(float(DATA_OUT_D))
			ACTIVITY_BIT_D = int(ACTIVITY_BIT_D)
		else:
			DATA_OUT_D = 0
			ACTIVITY_BIT_D = 0
			
		print("Group D")
		print(f"data out 1 {DATA_OUT_D} ac out 1 {ACTIVITY_BIT_D}")
		asyncio.run(write_to_plc(OUTGOING_ADDRESS_D, DATA_OUT_D)) # sending GROUP1 data to PLC
		asyncio.run(write_to_plc(ACTIVITY_ADDRESS_D, ACTIVITY_BIT_D)) # sending GROUP1 ac to PLC		
		
		        # --------------------------------------------------------------
        #  Lab 4B Extension – Control Within Table (Random Pairing)
        # --------------------------------------------------------------

        # 1 Random partner selection every ~30 seconds
		if "pair_change_time" not in locals():
			pair_change_time = time.time()
			current_partner = "A"  # initial partner
		if time.time() - pair_change_time > 30:
			import random
			current_partner = random.choice(["A", "B", "C"])
			pair_change_time = time.time()
			print(f"\n[Lab4B] Now paired with Group {current_partner}")

        # 2 Map partner → address
		partner_address_map = {"A": OUTGOING_ADDRESS_A,
								"B": OUTGOING_ADDRESS_B,
                               "C": OUTGOING_ADDRESS_C}
		partner_address = partner_address_map[current_partner]

        # 3️ Read partner's potentiometer value from PLC
		partner_val = 50
        #asyncio.run(read_from_plc(partner_address))

        # 4 Send that value to our Pico via UART
        #    (so our servo mimics the partner’s potentiometer)
		if partner_val is not None:
			msg = f"{partner_val}\n"
			ser.write(msg.encode())
			print(f"[Lab4B] Sent partner value {partner_val} → Pico")

        # 5 Scale to 0–5000 and write to our own Read Address for HMI display
		scaled_val = max(0, min(5000, int(partner_val * (5000 / 180))))
		asyncio.run(write_to_plc(READ_DATA_ADDRESS_D, scaled_val))

		time.sleep(.1)
		
		
		'''
		 # Christmas Light Protocol
		for i in range(5):
			color = iocument
			asyncio.run(write_to_plc(READ_DATA_ADDRESS_D, color)) # sending GROUP1 data to PLC
            # Activity Bit Protocol
			if ACTIVITY_BIT_D == 0:
				ACTIVITY_BIT_D = 1
			else:
				ACTIVITY_BIT_D = 0
		   
			asyncio.run(write_to_plc(ACTIVITY_ADDRESS_D, ACTIVITY_BIT_D)) # sending GROUP1 data to PLC
			time.sleep(1)
		'''
       

except KeyboardInterrupt:
	print("Exiting...")

finally:
	try:
		ser.close()
		#ser2.close()
	except Exception:
		pass


'''
	for port in ports:
		port.close()

'''
