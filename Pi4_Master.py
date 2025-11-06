from pymodbus.client import AsyncModbusTcpClient 
import RPi.GPIO as GPIO
import spidev as spidev

#import paho.mqtt.client as mqtt
import smbus2
import asyncio
import serial
import select
import time
from collections import deque


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

#Group B comms
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1_000_000   # start at 1 MHz; adjust if needed
spi.mode = 0                   # MODE0
spi.bits_per_word = 8
# Frame format: [0xAA][pot_hi][pot_lo][angle][checksum]
FRAME_LEN = 5
HEADER = 0xAA
FRESH_SECS = 0.5
last_ok_ts = 0.0
latest_pot = 0
latest_angle = 0
def read_frame_from_esp32():
    """
    Master clocks 5 bytes from slave.
    Returns (ok, pot, angle)
    """
    try:
        rx = spi.xfer2([0x00] * FRAME_LEN)  # clock out 5 bytes
        if len(rx) != FRAME_LEN:
            return (False, 0, 0)
        if rx[0] != HEADER:
            return (False, 0, 0)
        pot_hi, pot_lo, angle, csum = rx[1], rx[2], rx[3], rx[4]
        calc = (pot_hi + pot_lo + angle) & 0xFF
        if calc != csum:
            return (False, 0, 0)
        pot = ((pot_hi << 8) | pot_lo) & 0xFFFF
        return (True, pot, angle)
    except Exception as e:
        # SPI bus error; keep calm, report once per loop
        # print(f"SPI error: {e}")
        return (False, 0, 0)


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

# Group C Comms
CLOCK_PIN = 18  # Pi BCM 18 (from ESP32 GPIO26)
DATA_PIN  = 23  # Pi BCM 23 (from ESP32 GPIO27)

GPIO.setmode(GPIO.BCM)
GPIO.setup(CLOCK_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(DATA_PIN,  GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def read_angle_from_esp32():
    """Read one 8-bit frame: sample DATA on each rising CLOCK."""
    bits = []
    while GPIO.input(CLOCK_PIN) == 1:
        pass  # start when clock is low
    while len(bits) < 8:
        GPIO.wait_for_edge(CLOCK_PIN, GPIO.RISING)
        bits.append(GPIO.input(DATA_PIN))
    val = 0
    for b in bits: val = (val << 1) | (1 if b else 0)
    return min(max(val, 0), 180)

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
PLC_IP = "192.168.1.22"
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

		print("Group B: SPI master (spidev) pulling frames from ESP32 slave. Ctrl+C to exit.")
		# Echo pattern (same as your other groups)
		DATA_IN_B = asyncio.run(read_from_plc(INCOMING_ADDRESS_B))
		asyncio.run(write_to_plc(READ_DATA_ADDRESS_B, int(DATA_IN_B)))
		ok, pot, angle = read_frame_from_esp32()
		now = time.time()
		if ok:
			latest_pot = pot
			latest_angle = angle
			last_ok_ts = now

			alive = 1 if (now - last_ok_ts) <= FRESH_SECS else 0
			DATA_OUT_B = int(latest_pot)   # send pot (0..4095) to PLC
			ACTIVITY_BIT_B = int(alive)

			# Send to PLC
			asyncio.run(write_to_plc(OUTGOING_ADDRESS_B, DATA_OUT_B))
			asyncio.run(write_to_plc(ACTIVITY_ADDRESS_B, ACTIVITY_BIT_B))

			# Debug
			print(f"[B] pot={DATA_OUT_B:4d} angle={latest_angle:3d} alive={ACTIVITY_BIT_B} ok={int(ok)}")

			time.sleep(0.05)  # ~20 Hz

		
		'''
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
		time.sleep(0.05)
		
		'''
		#Updating Group A
		DATA_IN_A = asyncio.run(read_from_plc(INCOMING_ADDRESS_A))	# reading data from PLC
		asyncio.run(write_to_plc(READ_DATA_ADDRESS_A, int(DATA_IN_A)))   # sending data back to PLC
		# Scanning Group A
		data = ser.readline().decode('utf-8').strip()
		parts = data
		DATA_OUT_A = ''.join(c for c in parts[0] if c.isdigit()) if len(parts) > 0 else None
		ACTIVITY_BIT_A = ''.join(c for c in parts[1] if c.isdigit()) if len(parts) > 1 else None
		if len(DATA_OUT_A) > 0:
			DATA_OUT_A = int(float(DATA_OUT_A))
			ACTIVITY_BIT_A = int(ACTIVITY_BIT_A)
		else:
			DATA_OUT_A = 0
			ACTIVITY_BIT_A = 0
		print(f"data out 1 {DATA_OUT_A} ac out 1 {ACTIVITY_BIT_A}")
		asyncio.run(write_to_plc(OUTGOING_ADDRESS_A, DATA_OUT_A)) # sending GROUP1 data to PLC
		asyncio.run(write_to_plc(ACTIVITY_ADDRESS_A, ACTIVITY_BIT_A)) # sending GROUP1 ac to PLC		
		time.sleep(0.05)
		'''
		#Updating Group D
		DATA_IN_B = asyncio.run(read_from_plc(INCOMING_ADDRESS_B))	# reading data from PLC
		asyncio.run(write_to_plc(READ_DATA_ADDRESS_B, int(DATA_IN_B)))   # sending data back to PLC
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
		print(f"data out 1 {DATA_OUT_D} ac out 1 {ACTIVITY_BIT_D}")
		asyncio.run(write_to_plc(OUTGOING_ADDRESS_D, DATA_OUT_D)) # sending GROUP1 data to PLC
		asyncio.run(write_to_plc(ACTIVITY_ADDRESS_D, ACTIVITY_BIT_D)) # sending GROUP1 ac to PLC		
		time.sleep(0.05)
		'''
except KeyboardInterrupt:
	print("Exiting...")

finally:
	try:
		ser.close()
		ser2.close()
	except Exception:
		pass
	try:
		spi.close()
	except Exception:
		pass

	'''
	for port in ports:
		port.close()
	'''
