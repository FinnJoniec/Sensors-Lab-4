from pymodbus.client import AsyncModbusTcpClient 
import RPi.GPIO as GPIO
import spidev as spidev
import random

#import paho.mqtt.client as mqtt
from smbus2 import SMBus, i2c_msg
import asyncio
import serial
import select
import time
from collections import deque
import re


# LARGE TEAM
ACTIVITY_ADDRESS = 3
TEST_ADDRESS = 51


# GROUP A - I2C
I2C_ADDR = 0x08
bus= SMBus(1)
OUTGOING_ADDRESS_A = 24
DATA_OUT_A = 0
ACTIVITY_ADDRESS_A = 24591
ACTIVITY_BIT_A = 0
INCOMING_ADDRESS_A = 1
DATA_IN_A = 0  
READ_DATA_ADDRESS_A = 28
parts_a = ""



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
parts_b = ""

# COMMS FOR GROUPB - UARTcio.r
sar = serial.Serial(port='/dev/ttyAMA5', baudrate=9600, timeout=0.05)
sar.reset_input_buffer()
       

	

# PLC info
PLC_IP = "192.168.1.22"
PLC_PORT = 502


# READING DATA FROM PLC

_ints = re.compile(r'(-?\d+)')
def parse_two_intsB(line, default=(0, 0)):
    nums = _ints.findall(line)
    if not nums:
        return default
    if len(nums) == 1:
        return (int(nums[0]), 0)
    return (int(nums[0]), int(nums[1]))



async def read_from_plcB(address):
    client = AsyncModbusTcpClient(PLC_IP, port=PLC_PORT)
    time.sleep(0.05)
    await client.connect()
    value = 0
    if client.connected:
        result = await client.read_holding_registers(address)
        if not result.isError():
            value = result.registers[0]
            if value:
                # print(f"data from port {address} = {value}")
                value = value
            else:
                # print(f"coil {address} is not active")
                value = value
        else:
            print(f"modbus error reading {address} : {value}")
            value = value
    else:
        print("Connection failed")

    client.close()
    return value


# WRITING DATA TO PLC
async def write_to_plcB(address, send_data):
    try:
        async with AsyncModbusTcpClient(PLC_IP, port=PLC_PORT) as client:
            result = await client.write_register(address, send_data)
            if result.isError():
                print(f"Error writing to register {address}: {result}")
            else:
                # print(f"Successfully wrote {send_data} to register {address}")
                #pass  # placeholder, nothing to do here
                i = 0
    except Exception as e:
        print(f"Unexpected error: {e}")

	



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
parts_c = ""
                                                                     
# Group C fixed pin variables
CLOCK_PIN = 18        # Pi BCM 18 (wire to ESP32 GPIO26)
DATA_PIN  = 23        # Pi BCM 23 (wire to ESP32 GPIO27)
SYNC_BYTE = 0xA5

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(CLOCK_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(DATA_PIN,  GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def _wait_level(pin, level, timeout=0.5):
    t0 = time.time()
    while GPIO.input(pin) != level:
        if time.time() - t0 > timeout:
            return False
    return True

def read_byte(timeout=0.5):
	val = 0
	for _ in range(8):
		if not _wait_level(CLOCK_PIN, 1, timeout):
			return None
		#while GPIO.input(CLOCK_PIN) == 0:
		#	pass
		bit = GPIO.input(DATA_PIN)
		val = (val << 1) | bit
		if not _wait_level(CLOCK_PIN, 0, timeout):
			return None
		#while GPIO.input(CLOCK_PIN) == 1:
		#	pass
	return val

def read_frame(timeout=2.0):
    """Find SYNC, then LEN, then payload bytes; returns list[int] or None."""
    t_dead = time.time() + timeout
    while time.time() < t_dead:
        b = read_byte(timeout=0.25)
        if b is None:
            continue
        if b == SYNC_BYTE:
            n = read_byte(timeout=0.25)
            if n is None:
                return None
            payload = []
            for _ in range(n):
                bx = read_byte(timeout=0.25)
                if bx is None:
                    return None
                payload.append(bx)
            return payload
    return None


def send_byte(value):
	"""
	Pi replies one byte while ESP32 supplies CLOCK for the reply.
	We: wait for rising, set DATA bit, hold until falling; repeat.
	"""
	#time.sleep(.005)
	GPIO.setup(DATA_PIN, GPIO.OUT, initial=GPIO.LOW)
	GPIO.output(DATA_PIN, 1)
	time.sleep(.0002)
	GPIO.output(DATA_PIN, 0)
	time.sleep(.0002)
	try:
		for i in range(7, -1, -1):
			#bit = (value >> i) & 1
			#GPIO.output(DATA_PIN, bit) 
			if not _wait_level(CLOCK_PIN, 1, 0.01):  # wait rising
				break
			GPIO.output(DATA_PIN, (value >> i) & 1)
			if not _wait_level(CLOCK_PIN, 0, 0.01):  # wait falling
				break
		time.sleep(.0005)
	finally:
		GPIO.setup(DATA_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


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
parts_d = ""


#PLC info
PLC_IP = "192.168.1.22"
PLC_PORT = 502

# COMMS FOR GROUPD - UART
ser = serial.Serial(
	port = '/dev/serial0',        
#NameError: name 'READ_DATA_ADDRESS_D'
	baudrate=9600,
	timeout=0.05
	)


async def read_from_plc(address):
	client = AsyncModbusTcpClient(PLC_IP,port=PLC_PORT)
	time.sleep(0.05)
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


def parse_two_ints(line, default=(0, 0)):
    nums = _ints.findall(line)
    if not nums:
        return default
    if len(nums) == 1:
        return (int(nums[0]), 0)
    return (int(nums[0]), int(nums[1]))


def group_a():
	global ACTIVITY_BIT_A, DATA_OUT_A, DATA_IN_A
	DATA_IN_A = asyncio.run(read_from_plc(INCOMING_ADDRESS_A))
	asyncio.run(write_to_plc(READ_DATA_ADDRESS_A, int(DATA_IN_A)))
	
	read = i2c_msg.read(I2C_ADDR, 8)
	bus.i2c_rdwr(read)
	raw = bytes(read)
	trimmed = bytes([b for b in raw if b not in (0x00, 0xFF)])
	data = trimmed.decode('utf-8').strip()
	parts = data.split(",")

	#DATA_OUT_A = int(''.join(c for c in parts[0] if c.isdigit())) if len(parts) > 0 else 0
	#ACTIVITY_BIT_A = int(''.join(c for c in parts[1] if c.isdigit())) if len(parts) > 1 else 0
	
	DATA_OUT_A = int(parts[0])
	ACTIVITY_BIT_A = int(parts[1])
	
	asyncio.run(write_to_plc(OUTGOING_ADDRESS_A, DATA_OUT_A))
	asyncio.run(write_to_plc(ACTIVITY_ADDRESS_A, ACTIVITY_BIT_A))


def group_b():
	global ACTIVITY_BIT_B, DATA_OUT_B, DATA_IN_B
	DATA_IN_B = asyncio.run(read_from_plcB(INCOMING_ADDRESS_B))
	asyncio.run(write_to_plcB(READ_DATA_ADDRESS_B, int(DATA_IN_B)))

	try:
		ser.readline().decode('utf-8').strip()
		lineB = sar.readline().decode(errors='ignore').strip()
	except Exception:
		lineB = ""
	
	DATA_OUT_B, ACTIVITY_BIT_B = parse_two_intsB(lineB, default=(0, 0))
	DATA_OUT_B = max(0, min(180, DATA_OUT_B))
	ACTIVITY_BIT_B = 1 
	_ints = re.compile(r'(-?\d+)') if ACTIVITY_BIT_B else 0  # keep negatives and multi-digits like 51

	# 3) Push our current values to PLC
	asyncio.run(write_to_plcB(OUTGOING_ADDRESS_B, DATA_OUT_B))
	asyncio.run(write_to_plcB(ACTIVITY_ADDRESS_B, ACTIVITY_ADDRESS_B))
	
	'''
	# 4) Lab 4B – Partner control every ~30 s (B follows A/C/D via PLC)
	if 'pair_change_time_b' not in locals():
		pair_change_time_b = time.time()
		current_partner_b = 'A'  # initial partner for B

	if time.time() - pair_change_time_b > 30:
		import random
		current_partner_b = random.choice(['A', 'C', 'D'])
		pair_change_time_b = time.time()
		print(f"\n[Lab4B] B now paired with Group {current_partner_b}")

	partner_addr_map_b = {
		'A': OUTGOING_ADDRESS_A,
		'C': OUTGOING_ADDRESS_C,
		'D': OUTGOING_ADDRESS_D,
	}
	partner_addr_b = partner_addr_map_b[current_partner_b]
	partner_val_b = asyncio.run(read_from_plcB(partner_addr_b))
	if partner_val_b is None:
		partner_val_b = 25

	# Send partner’s value to our ESP32 so its servo mirrors the partner
	sar.write(f"{partner_val_b}\n".encode())

	# Scale to 0–5000 for HMI
	scaled_b = max(0, min(5000, int((partner_val_b) * (5000 / 180))))
	asyncio.run(write_to_plcB(READ_DATA_ADDRESS_B, scaled_b))

	# small pacing so B does not starve other IO (keeps same timing you had here)
	'''
	time.sleep(0.1)

	
'''	
def group_c_read():
	global DATA_OUT_C
	payload = read_byte()
	if payload is None:
		return
	DATA_OUT_C = max(0, min(180, payload))
'''	
def group_c():
		global ACTIVITY_BIT_C, DATA_OUT_C, DATA_IN_C
		#payload = read_frame(timeout=2.0)
		payload = read_byte()
		if payload is None:
			ACTIVITY_BIT_C = 0
			return  # nothing received this tick
		#DATA_OUT_C = payload & 0xFF
		DATA_OUT_C = max(0, min(180, payload))
		ACTIVITY_BIT_C = 1
		# 2) REPLY IMMEDIATELY so the ESP32 sees it during its reply window
		#DATA_IN_C = (0) & 0xFF
		#reply_ok = send_byte(DATA_IN_C)
		#if not reply_ok:
			#print("Group C: send_byte failed or times out")
		#ok = send_byte(DATA_IN_C)

		#print(f"Group C Angle -> Received: {DATA_OUT_C}, Sent: {DATA_IN_C}, reply_ok={ok}")

		# 3) AFTER replying, do slower I/O (PLC + serial)
		try:
			asyncio.run(write_to_plc(OUTGOING_ADDRESS_C, int(DATA_OUT_C)))
			asyncio.run(write_to_plc(ACTIVITY_ADDRESS_C, int(ACTIVITY_BIT_C)))
		except Exception as e:
			print("write plc error:", e)
		try:
			# Read from PLC and mirror back
			DATA_IN_C = asyncio.run(read_from_plc(INCOMING_ADDRESS_C))
			DATA_IN_C = max(0, min(180, DATA_IN_C))
			asyncio.run(write_to_plc(READ_DATA_ADDRESS_C, int(DATA_IN_C)))
			# Optional serial scan (make sure 'ser' exists and has small timeout)
			'''
			try:
				line = ser.readline().decode('utf-8', errors='ignore').strip()
			except Exception:
				line = ""
			

			if line:
				parts = line.split(',')
				DATA_OUT_C = ''.join(c for c in parts[0] if c.isdigit()) if len(parts) > 0 else ""
				ACTIVITY_BIT_C  = ''.join(c for c in parts[1] if c.isdigit()) if len(parts) > 1 else ""
				if DATA_OUT_C== "":
					DATA_OUT_C = 0
					ACTIVITY_BIT_C  = 0
				else:
					DATA_OUT_C = int(float(DATA_OUT_C))
					ACTIVITY_BIT_C  = int(ACTIVITY_BIT_C) if ACTIVITY_BIT_C != "" else 0
			else:
				DATA_OUT_C = 0
				ACTIVITY_BIT_C = 0
			'''

		except Exception as e:
			print("read plc error", e)
		


def group_d():
	global ACTIVITY_BIT_D, DATA_OUT_D
	DATA_IN_D = asyncio.run(read_from_plc(INCOMING_ADDRESS_D))	# reading data from PLC
	asyncio.run(write_to_plc(READ_DATA_ADDRESS_D, int(DATA_IN_D)))   # sending data back to PLC

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
	
	asyncio.run(write_to_plc(OUTGOING_ADDRESS_D, DATA_OUT_D)) # sending GROUP1 data to PLC
	asyncio.run(write_to_plc(ACTIVITY_ADDRESS_D, ACTIVITY_BIT_D)) # sending GROUP1 ac to PLC

try:
	groups = ['A', 'B', 'C', 'D']
	assigned_group = random.choice(groups)
	last_assignment_time = time.time()
	while True:
		group_a()
		#group_b()
		group_c()
		#group_d()
		DATA_OUT_C = max(0, min(180, DATA_OUT_C))	
		outgoing_addresses = [OUTGOING_ADDRESS_A, OUTGOING_ADDRESS_B, OUTGOING_ADDRESS_C, OUTGOING_ADDRESS_D]
		if time.time() - last_assignment_time < 30:
			if assigned_group == 'A':
				DATA_IN_A = DATA_OUT_A
				DATA_IN_B = DATA_OUT_A
				DATA_IN_C = DATA_OUT_A
				DATA_IN_D = DATA_OUT_A
			elif assigned_group == 'B':
				DATA_IN_A = DATA_OUT_B
				DATA_IN_B = DATA_OUT_B
				DATA_IN_C = DATA_OUT_B
				DATA_IN_D = DATA_OUT_B
			elif assigned_group == 'C':
				DATA_IN_A = DATA_OUT_C
				DATA_IN_B = DATA_OUT_C
				DATA_IN_C = DATA_OUT_C
				DATA_IN_D = DATA_OUT_C
			elif assigned_group == 'D':
				DATA_IN_A = DATA_OUT_D
				DATA_IN_B = DATA_OUT_D
				DATA_IN_C = DATA_OUT_D
				DATA_IN_D = DATA_OUT_D
		else:
			assigned_group = random.choice(groups)
			last_assignment_time = time.time()
		print(DATA_IN_C)
		bus.write_byte(I2C_ADDR, DATA_IN_A) # Send to Group A
		#sar.write(f'{DATA_IN_B}\n'.encode()) # Send to Group B
		send_byte(DATA_IN_C) # Send to Group C
		ser.write(f'{DATA_IN_D}\n'.encode()) # Send to Group D
		
		# 5 Scale to 0–5000 and write to our own Read Address for HMI display
		# scaled_val = max(0, min(5000, int(partner_val * (5000 / 180))))
		# asyncio.run(write_to_plc(READ_DATA_ADDRESS_D, scaled_val))

		time.sleep(0.1)

		print(
			f"IN CONTROL: GROUP {assigned_group}\n",
			f"\rGROUP A | ACTIVITY BIT: {ACTIVITY_BIT_A} | ANGLE IN: {DATA_IN_A} | ANGLE OUT: {DATA_OUT_A}\n",
			f"\rGROUP B | ACTIVITY BIT: {ACTIVITY_BIT_B} | ANGLE IN: {DATA_IN_B} | ANGLE OUT: {DATA_OUT_B}\n",
			f"\rGROUP C | ACTIVITY BIT: {ACTIVITY_BIT_C} | ANGLE IN: {DATA_IN_C} | ANGLE OUT: {DATA_OUT_C}\n",
			f"\rGROUP D | ACTIVITY BIT: {ACTIVITY_BIT_D} | ANGLE IN: {DATA_IN_D} | ANGLE OUT: {DATA_OUT_D}\n"
		)
		
		time.sleep(0.25)


except KeyboardInterrupt:
	print("Exiting...")
	quit()
	
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
