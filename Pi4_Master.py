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


'''
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



GPIO.setwarnings(False)
#Group B pin variables
RX_PIN = 17   # Pi input (from ESP32)
TX_PIN = 25   # Pi output (to ESP32)

GPIO.setmode(GPIO.BCM)
GPIO.setup(RX_PIN, GPIO.IN)
GPIO.setup(TX_PIN, GPIO.OUT)


def measure_pulse():
    GPIO.wait_for_edge(RX_PIN, GPIO.RISING)
    start = time.time()
    GPIO.wait_for_edge(RX_PIN, GPIO.FALLING)
    width = (time.time() - start) * 1_000_000  # µs
    return width

try:
    while True:
        width = measure_pulse()
        if 400 < width < 2600:
            angle = int((width - 500) * 180 / 2000)
            print(f"Received pulse: {width:.1f} µs → {angle}°")

            # echo pulse back to ESP32
            pulse_out = 500 + (angle / 180) * 2000
            GPIO.output(TX_PIN, GPIO.HIGH)
            time.sleep(pulse_out / 1_000_000.0)
            GPIO.output(TX_PIN, GPIO.LOW)

        time.sleep(0.02)

except KeyboardInterrupt:
    GPIO.cleanup()

'''

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
CLOCK_PIN = 18   # from ESP32 GPIO26
DATA_PIN  = 23   # from ESP32 GPIO27
SERVO_PIN = 12   # servo control signal (PWM) — adjust if you like

SYNC_BYTE = 0xA5
LEN_BYTE  = 0x01

GPIO.setmode(GPIO.BCM)
GPIO.setup(CLOCK_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(DATA_PIN,  GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setwarnings(False)
# --- Servo setup: 50 Hz ---
servo = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz
servo.start(0)

def angle_to_duty(angle_deg):
    # Typical: 0deg≈0.5ms, 180deg≈2.5ms at 50Hz (20ms period)
    # duty% = (pulse_ms / 20ms) * 100  => 0.5ms->2.5%, 2.5ms->12.5%
    pulse_ms = 0.5 + (2.0 * (angle_deg / 180.0))
    return (pulse_ms / 20.0) * 100.0

def wait_rising(timeout_s=0.05):
    # Polling wait for rising edge to be robust vs RPi.GPIO wait_for_edge timing
    end = time.time() + timeout_s
    last = GPIO.input(CLOCK_PIN)
    while time.time() < end:
        cur = GPIO.input(CLOCK_PIN)
        if last == 0 and cur == 1:
            return True
        last = cur
    return False

def wait_falling(timeout_s=0.05):
    end = time.time() + timeout_s
    last = GPIO.input(CLOCK_PIN)
    while time.time() < end:
        cur = GPIO.input(CLOCK_PIN)
        if last == 1 and cur == 0:
            return True
        last = cur
    return False

def read_bit(timeout_s=0.05):
    # Sample DATA on rising edge, per your ESP32 sender
    if not wait_rising(timeout_s):
        return None
    bit = GPIO.input(DATA_PIN)
    # ensure we consume this clock and be ready for next edge
    if not wait_falling(timeout_s):
        return None
    return bit

def read_byte():
    val = 0
    for i in range(8):
        b = read_bit()
        if b is None:
            return None
        val = (val << 1) | (1 if b else 0)
    return val

def wait_frame_gap(min_low_time=0.001, overall_timeout=0.2):
    """
    Wait for the inter-frame low gap on CLOCK (~FRAME_GAP_US).
    We detect CLOCK staying low for at least min_low_time seconds.
    """
    t_end = time.time() + overall_timeout
    while time.time() < t_end:
        # Wait for clock to go low
        if GPIO.input(CLOCK_PIN) == 0:
            t0 = time.time()
            # stay low long enough?
            while GPIO.input(CLOCK_PIN) == 0:
                if (time.time() - t0) >= min_low_time:
                    return True
                # short sleep to reduce CPU
                time.sleep(0.00005)
        time.sleep(0.00005)
    return False

def read_frame():
    # 1) Wait for a noticeable low gap between frames
    if not wait_frame_gap(min_low_time=0.0015):   # ~1.5ms to match FRAME_GAP_US
        return None

    # 2) Read 4 bytes MSB-first on rising edges
    sync = read_byte()
    if sync is None:
        return None
    length = read_byte()
    if length is None:
        return None
    angle = read_byte()
    if angle is None:
        return None
    chk = read_byte()
    if chk is None:
        return None

    # 3) Validate
    if sync != SYNC_BYTE:
        return None
    if length != LEN_BYTE:
        return None
    if chk != (SYNC_BYTE ^ LEN_BYTE ^ angle):
        return None

    return angle

def set_servo(angle):
    angle = max(0, min(180, angle))
    servo.ChangeDutyCycle(angle_to_duty(angle))

try:
    print("Reading frames... (Ctrl+C to quit)")
    last_valid = None
    while True:
        angle = read_frame()
        if angle is not None:
            if angle != last_valid:
                print(f"Angle: {angle}")
                set_servo(angle)
                last_valid = angle
        else:
            # No valid frame this loop; tiny sleep so we don't spin too hard
            time.sleep(0.001)

except KeyboardInterrupt:
    pass
finally:
    servo.stop()
    GPIO.cleanup()


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

ser2 = serial.Serial(
	port = '/dev/ttyAMA3',
	baudrate=9600,
	timeout=1
)

'''
# COMMS FOR GROUP2 - MQTT

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
	'''
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
		'''
		
		
		'''
		try:
			angle = bus.read_byte(I2C_ADDR)
			print("ANGLE:", angle)
		except Exception as e:
			print(f"Error {e}")
		time.sleep(0.05)
		
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
		print(f"data out 1 {DATA_OUT_C} ac out 1 {ACTIVITY_BIT_C}")
		asyncio.run(write_to_plc(OUTGOING_ADDRESS_C, DATA_OUT_C)) # sending GROUP1 data to PLC
		asyncio.run(write_to_plc(ACTIVITY_ADDRESS_C, ACTIVITY_BIT_C)) # sending GROUP1 ac to PLC		
		time.sleep(0.05)

		try:
			while True:
				angle = read_frame()
				if angle is not None:
					print(f"ANGLE={angle}")

		except Exception as e:
			print(f"Error {e}")
			
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
		print(f"data out 1 {DATA_OUT_D} ac out 1 {ACTIVITY_BIT_D}")
		asyncio.run(write_to_plc(OUTGOING_ADDRESS_D, DATA_OUT_D)) # sending GROUP1 data to PLC
		asyncio.run(write_to_plc(ACTIVITY_ADDRESS_D, ACTIVITY_BIT_D)) # sending GROUP1 ac to PLC		
		time.sleep(0.05)
		
		'''
		 # Christmas Light Protocol
		for i in range(5):
			color = i
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
