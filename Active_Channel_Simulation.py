from pymodbus.client import AsyncModbusTcpClient
import asyncio
import time
import random

# =========================================================
# PLC / NETWORK CONFIG
# =========================================================
PLC_IP = "192.168.1.22"
PLC_PORT = 502

# =========================================================
# ADDRESS DEFINITIONS
# =========================================================
INDICATOR_ADDRESS_LARGE_TABLE = 51

# GROUP A
OUTGOING_ADDRESS_A = 24      # PICO sensor data to PLC
ACTIVITY_ADDRESS_A = 24591   # PICO activity bit
READ_DATA_ADDRESS_A = 28     # PICO read-data display

# GROUP B
OUTGOING_ADDRESS_B = 25
ACTIVITY_ADDRESS_B = 24592
READ_DATA_ADDRESS_B = 29

# GROUP C
OUTGOING_ADDRESS_C = 26
ACTIVITY_ADDRESS_C = 24593
READ_DATA_ADDRESS_C = 30

# GROUP D
OUTGOING_ADDRESS_D = 27
ACTIVITY_ADDRESS_D = 24594
READ_DATA_ADDRESS_D = 31

# =========================================================
# MODBUS FUNCTIONS
# =========================================================
async def write_to_plc(address, send_data):
    """Writes a single value to a PLC Modbus register."""
    try:
        async with AsyncModbusTcpClient(PLC_IP, port=PLC_PORT) as client:
            result = await client.write_register(address, send_data)
            if result.isError():
                print(f"❌ Error writing to address {address} : {result}")
    except Exception as e:
        print(f"Unexpected error writing to {address}: {e}")

# =========================================================
# MAIN SIMULATION LOOP
# =========================================================
if __name__ == "__main__":
    print("=== Starting Active Channel Simulation ===")

    ACTIVITY_BIT_A = 0

    try:
        while True:
            # Toggle Activity Bit A
            ACTIVITY_BIT_A = 1 - ACTIVITY_BIT_A
            asyncio.run(write_to_plc(ACTIVITY_ADDRESS_A, ACTIVITY_BIT_A))

            # --- Generate simulated data (0–5000) ---
            DATA_OUT_A = random.randint(0, 5000)
            READ_DATA_A = random.randint(0, 5000)

            DATA_OUT_B = random.randint(0, 5000)
            READ_DATA_B = random.randint(0, 5000)

            DATA_OUT_C = random.randint(0, 5000)
            READ_DATA_C = random.randint(0, 5000)

            DATA_OUT_D = random.randint(0, 5000)
            READ_DATA_D = random.randint(0, 5000)

            # --- Send simulated data to PLC ---
            asyncio.run(write_to_plc(OUTGOING_ADDRESS_A, DATA_OUT_A))
            asyncio.run(write_to_plc(READ_DATA_ADDRESS_A, READ_DATA_A))

            asyncio.run(write_to_plc(OUTGOING_ADDRESS_B, DATA_OUT_B))
            asyncio.run(write_to_plc(READ_DATA_ADDRESS_B, READ_DATA_B))

            asyncio.run(write_to_plc(OUTGOING_ADDRESS_C, DATA_OUT_C))
            asyncio.run(write_to_plc(READ_DATA_ADDRESS_C, READ_DATA_C))

            asyncio.run(write_to_plc(OUTGOING_ADDRESS_D, DATA_OUT_D))
            asyncio.run(write_to_plc(READ_DATA_ADDRESS_D, READ_DATA_D))

            # Optional: indicator / color cycling
            color = random.randint(0, 4)
            asyncio.run(write_to_plc(INDICATOR_ADDRESS_LARGE_TABLE, color))

            # --- Console feedback ---
            print(
                f"A: {DATA_OUT_A}/{READ_DATA_A} | "
                f"B: {DATA_OUT_B}/{READ_DATA_B} | "
                f"C: {DATA_OUT_C}/{READ_DATA_C} | "
                f"D: {DATA_OUT_D}/{READ_DATA_D}"
            )

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nExiting simulation...")

    finally:
        print("Simulation stopped.")
