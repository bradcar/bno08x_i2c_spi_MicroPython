# Linear_acceleration buffer logging BNO08x MicroPython SPI Test
#
#
# Reading Linear_Acc for 1000 rows, doing nothing with output
# sensor timestamps last_sensor_ms=5724.8 first_sensor_ms=644.9  sensor duration: 5.1 s
# Sensor msec/Lin_Acc = 5.08 ms
# Clock msec/Lin_Acc  = 5.06 ms
# 
# Writing data in sector chunks to flash in 4 KiB sectors with flush no CRC
# Array each result for 1000 rows:
# sensor timestamps last_sensor_ms=11931.5 first_sensor_ms=5724.8  sensor duration: 6.2 s
# Sensor msec/reports = 6.21 ms
# Clock msec/reports  = 6.28 ms
# BYTES_PER_ROW=44, data size = 44000 bytes
# Array = 43.0 KiB, xfer = 6.8 KiB/s
#
# Writing data in sector chunks to flash in 4 KiB sectors with flush WITH CRC
# Array each result for 1000 rows:
# sensor timestamps last_sensor_ms=11997.0 first_sensor_ms=5785.0  sensor duration: 6.2 s
# Sensor msec/reports = 6.21 ms
# Clock msec/reports  = 6.29 ms
# BYTES_PER_ROW=44, data size = 44000 bytes
# Array = 43.0 KiB, xfer = 6.8 KiB/s
#

from time import sleep
from utime import sleep_ms, ticks_ms, sleep_us

from bno08x import *

from machine import SPI, Pin
from spi import BNO08X_SPI

import struct
import os
import binascii # For fast CRC32

int_pin = Pin(14, Pin.IN)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# miso=Pin(16) - BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)
# sck=Pin(18)  - BNO SCK 
# mosi=Pin(19) - BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # BNO WAK

spi = SPI(0, baudrate=3000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin, debug=False)

print(spi) # baudrate=3000000 required
print("Start")
print("====================================\n")

bno.linear_acceleration.enable(200)
bno.gyro.enable(200)
bno.quaternion.enable(200)

bno.print_report_period()
print("BNO08x sensors enabled\n")


# Initialize flashbuffers and file
filename = "sensor_log.bin"
# Create/Clear the file
with open(filename, "wb") as f:
    pass


# Storing results in memory at each update
ROWS = 1000
NUM_FLOATS = 11
BYTES_PER_ROW = 44
pack_string = "<" + (NUM_FLOATS * "f") # number of f's match count


print("\nStoring results in an in-memory buffer")

# In-memory buffer
buffer = bytearray(ROWS * BYTES_PER_ROW)

bno.update_sensors()
_, _, _, _, first_sensor_ms = bno.linear_acceleration.full

i = 0
start = ticks_ms()
while i < ROWS:
    while not bno.update_sensors():
        pass
    
    if bno.linear_acceleration.updated:
        ax, ay, az, acc, ts_ms = bno.linear_acceleration.full
        qr, qi, qj, qk = bno.quaternion
        gy, gp, gr = bno.gyro

        offset = i * BYTES_PER_ROW
        struct.pack_into(pack_string, buffer, offset, ts_ms, ax, ay, az, qr, qi, qj, qk, gy, gp, gr)

        i += 1

last_sensor_ms = ts_ms
pico_ms = ticks_diff(ticks_ms(), start)

print(f"\nPrinting each Linear_Acc for {ROWS} rows:")
print(f"sensor timestamps {last_sensor_ms=} {first_sensor_ms=}  sensor duration: {(last_sensor_ms-first_sensor_ms)/1000:.1f} s")
print(f"Sensor msec/reports = {(last_sensor_ms-first_sensor_ms)/ROWS:.2f} ms")

print(f"Clock msec/reports  = {(pico_ms/ROWS):.2f} ms")
    
# Reset file
filename = "sensor_log.bin"
with open(filename, "wb") as f:
    pass

write_start = ticks_ms()
with open(filename, "ab") as f:
    f.write(buffer) # Write whole buffer 44,000 bytes
    f.flush()
    os.sync()
print(f"Time to write {len(buffer)} bytes, time = {ticks_diff(ticks_ms(), write_start)/1000.0} s")
    

# =====================================================================
# Time Packing data into a 4 KiB buffer & writing sectors for flash
print("\nWriting data to Flash in 4 KiB sector chunks to flash")
ROWS = 1000
NUM_FLOATS = 11
BYTES_PER_ROW = 44
ROWS_PER_SECTOR = 93 
DATA_SIZE = BYTES_PER_ROW * ROWS_PER_SECTOR # 4092 = 44 * 93
SECTOR_SIZE = 4096  # Exactly 4 KiB

# Buffer of exactly 4 KiB, data: 4092 CRC: last 4 bytes
sector_buffer = bytearray(SECTOR_SIZE)

# Reset file
filename = "sector_log.bin"
with open(filename, "wb") as f:
    pass

bno.update_sensors()
_, _, _, _, first_sensor_ms = bno.linear_acceleration.full

i = 0
start = ticks_ms()
with open(filename, "ab") as f:
    while i < ROWS:
        
        # Accumulate one sector of data ~ 96 rows
        sector_row_count = 0
        while sector_row_count < ROWS_PER_SECTOR and i < ROWS:
            if not bno.update_sensors():
                pass
            
            if bno.linear_acceleration.updated:
                ax, ay, az, acc, ts_ms = bno.linear_acceleration.full
                qr, qi, qj, qk = bno.quaternion
                gy, gp, gr = bno.gyro

                # Pack ONLY into the sector buffer
                offset = sector_row_count * BYTES_PER_ROW
                struct.pack_into(pack_string, sector_buffer, offset, 
                                 ts_ms, ax, ay, az, qr, qi, qj, qk, gy, gp, gr)

                sector_row_count += 1                
                i += 1
                
        # Calculate CRC32 on the data (first 4092 bytes), Pack the CRC  32bit "I" into the last 4 sector bytes
        # adds .08 ms to loop  6.18 ms with flush, 6.26 with flush & CRC
        crc = binascii.crc32(memoryview(sector_buffer)[:DATA_SIZE])
        struct.pack_into("<I", sector_buffer, DATA_SIZE, crc)

        # Write sector to flash:  bytes 0-4091 are data, last 4 bytes are CRC or 0x00 padding
        # write_start = ticks_ms()
        f.write(sector_buffer) # Write exactly 4 KiB
        f.flush()  # 5.85 ms without flush, 6.18 ms with flush
        # os.sync()  # 6.22 ms with sync
        
        # typical output: Sector flushed (4 KiB). Write: 45 ms. Total Rows so far: 930
        # write_time = ticks_diff(ticks_ms(), write_start)
        # print(f"Sector flushed (4 KiB). Write: {write_time} ms. Total Rows so far: {i}")
        
    if sector_row_count > 0:
        # Final partial sector
        crc = binascii.crc32(memoryview(sector_buffer)[:DATA_SIZE])
        struct.pack_into("<I", sector_buffer, DATA_SIZE, crc)
        f.write(sector_buffer)
        f.flush()
        os.sync()
        print(f"Final partial sector flushed. Total rows: {i}")
          
last_sensor_ms = ts_ms
pico_ms = ticks_diff(ticks_ms(), start)

print(f"\nArray each result for {ROWS} rows:")
print(f"Sensor timestamps {last_sensor_ms=} {first_sensor_ms=}  sensor duration: {(last_sensor_ms-first_sensor_ms)/1000:.1f} s")
print(f"Sensor msec/reports = {(last_sensor_ms-first_sensor_ms)/ROWS:.2f} ms")

print(f"Clock msec/reports  = {(pico_ms/ROWS):.2f} ms")

print(f"{BYTES_PER_ROW=}, data size = {(ROWS * BYTES_PER_ROW)} bytes")
kbytes = (BYTES_PER_ROW * ROWS) / 1024
print(f"Array = {kbytes:.1f} KiB, xfer = {kbytes/(pico_ms/1000.0):.1f} KiB/s")
