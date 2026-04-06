import serial 
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

def checksum(data):
    return (~sum(data)) & 0xFF

def ping_servo(servo_id):
    packet = [0x55, 0x55, servo_id, 3, 14]
    packet.append(checksum(packet[2:]))

    ser.write(bytearray(packet))
    time.sleep(0.02)

    response = ser.read(6)
    return len(response) > 0

print("Scanning for servos...")

for i in range(1, 254):
    if ping_servo(i):
        print(f"Found servo at ID: {i}")

ser.close()
