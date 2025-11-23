import serial

ser = serial.Serial("COM10", 115200, timeout=0.5)  # change COM port!

print("Listening...")

while True:
    data = ser.read(64)
    if data:
        print(data)
