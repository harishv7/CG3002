import serial

port = serial.Serial(
    "/dev/ttyAMA0",
    baudrate = 115200,
    timeout = 0
)

port.write(bytes('556', 'UTF-8'))