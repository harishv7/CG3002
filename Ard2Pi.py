import serial
import struct
import binascii

def readlineCR(port):
        temp = []
        data = []
        ch = port.read()
        checksum = 0
        xor = 0
        size = int(binascii.hexlify(ch), 16)
        ch = port.read()
        
        while (ch != b'\r' and ch != b''):
                xor = xor ^ int(binascii.hexlify(ch), 16)
                # If the len of arr is less than 4,
                # we add it to the array after converting the received
                # bytes into hexadecimal
                if(len(temp) < 4):
                        temp.append(binascii.hexlify(ch))
                else:
                        b = convert_to_float(temp)
                        data.append(b[0])
                        temp = []
                        temp.append(binascii.hexlify(ch))
                ch = port.read()

        
        try:
                # Append the last reading into the return data
                data.append(convert_to_float(temp)[0])

                # Read checksum
                checksum = int(binascii.hexlify(port.read()), 16)
        except:
                print("Exception")

        # Return a tuple containing the size and data
        return (size, checksum, xor, data)

# Convert the array of bytes into float rep
def convert_to_float(data):
        data.reverse()
        int_data = [int(x, 16) for x in data]
        b = struct.pack('4B', *int_data)
        b = struct.unpack('>f', b)
        return b

port = serial.Serial(
          "/dev/ttyAMA0",
          baudrate = 115200,
          timeout = 0
          )

ctr = 0

while True:
        if (port.inWaiting() > 0):
                ctr += 1
                data = readlineCR(port)
                print("Iteration " + str(ctr))
                print(data)
