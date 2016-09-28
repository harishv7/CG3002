import serial
import struct
import binascii

def readlineCR(port):
    temp = []
    data = []
    checksum = 0
    computed_checksum = 0
    
    # Read the op code
    current_byte = port.read()
    size = int(binascii.hexlify(current_byte), 16)
    
    current_byte = port.read()
    while (current_byte != b'\r' and current_byte != b''):
        # Convert the current_byte to hex format
        current_hex = binascii.hexlify(current_byte)
        # Update the computed_checksum by xor-ing with the newly acquired data
        computed_checksum = computed_checksum ^ int(current_hex, 16)
        # If length is < 4, keep appending
        if(len(temp) < 4):
            temp.append(current_hex)
        else:
            current_float = convert_to_float(temp)
            data.append(current_float[0])
            temp = []
            temp.append(current_hex)
        
        current_byte = port.read()
        
    try:
        # Append the last reading into the return data
        data.append(convert_to_float(temp)[0])
        
        # Read checksum
        checksum = int(binascii.hexlify(port.read()), 16)
    except:
        print("InvalidArgumentException")
        
    # Return a tuple containing the size and data
    return (size, checksum, computed_checksum, data)

# Convert the array of bytes into float rep
def convert_to_float(data):
    data.reverse()
    int_data = [int(x, 16) for x in data]
    float_result = struct.pack('4B', *int_data)
    float_result = struct.unpack('>f', b)
    return float_result

port = serial.Serial(
    "/dev/ttyAMA0",
    baudrate = 115200,
    timeout = 0
)

while True:
    if (port.inWaiting() > 0):
        buffer_data = readlineCR(port)
        print(buffer_data)