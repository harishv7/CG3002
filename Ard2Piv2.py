import serial
import struct
import binascii
import time

##Define Constants
SYN = 0
SYNACK = 1
ACK = 2
NACK = 3
DATA = 4

# list of packet codes
packet_codes = [0,1,2,3,4,5]

is_SYN_sent = False
is_ACK_sent = False

FIRST_PACKET_CODE = -1
    
def initiate_handshake():
    global is_SYN_sent
    print("Sending SYN")
    port.write(bytes(chr(SYN), 'UTF-8'))
    syn_sent_time = time.time()
    while(not is_SYN_sent):
        if((time.time() - syn_sent_time) > 1):
            return
        if(port.inWaiting() > 0):
            current_byte = port.read()
            packet_code = int(binascii.hexlify(current_byte), 16)
    
            if(packet_code == SYNACK):
                print("Packet Code is SYNACK")
                print("SYN done")
                is_SYN_sent = True

def finalise_handshake():
    global is_ACK_sent
    print("Sending ACK")
    port.write(bytes(chr(ACK), 'UTF-8'))
    ack_sent_time = time.time()
    while(not is_ACK_sent):
        if((time.time() - ack_sent_time) > 1):
            return
        if(port.inWaiting() > 0):
            current_byte = port.read()
            packet_code = int(binascii.hexlify(current_byte), 16)

        
            if(packet_code == DATA):
                global first_packet_code
                first_packet_code = packet_code
                print("Packet Code is DATA")
                print("ACK done")
                is_ACK_sent = True

def readlineCR(port):
    temp = []
    data = []
    checksum = 0
    computed_checksum = 0
    
    # Read the op code
    global first_packet_code
    if(first_packet_code is not -1):
        packet_code = first_packet_code
        first_packet_code = -1
    else:
        current_byte = port.read()
        packet_code = int(binascii.hexlify(current_byte), 16)
        
    
    current_byte = port.read()
    size = int(binascii.hexlify(current_byte), 16)

    stepcount_bytes = []
    for i in (0,4):
        stepcount_bytes.append(port.read())

    stepcount = convert_to_int(stepcount_bytes)
    
    current_byte = port.read()
    while (current_byte != b'\r' and current_byte != b''):
        # Convert the current_byte to hex format
        current_hex = binascii.hexlify(current_byte)
        # Update the computed_checksum by xor-ing with the newly acquired data
        computed_checksum = computed_checksum ^ int(current_hex, 16)
        # If length is < 4 bytes, keep appending
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
        port.flushInput()
    
    # Return a tuple containing the size and data
    return (packet_code, size, checksum, computed_checksum, data)

def convert_to_int(data):
    data.reverse()
    int_data = [int(x, 16) for x in data]
    int_result = struct.pack('4B', *int_data)
    int_result = struct.unpack('>i', int_result)
    return int_result

# Convert the array of bytes into float rep
def convert_to_float(data):
    data.reverse()
    int_data = [int(x, 16) for x in data]
    float_result = struct.pack('4B', *int_data)
    float_result = struct.unpack('>f', float_result)
    return float_result

port = serial.Serial(
    "/dev/ttyAMA0",
    baudrate = 115200,
    timeout = 0
)

# clear buffer
#while(port.inWaiting() > 0):
 #   port.read()

port.flushInput()

while True:
    if (not is_SYN_sent):
        initiate_handshake()
    elif (not is_ACK_sent):
        finalise_handshake()
    elif (port.inWaiting() > 0):
        buffer_data = readlineCR(port)
        print(buffer_data)