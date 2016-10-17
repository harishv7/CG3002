import serial
import struct
import binascii
import time
import requests
import json
from Graph import Graph
from Dijkstra import Dijkstra

# Define Constants
SYN = 0
SYNACK = 1
ACK = 2
DATA = 3

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

# Finalises the handshake by receiving the ACK and the first data
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
    for i in range(0, 4):
        current_byte = port.read()
        stepcount_bytes.append(binascii.hexlify(current_byte))
        
    stepcount = convert_to_int(stepcount_bytes)[0]

    computed_checksum = computed_checksum ^ stepcount
    
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
    
    # Return data together with stepcount
    collated_data = [stepcount, data]

    # Return a tuple containing the size and data
    return (packet_code, size, checksum, computed_checksum, collated_data)

# Converts the array of bytes into integer
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

def main():
   # Define port details
   port = serial.Serial(
       "/dev/ttyAMA0",
       baudrate = 115200,
       timeout = 0
   )
   
   jsonRequest = requests.get("http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building=COM1&Level=1")

   graph = Graph(jsonRequest.json())
   
   os.system("espeak -ven+f3 Please enter the source node ID")
   source_id = input("Please enter the source node ID")
   
   shortest_path = Dijkstra(graph, source_id)
   
   os.system("espeak -ven+f3 Please enter your destination node ID")
   destination_id = input("Please enter the destination node ID")
   
   distance = shortest_path.dist_to_node(destination_id)
   path = shortest_path.get_path(destination_id)
   
   os.system("espeak -ven+f3 You are " + str(distance / 100) + " metres from your destination")
   print(path)
   
   # Before we begin, we flush the port
   port.flushInput()

   # Infinite loop to read data from port
   while True:
       if (not is_SYN_sent):
           initiate_handshake()
       elif (not is_ACK_sent):
           finalise_handshake()
       elif (port.inWaiting() > 0):
           buffer_data = readlineCR(port)
           print(buffer_data)
    
if __name__ == "__main__":
    main()