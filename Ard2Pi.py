import os
import serial
import struct
import binascii
import time
import requests
import json
import math
from Graph import Graph
from Dijkstra import Dijkstra
from Constant import Constant
from GraphUtility import GraphUtility

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
        port.flushInput()
        return (float('NaN'), float('NaN'), float('NaN'), float('NaN'), float('NaN'))
        # print("InvalidArgumentException")
    
    # Return data together with stepcount
    data.append(stepcount)

    # Return a tuple containing the size and data
    return (packet_code, size, checksum, computed_checksum, data)

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

# Define port details
port = serial.Serial(
	"/dev/ttyAMA0", 
	baudrate = 115200, 
	timeout = 0
)

def main():
    jsonRequest = requests.get("http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building=COM1&Level=1")

    graph = Graph(jsonRequest.json())
   
    os.system("espeak 'Please enter the source node ID' -w out.wav && aplay out.wav")
    source_id = int(input("Please enter the source node ID: "))
   
    shortest_path = Dijkstra(graph, source_id)
   
    os.system("espeak 'Please enter your destination node ID' -w out.wav && aplay out.wav")
    destination_id = int(input("Please enter the destination node ID: "))
   
    distance = shortest_path.dist_to_node(destination_id)
    path = shortest_path.get_path(destination_id)

    os.system("espeak 'Hello Harish' -w out.wav && aplay out.wav")

    prompt = "espeak \'You are " + str(distance / 100) + " metres from your destination\' -w out.wav && aplay out.wav"
    os.system(prompt)
    print(path)

    os.system("espeak 'Attempting to connect with Arduino' -w out.wav && aplay out.wav")

    is_first_data = True
    last_step_count = 0
    current_id_in_path = 0
    next_id_in_path = 1
    
    # Before we begin, we flush the port
    port.flushInput()
    
    # Infinite loop to read data from port
    while True:
        if (not is_SYN_sent):
            initiate_handshake()
        elif (not is_ACK_sent):
            finalise_handshake()
        elif (port.inWaiting() > 0):
            break

    is_first_data = True
    last_prompt_time = time.time()
    last_step_count = 0
    current_id_in_path = 0
    current_node = graph.get_node(path[current_id_in_path])
    next_id_in_path = 1
    next_node = graph.get_node(path[next_id_in_path])
    current_position_x = current_node.get_x()
    current_position_y = current_node.get_y()

    while True:
        if (port.inWaiting() == 0):
            continue;

        buffer_data = readlineCR(port)
        # If data is cut or corrupted
        if (buffer_data[0] != buffer_data[0] or buffer_data[2] != buffer_data[3]):
            continue
        else:
            print(buffer_data)
            if (is_first_data):
                rotate_direction = math.trunc(next_node.get_rotation_difference_from_node(
                    current_node,
                    buffer_data[4][0],
                    graph.get_north_angle()
                    ))
                walk_distance = next_node.calculate_euclidean_distance_from_node(current_node)

                print("To get to the next node (" + str(next_node) + "), you have to: ")
                print("Rotate " + str(rotate_direction) + " degrees and walk " + str(round(walk_distance / 100, 1)) + " meters")
                
                prompt = "espeak \'Rotate " + str(rotate_direction) + " degress and walk " + str(round(walk_distance / 100, 1)) + " meters\' -w out.wav && aplay out.wav"
                os.system(prompt)
                
                last_prompt_time = time.time()
                last_step_count = buffer_data[4][2]
                is_first_data = False
            else:
                current_position_x += (math.sin(GraphUtility.calculate_map_aligned_angle(buffer_data[4][0], graph.get_north_angle()) * Constant.RADIAN_TO_DEGREE_RATIO) *
                                       (buffer_data[4][2] - last_step_count) * Constant.AVERAGE_STEP_DISTANCE)
                current_position_y += (math.cos(GraphUtility.calculate_map_aligned_angle(buffer_data[4][0], graph.get_north_angle()) * Constant.RADIAN_TO_DEGREE_RATIO) *
                                       (buffer_data[4][2] - last_step_count) * Constant.AVERAGE_STEP_DISTANCE)

                # If next node is reached
                if (next_node.calculate_euclidean_distance_from_point(current_position_x, current_position_y) <= Constant.AVERAGE_STEP_DISTANCE):
                    if (path[next_id_in_path] == destination_id):
                        break
                    
                    current_id_in_path = next_id_in_path
                    current_node = next_node

                    next_id_in_path += 1
                    next_node = graph.get_node(path[next_id_in_path])
                
                rotate_direction = math.trunc(next_node.get_rotation_difference_from_point(
                    current_position_x,
                    current_position_y,
                    buffer_data[4][0],
                    graph.get_north_angle()
                    ))
                walk_distance = next_node.calculate_euclidean_distance_from_point(current_position_x, current_position_y)

                '''nearest_edge = graph.get_nearest_edge_from_point(current_position_x, current_position_y)
                nearest_node = nearest_edge.get_nearest_node_from_point(current_position_x, current_position_y)
                # If still within the designated path
                if ((nearest_edge.get_either() == current_node and nearest_edge.get_other(current_node) == next_node) or
                    (nearest_edge.get_either() == next_node and nearest_edge.get_other(next_node) == current_node)):
                    rotate_direction = math.trunc(next_node.get_rotation_difference_from_point(
                        current_position_x,
                        current_position_y,
                        buffer_data[4][0],
                        graph.get_north_angle()
                        ))
                    walk_distance = next_node.calculate_euclidean_distance_from_point(current_position_x, current_position_y)
                # Too far from designated path, reroute
                else:
                    print("Reroute")
                    shortest_path = Dijkstra(graph, current_node.get_id())
                    distance = shortest_path.dist_to_node(destination_id)
                    path = shortest_path.get_path(destination)
                    current_id_in_path = 0
                    current_node = graph.get_node(path[current_id_in_path])
                    next_id_in_path = 1
                    next_node = graph.get_node(path[next_node_id_in_path])

                    rotate_direction = math.trunc(next_node.get_rotation_difference_from_point(
                        current_position_x,
                        current_position_y,
                        buffer_data[4][0],
                        graph.get_north_angle()
                        ))
                    walk_distance = next_node.calculate_euclidean_distance_from_point(current_position_x, current_position_y)'''
                    
                last_step_count = buffer_data[4][2]

                if (time.time() - last_prompt_time >= Constant.PROMPT_DELAY):
                    print("To get to the next node (" + str(next_node) + "), you have to: ")
                    print("Rotate " + str(rotate_direction) + " degrees and walk " + str(round(walk_distance / 100, 1)) + " meters")
                    
                    prompt = "espeak \'Rotate " + str(rotate_direction) + " degress and walk " + str(round(walk_distance / 100, 1)) + " meters\' -w out.wav && aplay out.wav"
                    os.system(prompt)

                    last_prompt_time = time.time()

                # print(str(current_position_x) + " " + str(current_position_y))

if __name__ == "__main__":
    main()
