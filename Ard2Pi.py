import os
import sys
import serial
import struct
import binascii
import time
import requests
import json
import math
import _thread as thread
from Graph import Graph
from Edge import Edge
from Dijkstra import Dijkstra
from Constant import Constant
from GraphUtility import GraphUtility

# Define Constants
SYN = 0
SYNACK = 1
ACK = 2
DATA = 3
# Handshake timeout duration in seconds
HANDSHAKE_TIMEOUT = 2
# Espeak cmd format
ESPEAK_FORMAT = "espeak \'{}\' -w out.wav && aplay out.wav"

is_SYN_sent = False
is_ACK_sent = False

first_packet_code = -1

flag_start = False
flag_kill = False

def initiate_handshake():
    global is_SYN_sent
    print("Sending SYN")
    port.write(bytes(chr(SYN), 'UTF-8'))
    syn_sent_time = time.time()
    while (not is_SYN_sent):
        if ((time.time() - syn_sent_time) > HANDSHAKE_TIMEOUT):
            return
        if (port.inWaiting() > 0):
            current_byte = port.read()
            packet_code = int(binascii.hexlify(current_byte), 16)
    
            if (packet_code == SYNACK):
                print("SYNACK received")
                is_SYN_sent = True

# Finalises the handshake by receiving the ACK and the first data
def finalise_handshake():
    global is_ACK_sent
    print("Sending ACK")
    port.write(bytes(chr(ACK), 'UTF-8'))
    ack_sent_time = time.time()
    while (not is_ACK_sent):
        if ((time.time() - ack_sent_time) > HANDSHAKE_TIMEOUT):
            return
        if (port.inWaiting() > 0):
            current_byte = port.read()
            packet_code = int(binascii.hexlify(current_byte), 16)

            if (packet_code != SYNACK):
                global first_packet_code
                first_packet_code = packet_code
                print("DATA received")
                is_ACK_sent = True

def readlineCR(port):
    temp = []
    data = []
    checksum = 0
    computed_checksum = 0
    
    # Read the op code
    global first_packet_code
    if (first_packet_code is not -1):
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

# Converts the array of bytes into float rep
def convert_to_float(data):
    data.reverse()
    int_data = [int(x, 16) for x in data]
    float_result = struct.pack('4B', *int_data)
    float_result = struct.unpack('>f', float_result)
    return float_result

# Serial port details
port = serial.Serial(
	"/dev/ttyAMA0", 
	baudrate = 115200, 
	timeout = 0
)

def main_thread():
    global flag_kill
    
    os.system(ESPEAK_FORMAT.format("Please enter the building number"))
    building_num = int(input("Please enter the building number: "))
    
    os.system(ESPEAK_FORMAT.format("Please enter the level number"))
    level_num = int(input("Please enter the level number: "))
    
    try:
        jsonRequest = requests.get("http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building=" + str(building_num) + "&Level=" + str(level_num))
        json_data = jsonRequest.json()
    # in the case of any exceptions use the cached map
    except:
        filename = "Building" + building_num + "Level" + level_num + ".json"
        
        # open local map file in map_cache
        with open('map_cache/' + filename) as cached_data:
            json_data = json.load(cached_data)

    graph = Graph(json_data)

    os.system(ESPEAK_FORMAT.format("Please enter your source node ID"))
    source_id = int(input("Please enter your source node ID: "))

    # Initialize shortest path from source
    shortest_path = Dijkstra(graph, source_id)

    os.system(ESPEAK_FORMAT.format("Please enter your destination node ID"))
    destination_id = int(input("Please enter your destination node ID: "))

    distance = shortest_path.dist_to_node(destination_id)
    path = shortest_path.get_path(destination_id)

    os.system(ESPEAK_FORMAT.format("You are" + str(round(distance / 100, 1)) + " meters from your destination"))

    os.system(ESPEAK_FORMAT.format("Attempting to establish connection with Arduino"))

    is_first_data = True
    last_step_count = 0
    current_id_in_path = 0
    next_id_in_path = 1
    
    # Before we begin, we flush the port
    port.flushInput()
    
    # Connection establishment loop (handshake protocol)
    while True:
        if(flag_kill):
            flag_kill = False
            return
        
        if (not is_SYN_sent):
            initiate_handshake()
        elif (not is_ACK_sent):
            finalise_handshake()
        elif (port.inWaiting() > 0):
            break

    os.system(ESPEAK_FORMAT.format("Connection with Arduino has been established"))

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
        if(flag_kill):
            flag_kill = False
            return
        
        if (port.inWaiting() == 0):
            continue;

        buffer_data = readlineCR(port)

        # Check if data is cut or corrupted
        if (buffer_data[0] != buffer_data[0] or buffer_data[2] != buffer_data[3]):
            continue
        else:
            print(buffer_data)
            # Check if this is the first data received
            if (is_first_data):
                rotate_direction = next_node.get_rotation_difference_from_node(
                    current_node,
                    buffer_data[4][0],
                    graph.get_north_angle()
                    )
                walk_distance = next_node.calculate_euclidean_distance_from_node(current_node)

                print("Rotate " + str(int(round(rotate_direction, 0))) + " degrees and walk " + str(round(walk_distance / 100, 1)) + " meters")
                
                os.system(ESPEAK_FORMAT.format(
                    "Rotate " + str(int(round(rotate_direction, 0))) + " degrees and walk " + str(round(walk_distance / 100, 1)) + " meters"
                    ))
                port.flushInput()
                
                last_prompt_time = time.time()
                last_step_count = buffer_data[4][2]
                is_first_data = False
            else:
                current_position_x += (math.sin(GraphUtility.calculate_map_aligned_angle(buffer_data[4][0], graph.get_north_angle()) * Constant.RADIAN_TO_DEGREE_RATIO) *
                                       (buffer_data[4][2] - last_step_count) * Constant.AVERAGE_STEP_DISTANCE)
                current_position_y += (math.cos(GraphUtility.calculate_map_aligned_angle(buffer_data[4][0], graph.get_north_angle()) * Constant.RADIAN_TO_DEGREE_RATIO) *
                                       (buffer_data[4][2] - last_step_count) * Constant.AVERAGE_STEP_DISTANCE)

                # If next node is reached (within 1 step distance)
                if (next_node.calculate_euclidean_distance_from_point(current_position_x, current_position_y) <= 
                    Constant.DISTANCE_FROM_NODE_THRESHOLD):
                    # If this is the last node in the path, exit the program
                    if (path[next_id_in_path] == destination_id):
                        print("You have reached your destination")
                        os.system(ESPEAK_FORMAT.format("You have reached your destination"))
                        os.system(ESPEAK_FORMAT.format("Destination ID is " + str(destination_id)))
                        break
                    else:
                        print("You have reached the next node in the path: " + str(next_node))
                        os.system(ESPEAK_FORMAT.format("You have reached the next node in the path"))
                        os.system(ESPEAK_FORMAT.format("Node ID is " + str(next_node.get_id())))
                        port.flushInput()
                    
                    # Update current and next nodes
                    current_id_in_path = next_id_in_path
                    current_node = next_node
                    next_id_in_path += 1
                    next_node = graph.get_node(path[next_id_in_path])
                

                # If deviating from designated path
                if (Edge(current_node, next_node).get_normal_length_from_point(current_position_x, current_position_y) > Constant.DISTANCE_FROM_EDGE_THRESHOLD):
                    print("Rerouting... Please wait")
                    os.system(ESPEAK_FORMAT.format(
                        "Rerouting... Please wait"
                        ))
                    port.flushInput()

                    nearest_edge = graph.get_nearest_edge_from_point(current_position_x, current_position_y)
                    nearest_node = nearest_edge.get_nearest_node_from_point(current_position_x, current_position_y)
                    other_node = nearest_edge.get_other(nearest_node)

                    shortest_path = Dijkstra(graph, nearest_node.get_id())
                    path = shortest_path.get_path(destination_id)

                    current_id_in_path = -1
                    current_node = other_node
                    next_id_in_path = 0
                    next_node = nearest_node

                rotate_direction = next_node.get_rotation_difference_from_point(
                    current_position_x,
                    current_position_y,
                    buffer_data[4][0],
                    graph.get_north_angle()
                    )
                walk_distance = next_node.calculate_euclidean_distance_from_point(current_position_x, current_position_y)

                last_step_count = buffer_data[4][2]
                # Check if it is time to prompt the user
                if (time.time() - last_prompt_time >= Constant.PROMPT_DELAY):
                    print("Rotate " + str(int(round(rotate_direction, 0))) + " degrees and walk " + str(round(walk_distance / 100, 1)) + " meters")
                    
                    os.system(ESPEAK_FORMAT.format(
                        "Rotate " + str(int(round(rotate_direction, 0))) + " degrees and walk " + str(round(walk_distance / 100, 1)) + " meters"
                        ))
                    port.flushInput()

                    last_prompt_time = time.time()

def main():
    global flag_start
    # listen for escape key to restart program
    thread.start_new_thread(main_thread, ())
    # thread.start_new_thread(reset, ())
    while(True):
        if(flag_start):
            flag_start = False
            thread.start_new_thread(main_thread, ())
    
def reset():
    global flag_start
    global flag_kill
    while(True):
        reset_string = sys.stdin.read(1)
        if(reset_string.find('\x1b') != -1):
            flag_start = True
            flag_kill = True

if __name__ == "__main__":
    main()

