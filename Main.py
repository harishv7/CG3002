import math
import re
import numpy as np
import heapq
import requests
import json
import queue
import os
import serial
import struct
import binascii
import time

class Constant:
	# Mathematical constants
	EPS = 1e-03
	PI = math.acos(-1)
	RADIAN_TO_DEGREE = 180 / PI
	DEGREE_TO_RADIAN = PI / 180
	INF = 1000000007

	# Navigation related parameters
	EAST_TO_NORTH_ANGLE = 90
	AVERAGE_STEP_DISTANCE = 55
	PROMPT_DELAY = 4
	DISTANCE_FROM_NODE_THRESHOLD = 100
	DISTANCE_FROM_EDGE_THRESHOLD = 200

	# Serial communication message codes
	SYN_CODE = 0
	SYNACK_CODE = 1
	ACK_CODE = 2
	DATA_CODE = 3
	HANDSHAKE_TIMEOUT = 2

	# Serial communication states
	DISCONNECTED_STATE = 0
	SYNCHRONIZING_STATE = 1
	CONNECTED_STATE = 2

	# Other constants
	ESPEAK_FORMAT = "espeak \'{}\' -a 1000 -w out.wav && aplay out.wav"

class Point:
	def __init__(self, x, y):
		self.x = x
		self.y = y

	def get_x(self):
		return self.x

	def get_y(self):
		return self.y

	def calculate_rotation_from(self, other, graph_aligned_heading_angle):
		cartesian_rotation = math.atan2(self.y - other.get_y(), self.x - other.get_x()) * Constant.RADIAN_TO_DEGREE
		absolute_rotation = 90 - cartesian_rotation - graph_aligned_heading_angle
		if (absolute_rotation <= -180):
			absolute_rotation += 360
		else if (absolute_rotation > 180):
			absolute_rotation -= 360
		return absolute_rotation

	def __add__(self, other):
		return Point(self.get_x() + other.get_x(), self.get_y() + other.get_y())

	def __sub__(self, other):
		return Point(self.get_x() - other.get_x(), self.get_y() - other.get_y())

	@staticmethod
	def calculate_manhattan_distance(point1, point2):
		return abs(point1.get_x() - point2.get_x()) + abs(point1.get_y() - point2.get_y())

	@staticmethod
	def calculate_squared_distance(point1, point2):
		return math.pow(point1.get_x() - point2.get_x(), 2) + math.pow(point1.get_y() - point2.get_y(), 2)

	@staticmethod
	def calculate_euclidean_distance(point1, point2):
		return math.sqrt(Point.calculate_squared_distance(point1, point2))

class Node:
	def __init__(self, id, name, x, y):
		self.id = id
		self.name = name
		self.position = Point(x, y)
		self.edges = []

	def get_id(self):
		return self.id

	def get_name(self):
		return self.name

	def get_position(self):
		return self.position

	def get_edges(self):
		return self.edges

	def add_edge(self, edge):
		self.edges.append(edge)

	def get_connected_node_info(self):
		connecting_node_pattern = re.compile("^TO \d+\-\d+\-\d+$", re.IGNORECASE)
		if (connecting_node_pattern.match(self.name)):
			tokens = re.split(" |-", self.name)
			return (int(tokens[1]), int(tokens[2]), int(tokens[3]))
		else:
			return None

	def calculate_vector_to(self, other):
		return other.position - self.position

	def calculate_manhattan_distance_to(self, other):
		return Point.calculate_manhattan_distance(self.position, other.position)

	def calculate_squared_distance_to(self, other):
		return Point.calculate_squared_distance(self.position, other.position)

	def calculate_euclidean_distance_to(self, other):
		return Point.calculate_euclidean_distance(self.position, other.position)

	def __repr__(self):
		return "Node " + str(self.id) + " (" + str(self.name) + ")"

	def __str__(self):
		return self.__repr__()

class Edge:
	def __init__(self, source, target):
		self.source = source
		self.target = target
		self.manhattan_distance = source.calculate_manhattan_distance_to(target)
		self.squared_distance = source.calculate_squared_distance_to(target)
		self.euclidean_distance = source.calculate_euclidean_distance_to(target)

	def get_source(self):
		return self.source

	def get_target(self):
		return self.target

	def get_manhattan_distance(self):
		return self.manhattan_distance

	def get_squared_distance(self):
		return self.squared_distance

	def get_euclidean_distance(self):
		return self.euclidean_distance

	def calculate_euclidean_distance_to(self, other):
		if (isinstance(other, Point)):
			return calculate_distance_from_point(self, other)

	def calculate_euclidean_distance_to_point(self, point):
		source = self.get_source()
		target = self.get_target()

		source_to_target_vector = np.asarray(target.position - source.position)
		source_to_point_vector = np.asarray(point - source.position)

		distance = np.linalg.norm(np.cross(source_to_target_vector, source_to_point_vector)) / np.linalg.norm(source_to_target_vector)

	def __repr__(self):
		return "Edge from " + str(self.source) + " to " + str(self.target) + ", distance: " + str(self.euclidean_distance)

	def __str__(self):
		return self.__repr__()

class Graph:
	def __init__(self, building_num, level_num):
		self.building_num = building_num
		self.level_num = level_num
		self.north_angle = None
		self.nodes = []
		self.id_to_node_map = {}
		self.edges = []

		json = get_json(building_num, level_num)

		if (json is not None):
			# Get the north angle information
			self.north_angle = float(json["info"]["northAt"])
			if (self.north_angle > 180):
				self.north_angle -= 360
			# Get all node information
			for node_info in json["map"]:
				node_id = int(node_info["nodeId"])
				node_name = node_info["nodeName"]
				node_x = int(node_info["x"])
				node_y = int(node_info["y"])
				neighbours_id = node_info["linkTo"].split(", ")

				self.add_node(node_id, node_name, node_x, node_y)

				for neighbour_id in neighbours_id:
					self.add_edge(node_id, int(neighbour_id))

	def get_building_num(self):
		return self.building_num

	def get_level_num(self):
		return self.level_num

	def get_north_angle(self):
		return self.north_angle

	def get_nodes(self):
		return self.nodes

	def get_id_to_node_map(self):
		return self.id_to_node_map

	def get_connected_nodes_info(self):
		connected_nodes_info = []
		for node in self.nodes:
			connected_node_info = node.get_connected_node_info()
			if (connected_node_info is not None):
				connected_nodes_info.append(((self.building_num, self.level_num, node.get_id()), connected_node_info))
		return connected_nodes_info

	def add_node(self, node_id, node_name, node_x, node_y):
		node = Node(node_id, node_name, node_x, node_y)
		self.nodes.append(node)
		self.id_to_node_map[node_id] = node

	def add_edge(self, source_id, target_id):
		if (source_id not in self.id_to_node_map):
			return
		if (target_id not in self.id_to_node_map):
			return
		source = self.id_to_node_map[source_id]
		target = self.id_to_node_map[target_id]

		edge = Edge(source, target)
		source.add_edge(edge)
		self.edges.append(edge)

		edge = Edge(target, source)
		target.add_edge(edge)
		self.edges.append(edge)

	def calculate_shortest_path(self, source_id, target_id):
		distance_to = {}
		previous_of = {}

		for node in self.nodes:
			node_id = node.get_id()
			distance_to[node_id] = math.inf
			previous_of[node_id] = None
		distance_to[source_id] = 0.0

		pq = []
		heapq.heappush(pq, (source_id, distance_to[source_id]))
		while (len(pq)):
			relax_tuple = heapq.heappop(pq)
			relax_source_id = relax_tuple[0]
			relax_distance = relax_tuple[1]
			for edge in self.id_to_node_map[relax_source_id].get_edges():
				relax_target_id = edge.get_target().get_id()
				relax_weight = edge.get_manhattan_distance()
				if (distance_to[relax_source_id] + relax_weight < distance_to[relax_target_id]):
					distance_to[relax_target_id] = distance_to[relax_source_id] + relax_weight
					previous_of[relax_target_id] = relax_source_id
					heapq.heappush(pq, (relax_target_id, distance_to[relax_target_id]))

		path_to_target = []
		current_node_id = target_id
		while (previous_of[current_node_id] is not None):
			path_to_target.append(current_node_id)
			current_node_id = previous_of[current_node_id]
		path_to_target.append(current_node_id)

		return (distance_to[target_id], path_to_target[::-1])

	def calculate_graph_aligned_angle(self, north_aligned_angle):
        map_aligned_angle = north_aligned_angle + self.north_angle
        if (map_aligned_angle <= -180):
            map_aligned_angle += 360
        if (map_aligned_angle > 180):
            map_aligned_angle -= 360
        return map_aligned_angle

	def __repr__(self):
		info = "North is at " + str(self.north_angle) + " degrees"
		info += "Nodes in this graph:\n"
		for node in self.nodes:
			info += str(node) + "\n"
		info += "Edges in this graph:\n"
		for edge in self.edges:
			info += str(edge) + "\n"
		return info

	def __str__(self):
		return self.__repr__()

def get_json(building_num, level_num):
	try: # Try to retrieve json online
		json_request = requests.get("http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building=" + str(building_num) + "&Level=" + str(level_num), 
			timeout = 5)
		json_data = json_request.json()

		if (json_data["info"] is None):
			json_data = None
	except: # Failure in retrieving json online
		print("Network error in getting json, resorting to cached json...")
		try: # Try to load cached json
			filename = "Building" + building_num + "Level" + level_num + ".json"
			with open('map_cache/' + filename) as cached_data:
				json_data = json.load(cached_data)
		except: # Failure in loading cached json
			json_data = None

	if (json_data == None):
		os.system(Constant.ESPEAK_FORMAT.format("Invalid building or level number"))
		print("Invalid building or level number")

	return json_data

def request_node_info(prompt_keyword):
	json_data = None
	while (json_data is None):
		os.system(Constant.ESPEAK_FORMAT.format("Please enter the " + prompt_keyword + " building number"))
		building_num = int(input("Please enter the " + prompt_keyword + " building number: "))

		os.system(Constant.ESPEAK_FORMAT.format("Please enter the " + prompt_keyword + " level number"))
		level_num = int(input("Please enter the " + prompt_keyword + " level number: "))

		json_data = get_json(building_num, level_num)

	os.system(Constant.ESPEAK_FORMAT.format("Please enter the " + prompt_keyword + " node id"))
	node_id = int(input("Please enter the " + prompt_keyword + " node id: "))

	return (building_num, level_num, node_id)

def request_valid_node_info(prompt_keyword):
	json_data = None
	is_node_id_invalid = True
	while (json_data is None or is_node_id_invalid):
		node_info = request_node_info(prompt_keyword)
		json_data = get_json(node_info[0], node_info[1])

		if (json_data is not None):
			graph = Graph(node_info[0], node_info[1])
			if (node_info[2] in graph.get_id_to_node_map()):
				is_node_id_invalid = False

	return node_info

def identify_intermediate_nodes(source_node_info, target_node_info):
	os.system(Constant.ESPEAK_FORMAT.format("Identifying intermediate nodes"))
	print("Identifying intermediate nodes...")
	
	que = queue.Queue(maxsize = 0)
	que.put(source_node_info)
	previous_node_info = {}
	previous_node_info[source_node_info] = None
	is_visited = {}

	is_target_building_found = False
	while (que.qsize() != 0 and not is_target_building_found):
		current_node_info = que.get()
		current_graph = Graph(current_node_info[0], current_node_info[1])

		connected_nodes_info = current_graph.get_connected_nodes_info()
		for connected_node_info in connected_nodes_info:
			# Prevent node revisiting
			if (connected_node_info[1] in is_visited):
				continue
			# Check if the current node is a connecting node
			if (connected_node_info[0] == current_node_info):
				previous_node_info[connected_node_info[1]] = current_node_info
				is_visited[current_node_info] = True
			else:
				previous_node_info[connected_node_info[1]] = connected_node_info[0]
				is_visited[connected_node_info[0]] = True
				previous_node_info[connected_node_info[0]] = current_node_info
				is_visited[current_node_info] = True
			# Check if target node is reached
			if (connected_node_info[1][0] == target_node_info[0] and connected_node_info[1][1] == target_node_info[1]):
				is_target_building_found = True
				break
			else:
				que.put(connected_node_info[1])

	# Check if target graph has been found but not at destination node yet
	if (connected_node_info[1] != target_node_info):
		previous_node_info[target_node_info] = connected_node_info[1]

	os.system(Constant.ESPEAK_FORMAT.format("Backtracking"))
	print("Backtracking...")

	# Backtrack to get all the intermediate buildings info
	intermediate_nodes_info = []
	current_node_info = target_node_info
	if (is_target_building_found):
		while (previous_node_info[current_node_info] is not None):
			intermediate_nodes_info.append(current_node_info)
			current_node_info = previous_node_info[current_node_info]
	else:
		os.system(Constant.ESPEAK_FORMAT.format("Error: source and target buildings are not connected"))
		print("Error: source and target buildings are not connected")
	intermediate_nodes_info.append(source_node_info)

	# Reverse the intermediate nodes before returning
	return intermediate_nodes_info[::-1]

def send_synchronization(port, connection_state):
	print("Sending SYN")
	port.write(bytes(chr(Constant.SYN_CODE)), "UTF-8")
	last_send_time = time.time()
	while (connection_state == Constant.DISCONNECTED_STATE):
		if (time.time() - last_send_time > Constant.HANDSHAKE_TIMEOUT):
			break
		if (port.inWaiting() > 0):
			current_byte = port.read()
			packet_code = int(binascii.hexlify(current_byte), 16)

			if (packet_code == Constant.SYNACK_CODE):
				print("SYNACK received")
				connection_state = Constant.SYNCHRONIZING_STATE

	return (port, connection_state)

def send_acknowledgement(port, connection_state):
	print("Sending ACK")
	port.write(bytes(chr(Constant.ACK_CODE)), "UTF-8")
	last_send_time = time.time()
	while (connection_state == Constant.SYNCHRONIZING_STATE):
		if (time.time() - last_send_time > Constant.HANDSHAKE_TIMEOUT):
			break
		if (port.inWaiting() > 0):
			current_byte = port.read()
			packet_code = int(binascii.hexlify(current_byte), 16)

			if (packet_code == Constant.DATA_CODE):
				print("DATA received")
				connection_state = Constant.CONNECTED_STATE

	return (port, connection_state)

def convert_to_int(int_bytes):
	try:
		int_bytes.reverse()
		int_hex = [int(x, 16) for x in int_bytes]
		int_result = struct.pack('4B', *int_hex)
		int_result = struct.unpack('>i', int_result)
		return int_result[0]
	except:
		return float('NaN')

def convert_to_float(float_bytes):
	try:
		float_bytes.reverse()
		float_hex = [int(x, 16) for x in float_bytes]
		float_result = struct.pack('4B', *float_hex)
		float_result = struct.unpack('>f', float_result)
		return float_result[0]
	except:
		return float('NaN')

def read_byte(port):
	try:
		current_byte = port.read()
		return (port, int(binascii.hexlify(current_byte), 16))
	except:
		return (port, float('NaN'))

def read_int(port):
	try:
		current_bytes = []
		for i in range(4):
			current_byte = port.read()
			current_bytes.append(binascii.hexlify(current_byte))
		return (port, convert_to_int(current_bytes))
	except:
		return (port, float('NaN'))

def read_float(port):
	try:
		current_bytes = []
		for i in range(4):
			current_byte = port.read()
			current_bytes.append(binascii.hexlify(current_byte))
		return (port, convert_to_float(current_bytes))
	except:
		return (port, float('NaN'))

def receive_data(port, connection_state):
	if (connection_state == Constant.CONNECTED_STATE):
		# Packet code already read in send_acknowledgement, proceed with data reading
		connection_state = Constant.POLLING_STATE
	elif (connection_state = Constant.POLLING_STATE):
		# Extract packet code
		port, packet_code = read_byte(port)
	else:
		os.system(Constant.ESPEAK_FORMAT.format("Unexpected connection state. Reverting to disconnected state"))
		print("Unexpected connection state. Reverting to disconnected state...")
		connection_state = DISCONNECTED_STATE
		return (port, connection_state, None)

	computed_checksum = 0

	# Read step count
	port, step_count = read_int(port)
	if (math.isnan(step_count)):
		port.flushInput()
		return (port, connection_state, None)

	computed_checksum = computed_checksum ^ step_count

	# Read heading angle
	port, heading_angle = read_float(port)
	if (math.isnan(heading_angle)):
		port.flushInput()
		return (port, connection_state, None)

	computed_checksum = computed_checksum ^ heading_angle

	# Read surface height
	port, surface_height = read_float(port)
	if (math.isnan(surface_height)):
		port.flushInput()
		return (port, connection_state, None)

	computed_checksum = computed_checksum ^ surface_height

	# Read expected checksum
	port, expected_checksum = read_int(port)
	if (math.isnan(expected_checksum)):
		port.flushInput()
		return (port, connection_state, None)

	if (computed_checksum != expected_checksum):
		return (port, connection_state, None)
	else:
		return (port, connection_state, (step_count, heading_angle, surface_height))

def main():
	source_node_info = request_valid_node_info("source")
	source_graph = Graph(source_node_info[0], source_node_info[1])
	target_node_info = request_valid_node_info("target")
	target_graph = Graph(target_node_info[0], target_node_info[1])

	intermediate_nodes_info = identify_intermediate_nodes(source_node_info, target_node_info)

	print(len(intermediate_nodes_info))
	print(intermediate_nodes_info)

	# Initialize and clear serial port
	port = serial.Serial(
		"/dev/ttyAMA0", 
		baudrate = 115200, 
		timeout = 0
	)
	port.flushInput()

	os.system(Constant.ESPEAK_FORMAT.format("Initiating handshake protocol"))
	print("Initiating handshake protocol...")
	port.flushInput()

	# START OF HANDSHAKE PROTOCOL #
	connection_state == Constant.DISCONNECTED_STATE
	while (connection_state == Constant.DISCONNECTED_STATE || connection_state == Constant.SYNCHRONIZING_STATE):
		if (connection_state == Constant.DISCONNECTED_STATE):
			port, connection_state = send_synchronization(port, connection_state)
		elif (connection_state == Constant.SYNCHRONIZING_STATE):
			port, connection_state = send_acknowledgement(port, connection_state)
	# END OF HANDSHAKE PROTOCOL #

	os.system(Constant.ESPEAK_FORMAT.format("Handshake is successful"))
	print("Handshake is successful")
	port.flushInput()

	os.system(Constant.ESPEAK_FORMAT.format("Starting directional guidance"))
	print("Starting directional guidance...")
	port.flushInput()

	# START OF DIRECTIONAL GUIDANCE #
	intermediate_node_iter = 0
	last_intermediate_node_info = intermediate_nodes_info[intermediate_node_iter]
	intermediate_node_iter += 1
	if (last_intermediate_node_info == target_node_info):
		return
	next_intermediate_node_info = intermediate_nodes_info[intermediate_node_iter]
	intermediate_node_iter += 1
	# If building and level num do not match, this means that last and next nodes are connecting nodes
	if (last_intermediate_node_info[0] != next_intermediate_node_info[0] || last_intermediate_node_info[1] != next_intermediate_node_info[1]):
		last_intermediate_node_info = next_intermediate_node_info
		if (last_intermediate_node_info == target_node_info):
			return
		next_intermediate_node_info = intermediate_nodes_info[intermediate_node_iter]
		intermediate_node_iter += 1

	current_graph = Graph(last_intermediate_node_info[0], last_intermediate_node_info[1])
	id_to_node_map = current_graph.get_id_to_node_map()
	last_intermediate_node = id_to_node_map[last_intermediate_node_info[2]]
	next_intermediate_node = id_to_node_map[next_intermediate_node_info[2]]

	distance, path = current_graph.calculate_shortest_path(last_intermediate_node_info[2], next_intermediate_node_info[2])
	os.system(Constant.ESPEAK_FORMAT.format("You are " + str(round(distance / 100, 1)) + " meters from the next intermediate node"))
	print("You are " + str(round(distance / 100, 1)) + " meters from the next intermediate node")
	port.flushInput()
	
	path_iter = 0
	last_node = id_to_node_map[path[path_iter]]
	path_iter += 1
	next_node = id_to_node_map[path[path_iter]]
	path_iter += 1
	current_position = last_node.get_position()
	last_prompt_time = time.time()
	last_step_count = 0
	is_initial_data = True

	while (connection_state == Constant.CONNECTED_STATE || connection_state == Constant.POLLING_STATE):
		port, connection_state, data = receive_data(port, connection_state)
		if (data is None):
			continue
		step_count = data[0]
		heading_angle = data[1]
		surface_height = data[2]
		print(step_count, heading_angle, surface_height)

		rotate_direction = next_node.get_position().calculate_rotation_from(current_position, calculate_graph_aligned_angle(heading_angle))
		walk_distance = Point.calculate_euclidean_distance(current_position, next_node.get_position())
		num_steps = int(round(walk_distance / Constant.AVERAGE_STEP_DISTANCE, 0))

		if (is_initial_data):
			os.system(Constant.ESPEAK_FORMAT.format("Rotate " + str(int(round(rotate_direction, 0))) + " degrees and walk " + str(num_steps) + " steps"))
			print("Rotate " + str(int(round(rotate_direction, 0))) + " degrees and walk " + str(num_steps) + " steps")
			port.flushInput()

			last_prompt_time = time.time()
			last_step_count = step_count

			is_initial_data = False
		else:
			current_position = Point(
				current_position.get_x() + math.sin(current_graph.calculate_graph_aligned_angle(heading_angle) * Constant.DEGREE_TO_RADIAN) * 
				(step_count - last_step_count) * Constant.AVERAGE_STEP_DISTANCE, 
				current_position.get_y() + math.cos(current_graph.calculate_graph_aligned_angle(heading_angle) * Constant.DEGREE_TO_RADIAN) * 
				(step_count - last_step_count) * Constant.AVERAGE_STEP_DISTANCE
				)

			last_step_count = step_count

			distance_to_next_node = Point.calculate_euclidean_distance(current_position, next_node.get_position())
			# Check if next node is reached
			if (distance_to_next_node <= Constant.DISTANCE_FROM_NODE_THRESHOLD):
				last_node = next_node

				os.system(Constant.ESPEAK_FORMAT.format("You have reached " + str(last_node)))
				print("You have reached " + str(last_node))
				port.flushInput()
				# Check if next intermediate node is reached
				if (last_node == next_intermediate_node):
					# Check if target is reached
					if (next_intermediate_node_info == target_node_info):
						os.system(Constant.ESPEAK_FORMAT.format("You have reached your destination"))
						print("You have reached your destination")
						port.flushInput()
						return
					else:
						last_intermediate_node_info = next_intermediate_node_info
						next_intermediate_node_info = intermediate_nodes_info[intermediate_node_iter]
						intermediate_node_iter += 1

						if (last_intermediate_node_info[0] != next_intermediate_node_info[0] || last_intermediate_node_info[1] != next_intermediate_node_info[1]):
							last_intermediate_node_info = next_intermediate_node_info
							if (last_intermediate_node_info == target_node_info):
								os.system(Constant.ESPEAK_FORMAT.format("You have reached your destination"))
								print("You have reached your destination")
								port.flushInput()
								return
							next_intermediate_node_info = intermediate_nodes_info[intermediate_node_iter]
							intermediate_node_iter += 1

						current_graph = Graph(last_intermediate_node_info[0], last_intermediate_node_info[1])
						id_to_node_map = current_graph.get_id_to_node_map()
						last_intermediate_node = id_to_node_map[last_intermediate_node_info[2]]
						next_intermediate_node = id_to_node_map[next_intermediate_node_info[2]]

						distance, path = current_graph.calculate_shortest_path(last_intermediate_node_info[2], next_intermediate_node_info[2])
						os.system(Constant.ESPEAK_FORMAT.format("You are " + str(round(distance / 100, 1)) + " meters from the next intermediate node"))
						print("You are " + str(round(distance / 100, 1)) + " meters from the next intermediate node")
						port.flushInput()

						path_iter = 0
						last_node = id_to_node_map[path[path_iter]]
						path_iter += 1
						next_node = id_to_node_map[path[path_iter]]
						path_iter += 1
						current_position = Point(last_node.get_position())
				else:
					last_node = next_node
					next_node = id_to_node_map[path[path_iter]]
					path_iter += 1

			# Check if it is time to prompt the user
			if (time.time() - last_prompt_time > Constant.PROMPT_DELAY):
				os.system(Constant.ESPEAK_FORMAT.format("Rotate " + str(int(round(rotate_direction, 0))) + " degrees and walk " + str(num_steps) + " steps"))
				print("Rotate " + str(int(round(rotate_direction, 0))) + " degrees and walk " + str(num_steps) + " steps")
				port.flushInput()

				last_prompt_time = time.time()
	# END OF DIRECTIONAL GUIDANCE #

if (__name__ == "__main__"):
	main()