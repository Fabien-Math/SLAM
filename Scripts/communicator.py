import struct

from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from world import World
	from robot import BeaconRobot


class Communicator:
	def __init__(self, robot, comm_range: float, emission_rate: int, recepton_rate: int, world) -> None:
		self.world: World = world
		self.robot: BeaconRobot = robot
		self.id: int = robot.id
		self.comm_range: float = comm_range
		
		self.bit_rate_emission: int = emission_rate
		self.available_emit_bit: int = 0
		self.bit_rate_reception: int = recepton_rate
		self.available_recep_bit: int = 0

		self.time: float = world.time
		self.dt: float = world.dt
		self.status: int = 0

		self.sender_buffer: str = []
		self.sender_history: dict = {str(i): [] for i in range(15) if i != self.id}
		self.receiver_buffer: str = {str(i): [] for i in range(15) if i != self.id}
		self.receiver_history: list = []

		self.conv_history_file = "conv_history.txt"

		self.robot_connection_set: list = [] 

		if self.id == 0:
			self.send_message('all', [f"Bonjour, je suis {self.id}"])

	def update_communicator(self):
		self.diffuse_signal()
		self.scan_signal()

	def encode_message_id(self, message_id, receiver):
		if message_id[4:] == '0000':
			return ""

		match message_id:
			case "00000001":
				return ""
			case "00010001":
				return self.sender_history[receiver][-1]
			case "00100001":	# Encode robot connection set
				bm = f"{len(self.robot_connection_set):.04b}"
				for r_id in self.robot_connection_set:
					bm += f"{r_id:04b}"
				return bm
			case "00100001":	# Encode next wp
				bm = ""
				if self.robot.controller.waypoint:
					bm += encode_float(self.robot.controller.waypoint[0])
					bm += encode_float(self.robot.controller.waypoint[1])
				else:
					bm += '1'*32
				return bm
			case "00100010":	# Encode robot info
				pos_x = encode_float(self.robot.pos_calc.x)
				pos_y = encode_float(self.robot.pos_calc.y)
				vel = encode_float(self.robot.speed_calc)
				ang = encode_float(self.robot.rot_calc)
				ang_vel = encode_float(self.robot.rot_speed_calc)
				bat = f"{self.robot.battery:08b}"
				return pos_x + pos_y + vel + ang + ang_vel + bat 
			case "00100011":	# Encode next waypoint to reach as an order
				...
			case "00110001":	# Encode live grid update
				...
			case _:
				print(f"WARNING : Message id not recognized {message_id}")
	
	def decode_message_id(self, message_id:str, binary_message:str, index:int, sender:int) -> int:
		match message_id:
			case "00000000":	# Repeat the last message
				self.sender_buffer += self.sender_history[str(sender)][-1]
			case "00000001":	# Last message received correctly
				with open(self.conv_history_file, 'a') as f:
					last_message = self.sender_history[str(sender)].pop()
					f.write(last_message)
			case "00010000":	# Send robot connection set
				self.send_message([sender], {"type": "mID", "mID" : "00010001"})
			case "00010001":	# Decode robot connection set
				list_length = binary_message[index:index+4]
				for i in range(list_length):
					robot_id = int(binary_message[index + (i+1)*4:index + (i+2)*4])
					if robot_id not in self.robot_connection_set:
						self.connection_set.append(robot_id)
				return None, index + 4 * (list_length + 1)
			case "00100000":	# Send next wp
				self.send_message([sender], {"type": "mID"})
			case "00100010":	# Decode robot info
				return decode_robot_info(self.robot), index + 168
			case "00100011":	# Decode next waypoint to reach as an order
				...
			case "00110001":	# Decode live grid update
				...
			case _:
				print(f"WARNING : Message id not recognized {message_id}")
		
		return None, index




		

	def encode_message_binary(self, receivers, messages):
		"""The encoding method uses an Automatic Repeat Query protocol. This means that a verification is send with the message to ensure than the message is well distributed.
			The format of the encoding will be very simple as follow
			|----|			4 bits emiter ID
			|----|			4 optional bits for the number of receiver if different from 1
			|----...----|	4xreceiver_nb bits for the 4 receiver
			|----...----|	Message
			|----|			4 bits for the end of the message
			|----|			4 bits for the units of the sum
			|----|			4 bits emiter ID with parity bit encode in the last one, change it if odd.
		"""
				
		# Add Emitter ID (4 bits)
		binary_message = f"{self.id:04b}"
		
		# Add number of receivers (4 bits) if needed
		if isinstance(receivers, list):
			if len(receivers) > 1:
				binary_message += f"{len(receivers):04b}"
			else:
				binary_message += "0001"  # Default if there's only 1 receiver

			# Add Receiver IDs (4 bits each)
			for receiver_id in receivers:
				binary_message += f"{receiver_id:04b}"
		elif isinstance(receivers, str):
			if receivers == 'all':
				binary_message += "1111"  # Default if there's all receiver

		# Encode Message
		for item in messages:
			if isinstance(item, float):
				binary_message += "0011"  # Type indicator for float
				binary_message += encode_float(item)
			elif isinstance(item, int):
				if 0 <= item < 256:
					binary_message += f"0001{item:08b}"
				else:
					binary_message += "0010"
					binary_message += encode_int32(item)
			elif isinstance(item, str):
				binary_message += "0100"
				binary_message += encode_str(item)
			elif isinstance(item, list):
				if isinstance(item[0], int):
					binary_message += "0101"  # Type indicator for list of int
					binary_message += encode_int8_list(item)
				elif isinstance(item[0], float):
					binary_message += "0110"  # Type indicator for list of float
					binary_message += encode_float_list(item)
			elif isinstance(item, dict):
				if item["type"] == "mID":
					binary_message += "0111"
					binary_message += item['mID']
					binary_message += self.encode_message_id(item['mID'], receivers)
				if item["type"] == "robot_info":
					binary_message += "1000"
					binary_message += encode_robot_info(self.robot)
					
		# End of the message
		binary_message += "0000"

		# Sum bits (4 bits)
		sum_units = int(sum(int(bit) for bit in binary_message) % 16)
		binary_message += f"{sum_units:04b}"
		
		# Emitter ID with Parity Bit (4 bits)
		parity_bit = binary_message.count('1') % 2
		final_emitter_id = self.id
		if parity_bit == 1:
			final_emitter_id ^= 1  # Flip the last bit if parity is odd
		binary_message += f"{final_emitter_id:04b}"
	
		return binary_message
	

	def decode_message_binary(self, binary_message):
		"""
		Decode a binary string message encoded with the custom protocol.
		
		Args:
			binary_message (str): The encoded message as a binary string.
		
		Returns:
			dict: Decoded message components, including emitter ID, receiver IDs, and the message content.
		"""
		index = 0
		
		if len(binary_message) < 4:
			return
		# Step 1: Extract Emitter ID (4 bits)
		emitter_id = int(binary_message[index:index+4], 2)
		index += 4

		if len(binary_message) < index + 4:
			return
		# Step 2: Extract optional number of receivers (4 bits)
		num_receivers = int(binary_message[index:index+4], 2)
		index += 4

		if num_receivers == 15:
			# Step 3: Extract Receiver IDs (4 bits each)
			receivers = []
			for receiver_id in range(16):
				if receiver_id != emitter_id:
					receivers.append(receiver_id)
		else:
			# Step 3: Extract Receiver IDs (4 bits each)
			receivers = []
			if len(binary_message) < index + 4*num_receivers:
				return
			for _ in range(num_receivers):
				receivers.append(int(binary_message[index:index+4], 2))
				index += 4

		# Minimal length of the message 4 + 4 + 8 + 4
		if len(binary_message) < index + 20:
			return
		
		# Step 4: Extract the message
		message = []
		while index < len(binary_message) - 4:  # Leave space for sum and parity bits
			type_indicator = int(binary_message[index:index+4], 2)
			index += 4
			match type_indicator:
				case 0b000:
					break
				case 0b0001:	# Int8
					i = int(binary_message[index:index+8], 2)  # 8 bits for length
					message.append(i)
					index += 8
				case 0b0011:	# Float32
					float_bits = binary_message[index:index+32]
					f = decode_float(float_bits)
					message.append(f)
					index += 32
				case 0b0010:	# Int32
					int_bits = binary_message[index:index+32]
					i = decode_int(int_bits)
					message.append(i)
					index += 32
				case 0b0100:	# String 
					str_length = int(binary_message[index:index+8], 2)  # 8 bits for length
					index += 8
					string_bits = binary_message[index:index+8*str_length]
					s = decode_string(string_bits)
					message.append(s)
					index += 8 * str_length
				case 0b0101:	# List of int8
					list_length = int(binary_message[index:index+16], 2)  # 8 bits for length
					index += 16
					int_list_bits = binary_message[index:index+8*list_length]
					il = decode_int8_list(int_list_bits, list_length)
					message.append(il)
					index += 8 * list_length
				case 0b0101:	# List of int8
					list_length = int(binary_message[index:index+16], 2)  # 8 bits for length
					index += 16
					float_list_bits = binary_message[index:index+32*list_length]
					fl = decode_int8_list(float_list_bits, list_length)
					message.append(fl)
					index += 32 * list_length
				case 0b0111:	# Message with ID
					mID = binary_message[index:index+8]
					index += 8
					m, index = self.decode_message_id(mID, binary_message, index, emitter_id)
					if m:
						message.append(m)
				case 0b1000:	# Robot info
					rb = binary_message[index:index+168]
					rinfo, index = decode_robot_info(rb)
					message.append(rinfo)	
				case _:
					raise ValueError(f"Unknown type indicator: {type_indicator}")
				
		

		if len(binary_message) < index + 4:
			return
		
		# Step 5: Extract units of the sum (4 bits)
		sum_units = int(binary_message[index:index+4], 2)
		index += 4

		if len(binary_message) < index + 4:
			return
		
		# Step 6: Extract emitter ID with parity bit (4 bits)
		emitter_id_with_parity = int(binary_message[index:index+4], 2)
		index += 4

		sum_units_check = int(sum(int(bit) for bit in binary_message[:-8]) % 16)

		if sum_units != sum_units_check:
			raise ValueError("Sum check failed.")


		# Parity check (optional)
		parity_bit = binary_message[:-4].count('1') % 2
		is_parity_valid = ((emitter_id ^ emitter_id_with_parity) & 1) == parity_bit
		if not is_parity_valid:
			raise ValueError("Parity check failed.")
		
		return {
			"message_length": index,
			"emitter_id": emitter_id,
			"receivers": receivers,
			"message": message,
			"sum_units": sum_units,
			"parity_valid": is_parity_valid
		}


	def diffuse_signal(self):
		if not len(self.sender_buffer):
			return

		self.available_emit_bit += self.bit_rate_emission * self.world.dt

		if self.available_emit_bit < 1:
			return
		
		for receivers, message in self.sender_buffer:
			if self.available_emit_bit < 1:
				break
		
			bits_to_send = min(len(message), int(self.available_emit_bit))
			for r in receivers:
				message_diffused = message[:bits_to_send]
				self.sender_history[str(r)].append(message_diffused)
				self.world.message_buffer[str(self.id)] += message_diffused
			
			self.available_emit_bit -= min(len(message), int(self.available_emit_bit))

			if bits_to_send == len(message):
				...
				#TO BE DONE 


		self.sender_buffer = self.sender_buffer[int(self.available_emit_bit):]

		self.available_emit_bit -= int(self.available_emit_bit)

	def scan_signal(self):
		if not len(self.receiver_buffer):
			return
		
		if len(self.receiver_buffer) < 8:
			return
		
		message_received = self.decode_message_binary(self.receiver_buffer)
		if message_received:
			self.receiver_history.append(message_received)
			self.receiver_buffer = self.receiver_buffer[message_received["message_length"]+1:]

			print(self.id)
			print(message_received)
			print('\n')


	def send_message(self, receivers, message):
		encoded_message = self.encode_message_binary(receivers, message)
		self.sender_buffer.append((receivers, encoded_message))





#####   ENCODING TYPES   #####

def encode_float(f:float) -> str:
	float_bytes = struct.pack('>f', float)
	return ''.join(f"{byte:08b}" for byte in float_bytes)

def encode_int32(i:int) -> str:
	int_bytes = struct.pack('>i', i)
	return ''.join(f"{byte:08b}" for byte in int_bytes)
	
def encode_str(s:str) -> str:
	return f"{len(s):08b}" + ''.join(f"{ord(char):08b}" for char in s)

def encode_float_list(fl:list) -> str:
	bm = f"{len(fl):016b}"  # Length of the list (16 bits)
	for f in fl:
		bm += encode_float(f)
	return bm

def encode_int8_list(il:list) -> str:
	bm = f"{len(il):016b}"  # Length of the list (16 bits)
	for i in il:
		bm += f"{i:08b}"
	return bm

def encode_robot_info(robot) -> str:
	pos_x = encode_float(robot.pos_calc.x)
	pos_y = encode_float(robot.pos_calc.y)
	vel = encode_float(robot.speed_calc)
	ang = encode_float(robot.rot_calc)
	ang_vel = encode_float(robot.rot_speed_calc)
	bat = f"{robot.battery:08b}"
	return pos_x + pos_y + vel + ang + ang_vel + bat 


#####   DECODING TYPES   #####

def decode_float(fb:str) -> float:
	return struct.unpack('>f', int(fb, 2).to_bytes(4, byteorder='big'))[0]

def decode_int(ib:str) -> int:
	return struct.unpack('>i', int(ib, 2).to_bytes(4, byteorder='big'))[0]

def decode_string(sb:str) -> str:
	return ''.join(chr(int(sb[i:i+8], 2)) for i in range(0, len(sb), 8))

def decode_int8_list(ib:str, n:int) -> list:
	int8_list = [0]*n
	for i in range(n):
		int8_list[i] = int(ib[i*8:(i+1)*8], 2)
	return int8_list

def decode_float_list(fb:str, n:int) -> list:
	float32_list = [0]*n
	for i in range(n):
		float32_list[i] = decode_float(fb[i*32:(i+1)*32])
	return float32_list

def decode_robot_info(rb:str) -> str:
	pos_x = decode_float(rb[0:32])
	pos_y = decode_float(rb[32:32*2])
	vel = decode_float(rb[32*2:32*3])
	ang = decode_float(rb[32*3:32*4])
	ang_vel = decode_float(rb[32*4:32*5])
	bat = decode_float(rb[32*5:32*5 + 8])
	robot_info = {"pos": (pos_x, pos_y), "vel": vel, "ang": ang, "ang_vel": ang_vel, "bat": bat}
	return robot_info