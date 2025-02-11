import struct

# from typing import TYPE_CHECKING
# if TYPE_CHECKING:
# 	from world import World
# 	from robot import BeaconRobot


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

		self.last_hello_message: float = 0
		self.hello_message_freq: int = 1	# Every ... seconds

		self.sender_buffer: str = []
		self.sender_history: dict = {str(i): [] for i in range(15) if i != self.id}
		self.receiver_buffer: str = {str(i): '' for i in range(15) if i != self.id}
		self.receiver_history: list = []

		self.conv_history_file = "Scripts/conv_history.txt"
		if self.id == 0:
			with open(self.conv_history_file, 'w') as f:
				f.write(f"{self.id:04b}" + ' - Connected !\n')
		else:
			with open(self.conv_history_file, 'a') as f:
				f.write(f"{self.id:04b}" + ' - Connected !\n')


		self.robot_connection_set: list = [] 


	def update_communicator(self):
		self.time = self.world.time
		if self.last_hello_message + self.hello_message_freq < self.time:
			self.robot_connection_set = []
			self.send_message('all', [])
			self.last_hello_message = self.time
		self.diffuse_signal()
		self.scan_signal()


	def encode_message_id(self, message, receiver):
		message_id = message['mID']
		print("Encode message id : ", message_id)
		if message_id[4:] == '0000':
			return ""

		match message_id:
			case "00000000":
				return ""
			case "00000001":
				return ""
			case "00010000":	# Ask for robot connection set
				return ""
			case "00010001":	# Encode robot connection set
				bm = f"{len(self.robot_connection_set):04b}"
				for r_id in self.robot_connection_set:
					bm += f"{r_id:04b}"
				return bm
			case "00100000":	# Ask for next wp
				return ""
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
			case "00110000":	# Ask for live grid sync
				return ""
			case "00110001":	# Encode live grid update
				updated_live_grid_map = self.robot.live_grid_map.updated_cells
				print("Encode updated live grid map")
				print(updated_live_grid_map)
				live_grid_map = self.robot.live_grid_map.map
				bm = encode_int32(len(updated_live_grid_map.keys())*3)
				for key in updated_live_grid_map:
					bm += f"{key[0]:08b}"
					bm += f"{key[1]:08b}"
					bm += f"{live_grid_map[key[1], key[0]]:08b}"
				self.robot.live_grid_map.updated_cells = {}
				return bm
			case "01010101":
				message_length = len(message["message"])
				return encode_int32(message_length) + message["message"]
			case _:
				print(f"WARNING : Message id not recognized {message_id}")
	
	def decode_message_id(self, message_id:str, binary_message:str, index:int, sender:int) -> int:
		print('Decode message id', message_id, message_id == '00110001')
		
		match message_id:
			case "00000000":	# Repeat the last message
				self.sender_buffer += self.sender_history[str(sender)][-1]
			case "00000001":	# Last message received correctly
				with open(self.conv_history_file, 'a') as f:
					last_message = self.sender_history[str(sender)].pop()
					f.write(last_message + '\n')
				return "ACK", index
			case "00010000":	# Send robot connection set
				self.send_message([sender], [{"type": "mID", "mID" : "00010001"}])
			case "00010001":	# Decode robot connection set
				if len(binary_message[index:index+4]) != 4:
					return None, None

				list_length = int(binary_message[index:index+4], 2)
				m = []
				if len(binary_message[index + 4: index + (list_length+1)*4]) != list_length*4:
					return None, None

				for i in range(list_length):
					robot_id = int(binary_message[index + (i+1)*4:index + (i+2)*4], 2)
					if robot_id not in self.robot_connection_set and self.id != robot_id:
						self.robot_connection_set.append(robot_id)
					m.append(robot_id)
				return m, index + 4 * (list_length + 1)
			
			case "00100000":	# Send next wp
				self.send_message([sender], [{"type": "mID", "mID": "00100001"}])
				self.send_message([sender], [{"type": "mID", "mID": "00100010"}])
			case "00100010":	# Decode robot info
				if len(binary_message[index:index+168]) != 168:
					return None, None
				return decode_robot_info(binary_message[index:index+168]), index + 168
			case "00100011":	# Decode next waypoint to reach as an order
				x = decode_float32(binary_message[index:index+32])
				y = decode_float32(binary_message[index+32:index+64])
				return ("NWP", (x, y)), index + 64
			case "00110000":	# Send live grid update because asked
				print("Send live grid update")
				self.send_message([sender], [{"type": "mID", "mID": "00110001"}])
			case "00110001":	# Decode live grid update
				print("Decode live grid update")
				list_size = decode_int32(binary_message[index:index+32])
				updated_live_grid_map = decode_int8_list(binary_message[index+32:index+32+list_size*8], list_size)
				return ("LGM_UPDATE", updated_live_grid_map), index + 32 + list_size*8
			case "01010101":
				message_size = decode_int32(binary_message[index:index+32])
				check_message, message = self.decode_message_binary(binary_message[index+32:index+32+message_size])
				if check_message:
					self.handle_message(message)
				return message, index + 32 + message_size
			case _:
				print(f"WARNING : Message id not recognized {message_id}")
				return None, None
		
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
			binary_message += f"{len(receivers):04b}"

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
					binary_message += self.encode_message_id(item, receivers)
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
		if len(binary_message) < 20:
			return False, None
		
		emitter_id = int(binary_message[0:4], 2)
		receiver_nb = int(binary_message[4:8], 2)

		if receiver_nb == 15:
			receivers = [i for i in range(receiver_nb) if i != emitter_id]
			index = 8
		else:
			receivers = [int(binary_message[8 + i * 4: 8 + (i + 1) * 4], 2)for i in range(receiver_nb)]
			if len(binary_message[8:8+receiver_nb*4]) != receiver_nb*4:
				return False, None
			index = 8+receiver_nb*4

		message = []
		max_cpt = 0
		while max_cpt < 100000:  # Leave space for sum and parity bits
			max_cpt += 1
			if len(binary_message[index:index+4]) != 4:
				return False, None
			
			type_indicator = int(binary_message[index:index+4], 2)
			index += 4
			match type_indicator:
				case 0b0000:
					break
				case 0b0001:	# Int8
					if len(binary_message[index:index+8]) != 8:
						return False, None
					i = int(binary_message[index:index+8], 2)
					message.append(i)
					index += 8
				case 0b0011:	# Float32
					if len(binary_message[index:index+32]) != 32:
						return False, None
					float_bits = binary_message[index:index+32]
					f = decode_float32(float_bits)
					message.append(f)
					index += 32
				case 0b0010:	# Int32
					if len(binary_message[index:index+32]) != 32:
						return False, None
					int_bits = binary_message[index:index+32]
					i = decode_int32(int_bits)
					message.append(i)
					index += 32
				case 0b0100:	# String 
					if len(binary_message[index:index+8]) != 8:
						return False, None
					str_length = int(binary_message[index:index+8], 2)  # 8 bits for length
					index += 8
					if len(binary_message[index:index+8*str_length]) != 8*str_length:
						return False, None
					string_bits = binary_message[index:index+8*str_length]
					s = decode_string(string_bits)
					message.append(s)
					index += 8 * str_length
				case 0b0110:	# List of int8
					if len(binary_message[index:index+16]) != 16:
						return False, None
					list_length = int(binary_message[index:index+16], 2)  # 8 bits for length
					index += 16
					if len(binary_message[index:index+8*list_length]) != 8*list_length:
						return False, None
					int_list_bits = binary_message[index:index+8*list_length]
					il = decode_int8_list(int_list_bits, list_length)
					message.append(il)
					index += 8 * list_length
				case 0b0101:	# List of float32
					if len(binary_message[index:index+16]) != 16:
						return False, None
					list_length = int(binary_message[index:index+16], 2)  # 8 bits for length
					index += 16
					if len(binary_message[index:index+32*list_length]) != 32*list_length:
						return False, None
					float_list_bits = binary_message[index:index+32*list_length]
					fl = decode_float_list(float_list_bits, list_length)
					message.append(fl)
					index += 32 * list_length
				case 0b0111:	# Message with ID
					if len(binary_message[index:index+8]) != 8:
						return False, None
					mID = binary_message[index:index+8]
					index += 8
					m, index = self.decode_message_id(mID, binary_message, index, emitter_id)
					if index is None:
						return False, None
					if m:
						message.append(m)
				case 0b1000:	# Robot info
					if len(binary_message[index:index+168]) != 168:
						return False, None
					rb = binary_message[index:index+168]
					rinfo, index = decode_robot_info(rb)
					message.append(rinfo)
				case _:
					print(f"Unknown type indicator: {type_indicator}, {bin(type_indicator)}")
		
		if len(binary_message[index:index+8]) != 8:
			return False, None

		# Units of the sum (4 bits)
		sum_units = int(binary_message[index:index+4], 2)
		index += 4
		
		# Parity bit (4 bits)
		emitter_id_with_parity = int(binary_message[index:index+4], 2)
		index += 4

		sum_units_check = int(sum(int(bit) for bit in binary_message[:index-8]) % 16)

		# Sum check
		if sum_units != sum_units_check:
			raise ValueError("Sum check failed.")

		# Parity check
		parity_bit = binary_message[:index-4].count('1') % 2
		is_parity_valid = ((emitter_id ^ emitter_id_with_parity) & 1) == parity_bit
		if not is_parity_valid:
			raise ValueError("Parity check failed.")
		
		
		return True, {
			"message_length": index,
			"emitter_id": emitter_id,
			"receivers": receivers,
			"message": message,
			"sum_units": sum_units == sum_units_check,
			"parity_valid": is_parity_valid
		}
			


	def diffuse_signal(self):
		self.available_emit_bit += self.bit_rate_emission * self.world.dt

		if self.available_emit_bit < 1:
			return
		
		nb_message_sent = 0
		for k, (receivers, message) in enumerate(self.sender_buffer):
			if self.available_emit_bit < 1:
				break
		
			nb_bits_to_send = min(len(message), int(self.available_emit_bit))
			message_to_diffuse = message[:nb_bits_to_send]
			
			if receivers == 'all':
				for i in range(15):
					if i != self.id:
						self.sender_history[str(i)].append(message_to_diffuse)
			else:
				for r in receivers:
					self.sender_history[str(r)].append(message_to_diffuse)
			
			self.world.message_buffer[str(self.id)] += message_to_diffuse

			self.available_emit_bit -= min(len(message), int(self.available_emit_bit))

			if nb_bits_to_send == len(message):
				nb_message_sent += 1
			else:
				self.sender_buffer[k][1] = message[nb_bits_to_send:]
		
		self.sender_buffer = self.sender_buffer[nb_message_sent:]

	def scan_signal(self):
		for i in range(15):
			if self.id == i:
				continue

			message = self.receiver_buffer[str(i)]
			if len(message) < 20:
				return
			
			check_message, message_info = self.decode_message_binary(message)
			if check_message:
				if self.id in message_info["receivers"]:
					self.handle_message(message_info)
					self.receiver_history.append(message_info)
					self.receiver_buffer[str(i)] = self.receiver_buffer[str(i)][message_info["message_length"]:]
				else:
					emitter_id = str(message_info["emitter_id"])
					self.receiver_buffer[emitter_id] = self.receiver_buffer[emitter_id][message_info["message_length"]:]


	def send_message(self, receivers, message):
		print(self.time, '- Send message message : ', message)
		encoded_message = self.encode_message_binary(receivers, message)
		self.sender_buffer.append([receivers, encoded_message])


	def handle_message(self, message):
		emitter_id = message['emitter_id']
		if emitter_id not in self.robot_connection_set:
			self.robot_connection_set.append(emitter_id)
			print(f"Robot {self.id} connected to robot {emitter_id}, asking for live grid map update")
			self.send_message([emitter_id], [{"type": "mID", "mID": "00110000"}])

		if message['message'] != ['ACK']:
			self.send_message([emitter_id], [{"type": "mID", "mID": "00000001"}])
		
		for m in message['message']:
			if isinstance(m, tuple):
				if m[0] == "NWP":
					self.robot.controller.waypoint = m[1]
					print(f"Robot {self.id} received new waypoint : {m[1]}")
				if m[0] == "LGM_UPDATE":
					print("Robot", self.id, "received live grid map update")
					for i in range(0, len(m[1]), 3):
						x = m[1][i]
						y = m[1][i+1]
						value = m[1][i+2]
						if self.robot.live_grid_map.map[y, x] != 19:
							self.robot.live_grid_map.map[y, x] = value
		
		pass


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

def decode_float32(fb:str) -> float:
	return struct.unpack('>f', int(fb, 2).to_bytes(4, byteorder='big'))[0]

def decode_int32(ib:str) -> int:
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
		float32_list[i] = decode_float32(fb[i*32:(i+1)*32])
	return float32_list

def decode_robot_info(rb:str) -> str:
	pos_x = decode_float32(rb[0:32])
	pos_y = decode_float32(rb[32:32*2])
	vel = decode_float32(rb[32*2:32*3])
	ang = decode_float32(rb[32*3:32*4])
	ang_vel = decode_float32(rb[32*4:32*5])
	bat = decode_float32(rb[32*5:32*5 + 8])
	robot_info = {"pos": (pos_x, pos_y), "vel": vel, "ang": ang, "ang_vel": ang_vel, "bat": bat}
	return robot_info