import struct

from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from world import World


class Communicator:
	def __init__(self, id: int, range: float, emission_rate: int, recepton_rate: int, world) -> None:
		self.world: World = world
		self.id: int = id
		self.range: float = range
		
		self.bit_rate_emission: int = emission_rate
		self.available_emit_bit: int = 0
		self.bit_rate_reception: int = recepton_rate
		self.available_recep_bit: int = 0

		self.time: float = world.time
		self.dt: float = world.dt
		self.status: int = 0

		self.sender_buffer: str = ''
		self.sender_history: str = ''
		self.receiver_buffer: str = ''
		self.receiver_history: list = []

		if self.id == 0:
			self.send_message('all', [f"Bonjour, je suis {self.id}"])

	def update_communicator(self):
		self.diffuse_signal()
		self.scan_signal()
		

	
	def compute_message_size(self, receivers, message):
		"""Compute the total length of the message once encoded

		Args:
			receivers (list): List of 4-bit receiver IDs (0-15).
			message (list): List of elements to encode. Elements can be strings, floats, or integers.
		
		Returns:
			str: Encoded message as a binary string.
		"""
		
		# Step 1: Add Emitter ID (4 bits)
		const_size = 4
		
		# Step 2: Add number of receivers (4 bits) if needed
		if isinstance(receivers, list):
			const_size += 4 * (len(receivers) + 1)
		elif isinstance(receivers, str):
			if receivers == 'all':
				const_size += 4  # Default if there's all receiver
		
		binary_message_size = 0
		# Step 4: Encode Message
		for item in message:
			if isinstance(item, float):
				binary_message_size += 4 + 32
			elif isinstance(item, int):
				binary_message_size += 4 + 32
			elif isinstance(item, str):
				binary_message_size += 4  # Type indicator for string
				binary_message_size += 8  # Length of the string (8 bits) 256 caracters max
				binary_message_size += 8 * len(item)
		
		const_size += 8
		
		return const_size, const_size + binary_message_size
	

	def split_message(self, message, message_const_size, max_size):

		splitters = [0]
		current_message_size = message_const_size
		for i, item in enumerate(message):
			if isinstance(item, float):
				current_message_size += 4 + 32
			elif isinstance(item, int):
				current_message_size += 4 + 32
			elif isinstance(item, str):
				str_length = 4 + 8 + 8 * len(item)
				if str_length > max_size:
					raise ValueError(f"Message to long : {str_length}")
				current_message_size += str_length

			if current_message_size > max_size:
				splitters.append(i)
				current_message_size = message_const_size

		messages = []
		for i in range(len(splitters)-1):
			messages.append(message[splitters[i]:splitters[i+1]])
		messages.append(message[splitters[i+1]:])

		return messages


	def encode_message_binary(self, receivers, messages):
		"""The encoding method uses an Automatic Repeat Query protocol. This means that a verification is send with the message to ensure than the message is well distributed.
			The format of the encoding will be very simple as follow
			|----|			4 bits emiter ID
			|----|			4 optional bits for the number of receiver if different from 1
			|----...----|	4xreceiver_nb bits for the 4 receiver
			|----...----|	32 bytes message (8 32 bits floats)
			|----|			4 bits for the units of the sum
			|----|			4 bits emiter ID with parity bit encode in the last one, change it if odd.
			A 272 (34 bytes) to 332 bits (41.5 bytes) message template
		"""

		# m_const_size, m_size = self.compute_message_size(self.id, receivers, messages)
		# if m_size > 512:
		# 	messages = self.split_message(messages, m_const_size, 512)
		# 	print(messages)
		# else:
		# 	messages = [messages]

		# binary_messages = []
		# for message in messages:
		binary_message = ""
		
		# Step 1: Add Emitter ID (4 bits)
		binary_message += f"{self.id:04b}"
		
		# Step 2: Add number of receivers (4 bits) if needed
		if isinstance(receivers, list):
			if len(receivers) > 1:
				binary_message += f"{len(receivers):04b}"
			else:
				binary_message += "0001"  # Default if there's only 1 receiver

			# Step 3: Add Receiver IDs (4 bits each)
			for receiver_id in receivers:
				binary_message += f"{receiver_id:04b}"
		elif isinstance(receivers, str):
			if receivers == 'all':
				binary_message += "1111"  # Default if there's all receiver

		
		
		# Step 4: Encode Message
		for item in messages:
			if isinstance(item, float):
				binary_message += "0001"  # Type indicator for float
				float_bytes = struct.pack('>f', item)
				binary_message += ''.join(f"{byte:08b}" for byte in float_bytes)
			elif isinstance(item, int):
				binary_message += "0010"  # Type indicator for integer
				int_bytes = struct.pack('>i', item)
				binary_message += ''.join(f"{byte:08b}" for byte in int_bytes)
			elif isinstance(item, str):
				binary_message += "0011"  # Type indicator for string
				binary_message += f"{len(item):08b}"  # Length of the string (8 bits)
				binary_message += ''.join(f"{ord(char):08b}" for char in item)
			elif isinstance(item, list):
				binary_message += "0100"  # Type indicator for list of float
				binary_message += f"{len(item):016b}"  # Length of the list (16 bits)
				for fitem in item:
					float_bytes = struct.pack('>f', fitem)
					binary_message += ''.join(f"{byte:08b}" for byte in float_bytes)
		
		# End of the message
		binary_message += "0000"

		# Step 5: Add Units of the Sum (4 bits)
		# sum_units = int(sum(
		# 	(float(item) if isinstance(item, (float, int)) else len(item))
		# 	for item in messages
		# ) % 16)

		sum_units = int(sum(int(bit) for bit in binary_message) % 16)
		binary_message += f"{sum_units:04b}"
		
		# Step 6: Add Emitter ID with Parity Bit (4 bits)
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

		# Minimal length of the message 4 + 8 + 8 + 4
		if len(binary_message) < index + 24:
			return
		
		# Step 4: Extract the message
		message = []
		while index < len(binary_message) - 4:  # Leave space for sum and parity bits
			type_indicator = int(binary_message[index:index+4], 2)
			index += 4
			
			if type_indicator == 0b0001:  # Float
				float_bits = binary_message[index:index+32]
				float_value = struct.unpack('>f', int(float_bits, 2).to_bytes(4, byteorder='big'))[0]
				message.append(float_value)
				index += 32
			elif type_indicator == 0b0010:  # Integer
				int_bits = binary_message[index:index+32]
				int_value = struct.unpack('>i', int(int_bits, 2).to_bytes(4, byteorder='big'))[0]
				message.append(int_value)
				index += 32
			elif type_indicator == 0b0011:  # String
				str_length = int(binary_message[index:index+8], 2)  # 8 bits for length
				index += 8
				string_bits = binary_message[index:index+8*str_length]
				string_value = ''.join(
					chr(int(string_bits[i:i+8], 2)) for i in range(0, len(string_bits), 8)
				)
				message.append(string_value)
				index += 8 * str_length
			elif type_indicator == 0b0100:  # List
				list_length = int(binary_message[index:index+16], 2)  # 8 bits for length
				index += 16
				float_list = []
				for _ in range(list_length):
					float_bits = binary_message[index:index+32]
					float_value = struct.unpack('>f', int(float_bits, 2).to_bytes(4, byteorder='big'))[0]
					float_list.append(float_value)
					index += 32
				message.append(float_list)
			elif type_indicator == 0b0000:	# Message end
				break
			else:
				return
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
		
		message_diffused = self.sender_buffer[:int(self.available_emit_bit)]

		self.sender_history += message_diffused
		self.world.message_buffer[str(self.id)] += message_diffused

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
		self.sender_buffer += encoded_message


