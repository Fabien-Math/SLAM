import numpy as np
import struct


class Comminicator:
	def __init__(self, id: int, range: float, emission_rate: int, recepton_rate: int) -> None:
		self.id : int = id
		self.range : float = range
		
		self.bit_rate_emission : int = emission_rate
		self.bit_rate_reception : int = recepton_rate

		self.time : float = 0
		self.status : int = 0

		self.link_com_list : list = []

		self.receiver_buffer : list = []
		self.sender_buffer : list = []


	
	def compute_message_size(self, emitter_id, receivers, message):
		"""Compute the total length of the message once encoded

		Args:
			emitter_id (int): The 4-bit emitter ID (0-15).
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

	def split_message(self, message, message_const_size, message_size, max_size):

		splitters = [0]
		current_message_size = 0
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
				current_message_size = 0

		messages = []
		for i in range(len(splitters)-1):
			messages.append(message[splitters[i]:splitters[i+1]])

		return messages



	def encode_message_binary(self, emitter_id, receivers, messages):
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

		m_const_size, m_size = self.compute_message_size(emitter_id, receivers, messages)
		if m_size > 128:
			messages = self.split_message(messages, m_const_size, m_size, 128)
			print(messages)
		else:
			messages = [messages]

		binary_messages = []
		for message in messages:
			binary_message = ""
			
			# Step 1: Add Emitter ID (4 bits)
			binary_message += f"{emitter_id:04b}"
			
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
			for item in message:
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
			
			# Step 5: Add Units of the Sum (4 bits)
			sum_units = int(sum(
				(float(item) if isinstance(item, (float, int)) else len(item))
				for item in message
			) % 16)
			binary_message += f"{sum_units:04b}"
			
			# Step 6: Add Emitter ID with Parity Bit (4 bits)
			parity_bit = bin(emitter_id).count('1') % 2
			final_emitter_id = emitter_id
			if parity_bit == 1:
				final_emitter_id ^= 1  # Flip the last bit if parity is odd
			binary_message += f"{final_emitter_id:04b}"

			binary_messages.append(binary_message)
		
		return binary_messages
	
	

	def decode_message_binary(self, binary_message):
		"""
		Decode a binary string message encoded with the custom protocol.
		
		Args:
			binary_message (str): The encoded message as a binary string.
		
		Returns:
			dict: Decoded message components, including emitter ID, receiver IDs, and the message content.
		"""
		index = 0
		
		# Step 1: Extract Emitter ID (4 bits)
		emitter_id = int(binary_message[index:index+4], 2)
		index += 4
		
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
			for _ in range(num_receivers):
				receivers.append(int(binary_message[index:index+4], 2))
				index += 4


		
		
		# Step 4: Extract the message
		message = []
		while index < len(binary_message) - 8:  # Leave space for sum and parity bits
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
			else:
				raise ValueError(f"Unknown type indicator: {type_indicator}")
		
		# Step 5: Extract units of the sum (4 bits)
		sum_units = int(binary_message[index:index+4], 2)
		index += 4
		
		# Step 6: Extract emitter ID with parity bit (4 bits)
		emitter_id_with_parity = int(binary_message[index:index+4], 2)
		index += 4

		# Parity check (optional)
		parity_bit = bin(emitter_id).count('1') % 2
		is_parity_valid = ((emitter_id ^ emitter_id_with_parity) & 1) == parity_bit
		if not is_parity_valid:
			raise ValueError("Parity check failed.")
		
		return {
			"emitter_id": emitter_id,
			"receivers": receivers,
			"message": message,
			"sum_units": sum_units,
			"parity_valid": is_parity_valid
		}









	def decode_message():
		...

	def diffuse_signal():
		...

	def scan_signal():
		...

	def send_message():
		...

	def receive_message():
		...

	def draw_links():
		...


com = Comminicator(0, 3, 1000, 1000)

encoded = com.encode_message_binary(1, 'all', ["Bonjour", 1902.0, "Bonjour", 1902.0, "Bonjour", 1902.0, "Bonjour", 1902.0, "Bonjour", 1902.0])

print(encoded)
print(len(encoded))

for encod in encoded:
	decoded = com.decode_message_binary(encod)
	for d in decoded:
		print(d, decoded[d])
	print("\n##########\n")
