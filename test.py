import struct

item = 10
item = -100990

binary_message = ""
if 0 <= item < 256:
	int_bytes = struct.pack('>i', item)
	print(f"{item:08b}")
else:
	int_bytes = struct.pack('>i', item)
	print(bin(item)[2:])
	print(f"{item:032b}")
	# binary_message += ''.join(f"{byte:08b}" for byte in int_bytes)

print(binary_message)