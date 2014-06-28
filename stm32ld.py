#!/usr/bin/env python

import serial, struct, os, time, sys
import argparse, hashlib

parser = argparse.ArgumentParser(description="Boot loader for stm32.")
parser.add_argument("--port", "-p", default="/dev/ttyUSB0",help="Serial device")
parser.add_argument("--baudrate", "-b", default=115200, help="Baudrate")
parser.add_argument("--image", "-i", default="test.bin", help="Binary image name")

# TO-DO: make these arguments do stuff.
parser.add_argument("--boot_enable", "-e",help="Use GPIO to put the device in boot mode.")
parser.add_argument("--boot-line", "-l", default=112, help="GPIO for boot line.")
parser.add_argument("--reset-line", "-r", default=115, help="GPIO for reboot line.")

args = parser.parse_args()

ACK = "\x79"
GET = "\x00"
GID = "\x02"
ERASE = "\x44"
WPUN = "\x73"
WM = "\x31"
GO = "\x21"
RPUN = "\x92"
RM = "\x11"

STM32_FLASH_START_ADDRESS = "\x08\x00\x00\x00"
FLASH_SIGNATURE_ADDRESS = "\x08\x0F\xFF\xF0"

PACKET_SIZE = 256

printHex = lambda x: map(hex,map(ord,x))

class loader:
	def __init__(self,port="/dev/ttyO4",baudrate=115200):
		if not os.path.exists("/dev/ttyO4"):
			os.system("echo \"BB-UART4\" > /sys/devices/bone_capemgr.9/slots")
		if not os.path.exists("/sys/class/gpio/gpio112"):
			os.system("echo 112 > /sys/class/gpio/export")
		if not os.path.exists("/sys/class/gpio/gpio115"):
			os.system("echo 115 > /sys/class/gpio/export")
		self.serialPort = serial.Serial(port=port,baudrate=baudrate,parity=serial.PARITY_EVEN,stopbits=1,bytesize=serial.EIGHTBITS, timeout=1)
		self.commands = "\x00"
		self.supportedChips = ["\x04\x13"]
	
	def bootMode(self):
		# Raise boot line.
		os.system("echo high > /sys/class/gpio/gpio112/direction")
		time.sleep(.5)
		# Cycle reboot line
		os.system("echo low > /sys/class/gpio/gpio115/direction")
		time.sleep(.5)
		os.system("echo high > /sys/class/gpio/gpio115/direction")
		time.sleep(0.5)
	
	def bootLow(self):
		# Lower the boot line.
		os.system("echo low > /sys/class/gpio/gpio112/direction")
		time.sleep(.5)

	def reset(self):
		# Ensure that the boot line is not high.
		os.system("echo low > /sys/class/gpio/gpio112/direction")
		time.sleep(0.5)
		# Cycle reboot line
		os.system("echo low > /sys/class/gpio/gpio115/direction")
		time.sleep(.5)
		os.system("echo high > /sys/class/gpio/gpio115/direction")

	def byteToHex(self, datum):
		return hex(struct.unpack("B",datum)[0])
		
	def checksum(self,data):
		#return reduce(lambda x,y:x+y, map(ord, data))
		cksm = 0
		for datum in data:
			cksm ^= ord(datum)
		return cksm & 255
	
	def command(self,command):
		if not command in self.commands:
			print "Error: command not in command set!"
			return -1
		if command == GET:
			return self.getVersion()
		elif command == GID:
			return self.getID()
		elif command == ERASE:
			return self.erase()
		elif command == WPUN:
			return self.writeUnprotect()
		elif command == RPUN:
			return self.readUnprotect()
		elif command == GO:
			return self.go()

	def connectToBl(self):
		# Flush the input.
		self.serialPort.flushInput()
		# Send the initialize command 0x7F
		bytesWritten = self.serialPort.write("\x7F")
		if bytesWritten < 1:
			print "Failed to write init byte."
			return False
		connected = False
		for i in range(3):
			time.sleep(.1)
			data = self.serialPort.read(self.serialPort.inWaiting())
			if "\x79" in data:
				connected = True
				break
		if not connected:
			print "Failed to connect!"
			return False
		else:
			return True
		while(serialPort.inWaiting < 3):
			continue
	
	def expect(self, data):
		dataRead = self.serialPort.read(len(data))
		if data in dataRead:
			return True
		else:
			print "Expected {0}, received {1}".format(map(hex,map(ord,data)),map(hex,map(ord,dataRead)))
			return False
			
		return data in dataRead

	def getVersion(self):
		# send the GET command
		self.serialPort.write("\x00\xFF")
		if not self.expect(ACK):
			return -1
		numberOfCommands = struct.unpack("B",self.serialPort.read())[0]
		versionInt = struct.unpack("B",self.serialPort.read())[0]
		version = (versionInt >> 4) + .1*(versionInt & 15)
		self.commands = self.serialPort.read(numberOfCommands)
		if not self.expect(ACK):
			print "Failed to ACK after GET command."
			return -1
		else:
			return version
	
	def getID(self):
		# send the GID command.
		self.serialPort.write("\x02\xFD")
		if not self.expect(ACK):
			return -1
		numberOfBytes = struct.unpack("B",self.serialPort.read())[0] + 1
		productID = self.serialPort.read(numberOfBytes)
		if (not self.expect(ACK)) or (len(productID) < numberOfBytes):
			print "Failed to ACK after GID command."
			return -1
		else:
			return productID
		
	# TO-DO: read memory.

	def erase(self):
		self.serialPort.write("\x44\xBB")
		if not self.expect(ACK):
			return -1
		else:
			print "About to globally erase.  This will take a sec (20 sec timeout)"
		self.serialPort.write("\xFF\xFF\00")
		self.serialPort.timeout = 20
		if not self.expect(ACK):
			self.serialPort.timeout = 1
			return -1
		else:
			self.serialPort.timeout = 1
			return 1
	
	def go(self):
		self.serialPort.write("\x21\xDE")
		if not self.expect(ACK):
			print "No ACK from Go command."
			return -1
		address = STM32_FLASH_START_ADDRESS
		checksum = self.checksum(address) 
		for datum in address:
			self.serialPort.write(datum)
		self.serialPort.write(struct.pack("B",checksum))
		
		if not self.expect(ACK):
			print "GO address ACK failed."
			return -1
		return 1

	def hashFile(afile, hasher, blocksize=65536):
		buf = afile.read(blocksize)
		while len(buf) > 0:
			hasher.update(buf)
			buf = afile.read(blocksize)
		return hasher.digest()
	
	def readBlock(self,address, length):
		# Send read command.
		print "In waiting before command:", self.serialPort.inWaiting()
		self.serialPort.write("\x11\xEE")
		print "In waiting after command:", self.serialPort.inWaiting()
		if not self.expect(ACK):
			print "Failed to ACK pre read."
			return -1
		print "In waiting after 1st ACK:", self.serialPort.inWaiting()

		#Send address + checksum
		checksum = self.checksum(address) 
		for datum in address:
			self.serialPort.write(datum)
		self.serialPort.write(struct.pack("B",checksum))
		print "In waiting address and checksum:", self.serialPort.inWaiting()
		
		if not self.expect(ACK):
			print "Read address ACK failed."
			return -1
		print "In waiting after 2nd ACK:", self.serialPort.inWaiting()
		
		#Send length + checksum
		self.serialPort.write(struct.pack("B",(length - 1)))
		self.serialPort.write(struct.pack("B",~(length - 1) & 0xFF))
		print "In waiting after length and checksum:", self.serialPort.inWaiting()
		
		if not self.expect(ACK):
			print "Read length ACK failed."
			return -1
		print "In waiting after 3rd ACK:", self.serialPort.inWaiting()
		
		
		return self.serialPort.read(length)

		

	def readUnprotect(self):
		# sent the command.
		# Note: system will go into reset after this command.
		self.serialPort.write("\x92\x6D")
		if not self.expect(ACK):
			return -1
		else:
			print "About about to read unprotect."
		if not self.expect(ACK):
			return -1
		else:
			return 1

	def resetRelease(self):
		# Release line
		os.system("echo in > /sys/class/gpio/gpio115/direction")
	
	def resetHold(self):
		# Make the line an output again
		#os.system("echo in > /sys/class/gpio/gpio115/direction")
		os.system("echo high > /sys/class/gpio/gpio115/direction")
	
	def write(self,image):
		# See how long the input file is.
		imageFile = open(image)
		imageSize = os.stat(image).st_size
		written = 0
		while True:
			# Write the image block by block
			address = struct.pack(">I",struct.unpack(">I",STM32_FLASH_START_ADDRESS)[0] + written)
			
			# Send data size, data, and checksum.
			writeSize = imageSize - written
			if writeSize > PACKET_SIZE:
				writeSize = PACKET_SIZE
			data = imageFile.read(writeSize)
			written += len(data)
			if self.writeBlock(data, address) < 1:
				print "Failed to write block", address
			# Print progress
			progress = written*100.0 / imageSize
			sys.stdout.write("\r%d%%" %progress)
			sys.stdout.flush()
			if written >= imageSize:
				print ""
				break
		return 1

	def writeBlock(self,data, address):			
		# Send write command.
		self.serialPort.write("\x31\xCE")
		if not self.expect(ACK):
			print "Failed to ACK pre write."
			return -1

		#Send address + checksum
		checksum = self.checksum(address) 
		for datum in address:
			self.serialPort.write(datum)
		self.serialPort.write(struct.pack("B",checksum))
		
		if not self.expect(ACK):
			print "Address ACK failed."
			return -1
		
		if len(data) == 0:
			print "Error.  Data size is zero."
			return -1
		data = struct.pack("B",len(data) - 1)[0] + data
		checksum = self.checksum(data) 
		for datum in data:
			self.serialPort.write(datum)
		self.serialPort.write(struct.pack("B",checksum))
		
		if not self.expect(ACK):
			print "Failed to ACK post write."
			return -1
		return 1


	def writeUnprotect(self):
		# Unprotect memory.
		self.serialPort.write("\x73\x8C")
		if not self.expect(ACK):
			return -1
		else:
			print "About about to write unprotect."
		if not self.expect(ACK):
			return -1
		else:
			return 1
		

		




# sp = serial.open(port="/dev/tty04",baudrate=115200,parity=serial.PARITY_EVEN,stopbits=1)


if __name__ == "__main__":
	ldr = loader(port=args.port, baudrate=args.baudrate)
	
	# Boot the device into BL mode.
	ldr.reset()
	ldr.bootMode()
	
	# Connect to the boot loader.
	if(ldr.connectToBl()):
		print "Connected to bootloader."
	else:
		exit(-1)
	
	# Get the boot loader version.
	version = ldr.command(GET)
	if version < 0:
		print "Failed to GET"
		exit(-1)
	else:
		print "Boot loader version:", version
	
	# Get the product ID.
	productID = ldr.command(GID)
	if productID < 0:
		print "Failed to get product ID."
		exit(-1)
	elif productID not in ldr.supportedChips:
		print "{0} not supported.".format(printHex(productID))
	else:
		print "Product ID:", "".join(printHex(productID))

	if args.image:
		# If flashing, erase and flash.
		if (ldr.command(ERASE) > 0):
			print "erase successful."
		else:
			print "Failed to erase device."
			exit(-1)	
		
		# Clear write protection.
		ldr.resetRelease()
		if (ldr.command(WPUN) > 0):
			print "Cleared write protection."
		else:
			print "Failed to clear write protection."
			exit(-1)
		
		# reset the reset line
		ldr.resetHold()
		# Give her a bit to wake up.
		time.sleep(.5)
			
		# Connect to the boot loader.
		if(ldr.connectToBl()):
			print "Connected to bootloader."
		else:
			exit(-1)
		
		# Write the image. 
		print "Writing image."
		if (ldr.write(args.image) > 0):
			print "Write successful."
		else:
			print "Failed to write."
			exit(-1)
		
		# Send the go command to make the stuff run.
		if (ldr.command(GO) > 0):
			print "GO successful."
		else:
			print "No GO."
			exit(-1)
		
	
	ldr.bootLow()
	
	ldr.serialPort.close()
	exit(0)


	
	



