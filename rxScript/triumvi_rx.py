
import serial, serial.tools.list_ports, os, datetime, time

class packet(object):
	def __init__(self):
		self.address = []
		self.meterData = []

def WaitForPacket():
	global ser
	readChar = ''
	newPacket = packet()
	while True:
		if ser.inWaiting() != 0: 
			readChar += ser.read(1)
			if readChar[-1]=='\n':
				temp = readChar.split()
				readChar = ''
				if len(temp):
					if temp[0]=='Received':
						newPacket.address = temp[4:]
						addressReceived = True
					elif addressReceived==True and temp[0]=='Meter':
						newPacket.meterData = float(temp[2])/1000
						break
		else:
			time.sleep(0.5)
	return newPacket

def addrAggregate(address):
	result = 0
	for x in address:
		result = result*256 + int(x, 16)
	return result


def main():
	global ser
	allPorts = serial.tools.list_ports.comports()
	print "Find the following ports on the system"
	for i in range(len(allPorts)):
		print "{0}: {1} ({2})".format(i, allPorts[i][0], allPorts[i][1])
	selection = int(raw_input("\n Enter selection: "))
	ser = serial.Serial(allPorts[selection][0], 115200, timeout = 0)  # open serial port
	pktCounter = {}
	while True:
		receivedPacket = WaitForPacket()
		timeStamp = datetime.datetime.now()
		print "Time Stamp: {0}/{1} {2}:{3}:{4}"\
		.format(timeStamp.date().month, timeStamp.date().day, \
		timeStamp.time().hour, timeStamp.time().minute, timeStamp.time().second)
		addrDecimal = addrAggregate(receivedPacket.address)
		if  addrDecimal not in  pktCounter:
			pktCounter[addrDecimal] = 0
		else:
			pktCounter[addrDecimal] += 1
		print "Packet Counter: {0}".format(pktCounter[addrDecimal])
		print "Address: {0}".format(receivedPacket.address)
		print "Reading: {0} W\r\n".format(receivedPacket.meterData)
				

if __name__=="__main__":
	main()

