
import serial, serial.tools.list_ports, os, datetime, time

class packet(object):
	def __init__(self):
		self.address = []
		self.meterData = []
		self.panelID = None
		self.circuitID = None
		self.statusReg = None
		self.packetArrivalTime = None

class threePhasePower(object):
	def __init__(self):
		self.power = []
		self.firstTimeStamp = None
		self.myDict = {}
		self.threshold = 1 # three phase packets have to arrive within 1 second
	def addPacket(self, ID, power, timeStamp):
		if self.firstTimeStamp != None:
			# Flushes all data
			if dateTimeDiffInSeconds(timeStamp, self.firstTimeStamp) > self.threshold:
				self.power = []
				self.myDict = {}
				self.firstTimeStamp = timeStamp
		else:
			self.firstTimeStamp = timeStamp
		self.power.append(power)
		self.myDict[ID] = True

		if len(self.myDict) < 3:
			return -1
		else:
			for key in self.myDict.keys():
				if self.myDict[key] == False:
					return -1
			result = sum(self.power)
			self.firstTimeStamp = None
			self.power = []
			self.myDict = {}
			return result

def dateTimeDiffInSeconds(a, b):
	timeDiff = a - b
	return timeDiff.seconds


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
						newPacket.packetArrivalTime = datetime.datetime.now()
					elif addressReceived == True and temp[0] == 'Panel':
						newPacket.panelID = int(temp[2], 16)
					elif addressReceived == True and temp[0] == 'Circuit':
						newPacket.circuitID = int(temp[2])
					elif addressReceived == True and temp[0] == 'Status':
						newPacket.statusReg = int(temp[2], 16)
					elif addressReceived==True and temp[0]=='Meter':
						newPacket.meterData = float(temp[2])/1000
						break
		else:
			time.sleep(0.1)
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
	uartPortDict = {}
	uartPortList = []
	for i in range(len(allPorts)):
		if 'UART' in allPorts[i][1]:
			uartPortDict[i] = len(uartPortList)
			uartPortList.append(i)
			print "{0}: {1} ({2})".format(uartPortDict[i], allPorts[i][0], allPorts[i][1])
	selection = int(raw_input("\n Enter selection: "))
	ser = serial.Serial(allPorts[uartPortList[selection]][0], 115200, timeout = 0)  # open serial port
	print ""
	myThreePhase = threePhasePower()
	pktCounter = {}
	while True:
		receivedPacket = WaitForPacket()
		timeStamp = receivedPacket.packetArrivalTime
		print "Time Stamp: {0:02d}/{1:02d} {2:02d}:{3:02d}:{4:02d}"\
		.format(timeStamp.date().month, timeStamp.date().day, \
		timeStamp.time().hour, timeStamp.time().minute, timeStamp.time().second)
		addrDecimal = addrAggregate(receivedPacket.address)
		if  addrDecimal not in  pktCounter:
			pktCounter[addrDecimal] = 0
		else:
			pktCounter[addrDecimal] += 1
		print "Packet Counter: {0}".format(pktCounter[addrDecimal])
		myPWR = 0
		if receivedPacket.statusReg != None:
			print "External voltage waveform supplied: ",((receivedPacket.statusReg & int('0x80', 16)) > 0)
			print "External Battery Pack Attached:     ",((receivedPacket.statusReg & int('0x40', 16)) > 0)
			print "Three Phase Measurement:            ",((receivedPacket.statusReg & int('0x30', 16)) > 0)
			if ((receivedPacket.statusReg & int('0x30', 16)) > 0):
				myID = (receivedPacket.statusReg & int('0x30', 16))>>4
				print "Three Phase Unit ID: {0}".format(myID)
				myPWR = myThreePhase.addPacket(myID, receivedPacket.meterData, timeStamp)
			else:
				myPWR = 0
			print "FRAM Write Enabled:                 ",((receivedPacket.statusReg & int('0x08', 16)) > 0)
				
		if receivedPacket.panelID != None:
			print "Panel ID: {0}".format(hex(receivedPacket.panelID))
			print "Circuit ID: {0}".format(receivedPacket.circuitID)
		else:
			print "Address: {0}".format(receivedPacket.address)
		print "Reading: {0} W\r\n".format(receivedPacket.meterData)
		if myPWR > 0:
			print "\nThree Phase Power: {0}\n".format(myPWR)
				

if __name__=="__main__":
	main()

