
import serial, serial.tools.list_ports, os, datetime, time

class packet(object):
	def __init__(self):
		self.address = []
		self.meterData = []
		self.panelID = None
		self.circuitID = None
		self.statusReg = None

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
	pktCounter = {}
	while True:
		receivedPacket = WaitForPacket()
		timeStamp = datetime.datetime.now()
		print "Time Stamp: {0:02d}/{1:02d} {2:02d}:{3:02d}:{4:02d}"\
		.format(timeStamp.date().month, timeStamp.date().day, \
		timeStamp.time().hour, timeStamp.time().minute, timeStamp.time().second)
		addrDecimal = addrAggregate(receivedPacket.address)
		if  addrDecimal not in  pktCounter:
			pktCounter[addrDecimal] = 0
		else:
			pktCounter[addrDecimal] += 1
		print "Packet Counter: {0}".format(pktCounter[addrDecimal])
		if receivedPacket.statusReg != None:
			print "External voltage waveform supplied: ",((receivedPacket.statusReg & int('0x80', 16)) > 0)
			print "External Battery Pack Attached:     ",((receivedPacket.statusReg & int('0x40', 16)) > 0)
			print "Three Phase Measurement:            ",((receivedPacket.statusReg & int('0x20', 16)) > 0)
			print "FRAM Write Enabled:                 ",((receivedPacket.statusReg & int('0x10', 16)) > 0)
				
		if receivedPacket.panelID != None:
			print "Panel ID: {0}".format(hex(receivedPacket.panelID))
			print "Circuit ID: {0}".format(receivedPacket.circuitID)
		else:
			print "Address: {0}".format(receivedPacket.address)
		print "Reading: {0} W\r\n".format(receivedPacket.meterData)
				

if __name__=="__main__":
	main()

