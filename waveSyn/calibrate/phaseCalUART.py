
import math, scipy, sys
import matplotlib.pyplot as plt
SAMPLE_PER_CYCLE = 120
CLOCK_FREQ = 16e6


# return the matched phase per cycle
def phaseMatch(myData, sampleTimeStep):
	#print "Time Step: {0}".format(sampleTimeStep)
	phasePerSample = float(sampleTimeStep)/(CLOCK_FREQ/60)*360
	#print "Calibrate from {0} of samples".format(len(myData))

	phaseOffsetArr = range(-180, 180, 1)
	myPowerArr = []
	for phaseOffset in phaseOffsetArr:
		myPower = []
		power = 0
		for j in range(len(myData)):
			currentPhase = phaseOffset + j*phasePerSample
			power += math.sin(currentPhase/180*math.pi)*myData[j]
		myPower.append(float(power))
		#print "Average power for phase offset {0} is {1}".format(phaseOffset, scipy.mean(scipy.asarray(myPower)))
		myPowerArr.append([phaseOffset, scipy.mean(scipy.asarray(myPower))])
	
	sortedPower = sorted(myPowerArr, key=lambda x: x[1], reverse=True)
	phaseOffset = sortedPower[0][0]
	return phaseOffset
	

def main():
	myFile = open(sys.argv[1], 'r')
	myData = []
	packetData = []
	sampleTimeStep = 0
	lineNum = 1
	phaseCalculated = []
	for line in myFile:
		temp = line.split()
		lineNum += 1
		if len(temp): # non empty line
			if temp[1]=='reference:':
				reference = int(temp[2])
				if len(packetData) == SAMPLE_PER_CYCLE:
					myData.append(packetData)
					phaseCalculated.append(phaseMatch(packetData, sampleTimeStep))
				packetData = []
			elif temp[1]=='difference:':
				sampleTimeStep = int(temp[2])
			elif temp[1]=='reading:':
				#print lineNum
				if len(temp) == 3:
					packetData.append(int(temp[2]) - reference)
	myFile.close()
	print("Max phase offset: {0}".format(max(phaseCalculated)))
	print("Min phase offset: {0}".format(min(phaseCalculated)))
	print("average phase offset: {0}".format(float(sum(phaseCalculated))/len(phaseCalculated)))

	phaseOffset = float(sum(phaseCalculated))/len(phaseCalculated)
	print("using phase offset: {0}".format(phaseOffset))
	phasePerSample = float(sampleTimeStep)/(CLOCK_FREQ/60)*360

	myVoltage = [360*math.sin((phaseOffset + phasePerSample*x)/180*math.pi) for x in range(len(myData[0]))]
	xaxis = [x for x in range(len(myData[0]))]

	myOutFile = open(sys.argv[1][:sys.argv[1].find('Cal')] + 'SineTable.txt', 'w')
	myOutFile.write("int sineTable[BUF_SIZE] = {\r\n")
	for i in range(SAMPLE_PER_CYCLE):
		if i == SAMPLE_PER_CYCLE-1:
			myOutFile.write("{0}".format(int(round(SAMPLE_PER_CYCLE*math.sqrt(2)*(math.sin((phaseOffset+phasePerSample*i)/180*math.pi))))))
			myOutFile.write("};")
		else:
			myOutFile.write("{0}, ".format(int(round(SAMPLE_PER_CYCLE*math.sqrt(2)*(math.sin((phaseOffset+phasePerSample*i)/180*math.pi))))))
		if i%8==0 and i!=0:
			myOutFile.write("\r\n")
	myOutFile.close()
		
	
	plt.plot(xaxis, myVoltage)
	for i in range(len(myData)):
		plt.plot(xaxis, myData[i])
	plt.show()
	plt.grid()
		

			
			

if __name__=="__main__":
	main()
