
import math

def main():
	myDataSetsCurrent = []
	myDataSetsTime = []
	myDataTemp = []
	myDataSetsTimeCollected = False
	myFile = open('timeCapture.txt', 'r')
	timePrev = float('inf')
	timeOffset = 3; # use this to compensate the offsets between phase measurement and real measurement
	for line in myFile:
		temp = line.split()
		if len(temp):
			time = int(temp[2]) + timeOffset;
			current = int(temp[5])
			# new data set
			if time < timePrev:
				if len(myDataTemp):
					myDataSetsCurrent.append(myDataTemp)
					myDataSetsTimeCollected = True
				myDataTemp = []
			myDataTemp.append(current)
			if not myDataSetsTimeCollected:
				myDataSetsTime.append(time)
			timePrev = time
	myFile.close()

	# check if time difference between samples are consistent
	timeDiffbtwSamples = myDataSetsTime[1] - myDataSetsTime[0]
	for i in range(1, len(myDataSetsTime)):
		if (myDataSetsTime[i] - myDataSetsTime[i-1]) != timeDiffbtwSamples:
			print "Error!! Time inconsistent"
			return 0
	
	# check if all datasets have the same size
	for i in range(len(myDataSetsCurrent)):
		if len(myDataSetsCurrent[i]) != len(myDataSetsCurrent[0]):
			print "Error!! Same size inconsistent"
			return 0
	
	clockFreq = 16e6
	degreeStep = (float(timeDiffbtwSamples)/clockFreq)*360*60
	
	phaseStart = -58
	phaseStop = -53
	phaseStepSize = 0.25
	phaseEnergy = []
	for j in range(int(round((phaseStop-phaseStart)/phaseStepSize))):
		phaseOffset = phaseStart + phaseStepSize*j
		energyArr = []
		for current in myDataSetsCurrent:
			energy = 0
			for i in range(len(current)):
				degree = phaseOffset + i*degreeStep
				sineValue = 120*math.sqrt(2)*math.sin(degree/180*math.pi)
				energy += sineValue*(-1*current[i]) # reverse current
			energyArr.append(energy)
		phaseEnergy.append([phaseOffset, sum(energyArr)/len(energyArr)])

	# remove minimum
	temp = []
	for data in phaseEnergy:
		temp.append(data[1])
	
	for data in phaseEnergy:
		data[1] -= min(temp)
		
	print phaseEnergy

	print "Use -57 degree as phase offset..."

	myOutfile = open('sineTable.txt', 'w')
	myOutfile2 = open('sineTableWOTime.txt', 'w')

	phaseOffset = -57
	for i in range(len(myDataSetsCurrent[0])):
		degree = phaseOffset + i*degreeStep
		sineValue = 120*math.sqrt(2)*math.sin(degree/180*math.pi)
		sineValue += 120*math.sqrt(2) # offset to all positive
		myOutfile.write("{0}\t{1}\r\n".format(timeDiffbtwSamples/clockFreq*i, int(round(sineValue))))
		if i%8==7:
			myOutfile2.write("{0},\r\n".format(int(round(sineValue))))
		else:
			myOutfile2.write("{0}, ".format(int(round(sineValue))))
	myOutfile.close()
	myOutfile2.close()

if __name__=="__main__":
	main()
