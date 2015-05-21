

import os, scipy
import matplotlib.pyplot as plt

def main():
	myDirArr = ['BK0', 'BK2', 'BK3', 'BK5', 'BK6', 'BK8']

	myData = []
	for mydir in myDirArr:
		defaultPath = 'data/'
		myPath =  defaultPath + mydir
		myFiles = os.listdir(myPath)
		myPower = []
		myAveragePower = []
		for myFile in myFiles:
			fp = open((defaultPath + mydir + '/' + myFile), 'r')
			mydata = []
			for line in fp:
				temp = line.split()
				mydata.append(int(temp[1]))
			fp.close()
			mydata = scipy.asarray(mydata)
			myPower.append(float(myFile[1:myFile.find('_')]))
			myAveragePower.append(scipy.mean(mydata)/1000) # convert to W
		myAveragePower = [x for (y,x) in sorted(zip(myPower, myAveragePower))]
		myData.append([sorted(myPower), myAveragePower])

	'''
	myOutputFile = open('processed_data.txt', 'w')
	for mydir in myDirArr:
		myOutputFile.write("{0}\t\t\t".format("#"+mydir))
	myOutputFile.write("\r\n")

	for row in range(len(myData[0][0])):
		for column in range(len(myData)):
			myOutputFile.write("{0:.2f}\t{1:.2f}\t".format(myData[column][0][row], myData[column][1][row]))
		myOutputFile.write("\r\n")
	myOutputFile.close()
	'''
	dataCnt = 0
	for mydir in myDirArr:
		myOutputFile = open(mydir+'proc.txt', 'w')
		myOutputFile.write("{0}\t\t{1}\r\n".format("#"+mydir, "measured"))
		for row in range(len(myData[dataCnt][0])):
			myOutputFile.write("{0:.2f}\t{1:.2f}\r\n".format(myData[dataCnt][0][row], myData[dataCnt][1][row]))
		myOutputFile.close()
		dataCnt += 1
	

if __name__=="__main__":
	main()
