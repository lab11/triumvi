
import sys, numpy

def main():
	myFile = open(sys.argv[1], 'r')
	myData = []
	for line in myFile:
		temp = line.split()
		myData.append(int(temp[2]))
	myFile.close()


	myDataDiff = []
	#myOutFile = open(sys.argv[2], 'w')
	for i in range(1, len(myData)):
		#myOutFile.write("{0}\r\n".format(myData[i]-myData[i-1]))
		myDataDiff.append(myData[i]-myData[i-1])
	#myOutFile.close()

	x = numpy.array(myDataDiff)
	print 'Mean: ', numpy.mean(x), 'Std: ', numpy.std(x)


if __name__=="__main__":
	main()
