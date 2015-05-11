
import math

def main():
	offset_phase = -33 # Degree
	degreeStep = 3.0078 # Degrees
	timeStep = 1.3925e-4
	gain = 1

	myOutfile = open('sineTable.txt', 'w')
	myOutfile2 = open('sineTableWOTime.txt', 'w')

	for i in range(120):
		degree = degreeStep*i + offset_phase
		sineValue = math.sin(degree/180*math.pi)
		sineValue *= 120*math.sqrt(2)*gain
		sineValue += 120*math.sqrt(2)*gain
		myOutfile.write("{0}\t{1}\r\n".format(timeStep*i, int(round(sineValue))))
		myOutfile2.write("{0}\r\n".format(int(round(sineValue))))
	myOutfile.close()
	myOutfile2.close()


if __name__=="__main__":
	main()
