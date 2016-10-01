
import sys
from math import sqrt, sin, pi

def main():
    if len(sys.argv) != 2:
        print("Invalid parameter, useage: python voltage_gen.py amplitude\r\n")
        return
    fileName = 'sineTable_' + sys.argv[1] + 'v.h'
    outFile = open(fileName, 'w')
    outFile.write('/* table is generated with VRMS: {0} v */\r\n'.format(int(sys.argv[1])))
    outFile.write('const int stdSineTable[] = {\r\n')
    amplitude = sqrt(2)*int(sys.argv[1])
    for i in range(360):
        outFile.write('{0}'.format(int(round(amplitude*sin(i/180.0*pi)))))
        newLine = True if i % 12 == 11 else False
        if i == 359:
            outFile.write('};\r\n')
        else:
            outFile.write(', ')
            if newLine:
                outFile.write('\r\n')
    print('Successfully generated {0} v VRMS waveform\r\n'.format(sys.argv[1]))
    outFile.close()

if __name__=="__main__":
    main()
