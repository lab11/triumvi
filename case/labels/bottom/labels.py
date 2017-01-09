#!/usr/bin/env python

"""
Generate pdfs of labels for the bottom of Triumvi Cases
"""

import os
import sys
import svgutils.transform as sg

import sh
from sh import pdf2svg
from sh import rsvg_convert

MAIN_LABEL_SVGS = ['bottom_10.0.svg', 'bottom_10.1.svg']

POSITION_START_X = 0
POSITION_START_Y = 0
x = POSITION_START_X
y = POSITION_START_Y

label_specs = {}
label_specs['offset_x'] = 38
label_specs['gap_x']    = 0
label_specs['width_x']  = 72
label_specs['offset_y'] = 36
label_specs['gap_y']    = 9
label_specs['height_y'] = 54
label_specs['y_count']  = 9
label_specs['x_count']  = 10

def get_coordinates ():
	global x, y

	xpx = label_specs['offset_x'] + (x*(label_specs['gap_x'] + label_specs['width_x']))
	ypx = label_specs['offset_y'] + (y*(label_specs['gap_y'] + label_specs['height_y']))

	x += 1

	if x > label_specs['x_count']-1:
		x = 0
		y += 1

	return (round(xpx), round(ypx))

try:
	count = int(sys.argv[1])
	version = int(sys.argv[2])
except:
	print('{} <count> <version (0 or 1)>'.format(sys.argv[0]))

label_svg = MAIN_LABEL_SVGS[version]
label_pixels_x = 72
label_pixels_y = 54


label_sheet = sg.SVGFigure('792', '612') # 8.5"x11" paper at 72dpi
labels = []

for i in range(count):
	lbl = sg.fromfile(label_svg)
	lblr = lbl.getroot()
	pos = get_coordinates()
	lblr.moveto(pos[0], pos[1], 1) # position correctly (hand tweaked)
	labels.append(lblr)

label_sheet.append(labels)
base_name = 'triumvi_bottom_{}_{}'.format(count, version)
output_name = '{}.svg'.format(base_name)
label_sheet.save(output_name)
with open(output_name, 'r') as f:
	contents = f.read()
	contents = contents.replace(" standalone='yes'", "")
with open(output_name, 'w') as f:
	f.write(contents)
sh.rsvg_convert('-f', 'pdf', '-d', '72', '-p', '72', '-o',
	'{}.pdf'.format(base_name), '{}.svg'.format(base_name))
print('{}.pdf'.format(base_name))
