#!/usr/bin/env python2

import sys
from PIL import Image

image_filename = sys.argv[1]
array_name = sys.argv[2]

im = Image.open(image_filename)
rgb_im = im.convert('RGB')

print "uint32_t {}[] = ".format(array_name)
print "{"
for y in range(0, 24):
	print "/* {} */".format(y)
	for x in range(0, 24):
		r, g, b = rgb_im.getpixel((x, y))

		print "(({}/5) << 16) + (({}/5) << 8) + ({}/5),".format(r, g, b)
		#print "(({}) << 16) + (({}) << 8) + ({}),".format(r, g, b)
print "};"

