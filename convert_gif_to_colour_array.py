from PIL import Image

#im = Image.open('mm-24.gif')
im = Image.open('defcon-smiley.gif')
rgb_im = im.convert('RGB')

print "{"
for y in range(0, 24):
	print "/* {} */".format(y)
	for x in range(0, 24):
		r, g, b = rgb_im.getpixel((x, y))

		print "(({}/20) << 16) + (({}/20) << 8) + ({}/20),".format(r, g, b)
print "};"

