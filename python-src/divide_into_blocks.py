from PIL import Image
import numpy as np

im = Image.open('frame0.bmp')
block_width = int(im.width / 8)
block_height = int(im.height / 8)
rgb_pixels = list(im.getdata())

temp = []
blocks = []
for i in range(block_height):
	for j in range(block_width):
		for m in range(8):
			address = (i*8+m)*im.width + (j*8)
			temp.append(rgb_pixels[address: address+8])
	blocks.append(temp)
	temp = []


print(np.shape(blocks))