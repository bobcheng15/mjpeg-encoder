from PIL import Image
import numpy as np

im = Image.open('frame0.bmp')
block_width = int(im.width / 8)
block_height = int(im.height / 8)
rgb_pixels = list(im.getdata())
print(block_width)
print(block_height)

temp = []
block = []
blocks = []
for i in range(block_height):
	for j in range(block_width):
		for m in range(8):
			for n in range(8):
				address = (i*8+m)*im.width + (j*8) + n
				temp.append(rgb_pixels[address])
		block.append(temp)
		temp = []
	blocks.append(block)
	block = []


print(np.shape(blocks))