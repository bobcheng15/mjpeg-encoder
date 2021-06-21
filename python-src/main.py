from PIL import Image
import numpy as np

im = Image.open('frame0.bmp')
block_width = int(im.width / 8)
block_height = int(im.height / 8)
rgb_pixels = list(im.getdata())

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

yuv_blocks = []
yuv_block = []
yuv_pixels = []
for block in blocks:
	for pixels in block:
		for pixel in pixels:
			transfrom_metrix = [[0.257, 0.504, 0.098], [-0.148, -0.291, 0.439], [0.439, -0.368, -0.071]]
			yuv = np.dot(transfrom_metrix, pixel) + [16, 128, 128]
			yuv_pixels.append(yuv)
		yuv_block.append(yuv_pixels)
		yuv_pixels = []
	yuv_blocks.append(yuv_block)
	yuv_block = []

print(np.shape(yuv_blocks))
