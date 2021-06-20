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

yuv_blocks = []
yuv_block = []
yuv_row = []
for block in blocks:
	for row in block:
		for pixel in row:
			yuv_pixel = []
			transfrom_metrix = [[0.257, 0.504, 0.098], [-0.148, -0.291, 0.439], [0.439, -0.368, -0.071]]
			yuv = np.dot(transfrom_metrix, pixel) + [16, 128, 128]
			yuv_pixel.append(yuv)
		yuv_row.append(yuv_pixel)
	yuv_block.append(yuv_row)
	yuv_row = []
yuv_blocks.append(yuv_block)
yuv_block = []

return yuv_blocks