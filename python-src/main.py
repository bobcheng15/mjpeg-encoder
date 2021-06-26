from PIL import Image
import numpy as np

image = Image.open('frame0.bmp')
block_width = int(image.width / 8)    #160
block_height = int(image.height / 8)  #90

img_yuv = image.convert('YCbCr')
yuv_pixels = np.ndarray((image.size[1], image.size[0], 3), 'u1', img_yuv.tobytes())

temp = []
block = []
blocks = []
for i in range(block_height):
	for j in range(block_width):
		for m in range(8):
			for n in range(8):
				temp.append(yuv_pixels[i*8+m][j*8+n])
		block.append(temp)
		temp = []
	blocks.append(block)
	block = []
	
print(np.shape(blocks))
print(blocks[0][0])


# yuv_blocks = []
# yuv_block = []
# yuv_pixels = []
# for block in blocks:
	# for pixels in block:
		# for pixel in pixels:
			# transfrom_metrix = [[0.257, 0.504, 0.098], [-0.148, -0.291, 0.439], [0.439, -0.368, -0.071]]
			# yuv = np.dot(transfrom_metrix, pixel) + [16, 128, 128]
			# yuv_pixels.append(yuv)
		# yuv_block.append(yuv_pixels)
		# yuv_pixels = []
	# yuv_blocks.append(yuv_block)
	# yuv_block = []

# print(np.shape(yuv_blocks))
