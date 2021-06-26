from PIL import Image
import numpy as np

image = Image.open('frame0.bmp')
img_yuv = image.convert('YCbCr')
yuv_pixels = np.ndarray((image.size[1], image.size[0], 3), 'u1', img_yuv.tobytes())
# print(yuv_pixels.shape)

yuv_block = []
for pixel in yuv_pixels:
	yuv_block.append(pixel)
	
# print(yuv_block[719][1279][0])
