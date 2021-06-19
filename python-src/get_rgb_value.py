from PIL import Image
import numpy as np

im = Image.open('frame0.bmp')
rgb_pixels = list(im.getdata())

#r_pixels   = list(im.getdata(band = 0))
#g_pixels   = list(im.getdata(band = 1))
#b_pixels   = list(im.getdata(band = 2))

yuv_pixels = []
transfrom_metrix = [[0.257, 0.504, 0.098], [-0.148, -0.291, 0.439], [0.439, -0.368, -0.071]]
for pixel in rgb_pixels:
	yuv = np.dot(transfrom_metrix, pixel) + [16, 128, 128]
	yuv_pixels.append(yuv)

print(rgb_pixels[:10])
print(yuv_pixels[:10])