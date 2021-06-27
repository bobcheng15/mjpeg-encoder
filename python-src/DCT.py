import numpy as np
import cv2
from PIL import Image
import math
import sys
import progressbar
import numba
def computeDCTComponent(channelBlock, row_index, col_index):
    result = 0
    for i in range(0, 8):
        for j in range(0, 8):
            tmp = math.cos(math.pi * row_index / (2 * 8) * (2 * i + 1)) * math.cos(math.pi * col_index / (2 * 8) * (2 * j + 1))
            tmp = tmp * channelBlock[i][j]
            result = result + tmp
    # if (row_index == 0):
    #     result = result * 1 / math.sqrt(2)
    # if (col_index == 0):
    #     result = result * 1 / math.sqrt(2)
    result = result / 4
    return result
def TwoDDCT(inputMatrix, num_coef):
    coef_range = int(math.sqrt(num_coef))
    result = np.zeros((8, 8, 3))
    for channel in range(0, 3):
        for index_i in range(0, coef_range):
            for index_j in range(0, coef_range):
                print(index_i, index_j)
                result[index_i][index_j][channel] = computeDCTComponent(inputMatrix[:,:,channel], index_i, index_j)
    return result
def computeInvDCTComponent(channelBlock, row_index, col_index):
    result = 0
    for i in range(0, 8):
        for j in range(0, 8):
            tmp = math.cos(math.pi * i / (2 * 8) * (2 * row_index + 1)) * math.cos(math.pi * j / (2 * 8) * (2 * col_index + 1))
            tmp = tmp * channelBlock[i][j]
            # if (i == 0):
            #     tmp = tmp * 1 / math.sqrt(2)
            # if (j == 0):
            #     tmp = tmp * 1 / math.sqrt(2)
            result = result + tmp
    result = result / 4
    return int(result)
def InvTwoDDCT(inputMatrix):
    result = np.zeros((8, 8, 3))
    for channel in range(0, 3):
        for index_i in range(0, 8):
            for index_j in range(0, 8):
                result[index_i][index_j][channel] = computeInvDCTComponent(inputMatrix[:,:,channel], index_i, index_j)
    return result

def MSE(original, processed):
    yError = ((original[:, :, 0] - processed[:, :, 0])**2).sum()
    CbError = ((original[:, :, 1] - processed[:, :, 1])**2).sum()
    CrErrpr = ((original[:, :, 2] - processed[:, :, 2])**2).sum()
    mse = (yError + CbError + CrErrpr) / (3 * 877 * 1400)
    return mse

def PSNR(original, processed):
    mse = MSE(original, processed)
    result = 20 * np.log10(255/math.sqrt(mse))
    return result



if __name__ == "__main__":
    image = Image.open('0006.jpg')
    img_yuv = image.convert('YCbCr')
    img = np.array(img_yuv)
    print(img)
    result = np.zeros((32, 32, 3))
    invDCTResult = np.zeros((32, 32, 3))
    invResult = np.zeros((32, 32, 3))
    count = 0
    print("Running DCT on the Original Image")
    bar = progressbar.ProgressBar(maxval=1024, \
    widgets=[progressbar.Bar('=', '[', ']'), 'DCT', progressbar.Percentage()])
    bar.start()
    img_dump = []
    result_dump = []
    for i in range(0, 32, 8):
        for j in range(0, 32, 8):
            count = count + 1
            bar.update(count)
            inputMatrix = np.zeros((8, 8, 3))
            if (i != 872):
                inputMatrix = img[i:i+8, j:j+8, :]
                img_dump.append(inputMatrix.tolist())
            else:
                inputMatrix[0:5, 0:8, :] = img[i:i+5, j:j+8, :]
                inputMatrix[5:8, 0:8, :] = np.zeros((3, 8, 3))


            result[i:i+8, j:j+8, :] = TwoDDCT(inputMatrix, int(sys.argv[1]))
            result_dump.append(result[i:i+8, j:j+8, :].tolist())
    bar.finish()
    count = 0
    print("Dumping yuv image")
    print(np.array(img_dump).shape)
    print(np.array(img_dump))
    np.savetxt("yuv_input.csv", np.array(img_dump).flatten(), delimiter=",", fmt='%d')
    print("Dumping DCT result")
    print(np.array(result_dump).shape)
    print(np.array(result_dump))
    np.savetxt("DCT_output" + sys.argv[1] + ".csv", np.array(result_dump).flatten(), delimiter=",", fmt='%f')
    print("Running IDCT")
    invbar = progressbar.ProgressBar(maxval=19250, \
    widgets=[progressbar.Bar('=', '[', ']'), 'IDCT', progressbar.Percentage()])
    invbar.start()
    for i in range(0, 32, 8):
        for j in range(0, 32, 8):
            count = count + 1
            invbar.update(count)
            inputMatrix = np.zeros((8, 8, 3))
            inputMatrix = result[i:i+8, j:j+8, :]
            invDCTResult[i:i+8, j:j+8, :] = InvTwoDDCT(inputMatrix)
    invbar.finish()
    invResult = invDCTResult[0:32, 0:32, :]
    resultImg = Image.fromarray(invResult.astype(np.uint8), mode = 'YCbCr')
    resultImg = resultImg.convert('RGB')
    print("PSNR: " + str(PSNR(img, invResult)))
    #resultImg.show()
    resultImg.save("DCT_result_"+ sys.argv[1] +".png")
