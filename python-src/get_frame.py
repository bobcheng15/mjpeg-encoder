import cv2
vidcap = cv2.VideoCapture('big_buck_bunny_720p_5mb.mp4')
success,image = vidcap.read()
count = 0
while success:
  cv2.imwrite("frame%d.bmp" % count, image)     # save frame as JPEG file      
  success,image = vidcap.read()
  count += 1

