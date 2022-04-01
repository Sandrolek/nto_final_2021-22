import cv2
import numpy as np

#img = cv2.imread("line_hsv2.png")

vid_capture = cv2.VideoCapture('real.mkv')

def nothing(x):
    pass

cv2.namedWindow('marking')

cv2.createTrackbar('H Lower','marking',0,179,nothing)
cv2.createTrackbar('H Higher','marking',179,179,nothing)
cv2.createTrackbar('S Lower','marking',0,255,nothing)
cv2.createTrackbar('S Higher','marking',255,255,nothing)
cv2.createTrackbar('V Lower','marking',0,255,nothing)
cv2.createTrackbar('V Higher','marking',255,255,nothing)

fps = vid_capture.get(5)

while(vid_capture.isOpened()):

    ret, frame = vid_capture.read()
    if ret == True:
        cv2.imshow('Look', frame)
        
        # print('Кадр {0:04d}'.format(file_count))

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hL = cv2.getTrackbarPos('H Lower','marking')
        hH = cv2.getTrackbarPos('H Higher','marking')
        sL = cv2.getTrackbarPos('S Lower','marking')
        sH = cv2.getTrackbarPos('S Higher','marking')
        vL = cv2.getTrackbarPos('V Lower','marking')
        vH = cv2.getTrackbarPos('V Higher','marking')

        lower_value = np.array([hL, sL, vL], np.uint8)
        upper_value = np.array([hH, sH, vH], np.uint8)

        hsv = cv2.inRange(hsv, lower_value, upper_value)

        cv2.imshow("Res", hsv)
        cv2.imshow("Orig", frame)
        
        key = cv2.waitKey(50)
        
        if (key == ord('q')) or key == 27:
            break
    else:
        break

    

    if cv2.waitKey(10) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break

