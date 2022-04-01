import cv2
import numpy as np

#img = cv2.imread("line_hsv2.png")

vid_capture = cv2.VideoCapture('test.mkv')

fps = vid_capture.get(5)

mask = ((23, 43, 175), (56, 173, 255))

while(vid_capture.isOpened()):

    ret, frame = vid_capture.read()
    if ret == True:
    
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hsv = cv2.inRange(hsv, mask[0], mask[1])

        cnts, hierarchy = cv2.findContours(hsv.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        try:
            hierarchy = hierarchy[0] # get the actual inner list of hierarchy descriptions
        except Exception as e:
            continue

        # For each contour, find the bounding rectangle and draw it
        for component in zip(cnts, hierarchy):
            currentContour = component[0]
            currentHierarchy = component[1]
            x,y,w,h = cv2.boundingRect(currentContour)
            if currentHierarchy[2] < 0:
                # these are the innermost child components
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),3)
            elif currentHierarchy[3] < 0:
                # these are the outermost parent components
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),3)

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