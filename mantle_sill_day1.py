import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from sensor_msgs.msg import Range
from mavros_msgs.srv import CommandBool

from pyzbar.pyzbar import decode

import numpy as np
import math

import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('find_spots')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

debug = rospy.Publisher('debug', Image, queue_size=1)
center_pub = rospy.Publisher('center', Image, queue_size=1)
spot_pub = rospy.Publisher('oil_detect', Image, queue_size=1)

bridge = CvBridge()

HEIGHT = 240
WIDTH = 320

FLY_HEIGHT = 0.8

#mask = ((32, 45, 128), (65, 189, 255))
mask = ((20, 120, 0), (40, 140, 255))

k = 0.25 / 2300

SPEED_X = 0.08

defects = []

def navigate_wait(x=0, y=0, z=FLY_HEIGHT, yaw=1.57, speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2): # gop to pouint
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def home(xy):     # landing func
    s = list(map(float, xy)) 
    navigate_wait(x = s[0], y= s[1], z = 2)
    rospy.sleep(3)
    print("Landing")
    land()
    rospy.sleep(4)
    arming(False)

def convert_angle(angle=0, w_min=320, h_min=240): # from body copter to aruco angle

    if angle < -45:
        angle = 90 + angle
    if w_min < h_min and angle > 0:
        angle = (90 - angle) * -1
    if w_min > h_min and angle < 0:
        angle = 90 + angle

    return angle

def get_qr():  # getting qr code

    qr = None
    while qr == None:
        data = rospy.wait_for_message('main_camera/image_raw_throttled', Image)
    
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')

        barcodes = decode(cv_image) 
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect  
            qr = barcode.data

    return qr

def get_rect_top(img, orig): # finding rects in top part of img
    
    cnts, _ = cv2.findContours(img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts.sort(key=cv2.minAreaRect)

    (x_min, y_min), (w_min, h_min), angle = (0, 0), (0, 0), 0

    if len(cnts) > 0:
        cnt = max(cnts, key=cv2.contourArea)

        if cv2.contourArea(cnt) > 100:
            #state = 
            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect

            print(angle)

            #y_min += TOP_X
            #x_min += TOP_X - 40

            box = cv2.boxPoints(((x_min, y_min), (w_min, h_min), angle)) # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(orig, [box], 0, (255, 0, 0), 2)

            angle = convert_angle(angle=angle, w_min=w_min, h_min=h_min)

            return True, [(x_min, y_min), (w_min, h_min), angle]
        
    
    return False, []

def get_rect_full(img, orig): # getting rect to follow
    
    cnts, _ = cv2.findContours(img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts.sort(key=cv2.minAreaRect)

    if len(cnts) > 0:
        #cnt = cnts[0]
        cnt = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(cnt) > 50:
            #state = 
            #print("FULL", random.randint(0, 100))

            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect

            # x_min += FULL_X_B
            # y_min += FULL_Y_B

            box = cv2.boxPoints(((x_min, y_min), (w_min, h_min), angle)) # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(orig, [box], 0, (0, 255, 0), 2)

            angle = convert_angle(angle=angle, w_min=w_min, h_min=h_min)

            return True, [(x_min, y_min), (w_min, h_min), angle]
            
    return False, []

def img_cb(data): # lining
    global state

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, mask[0], mask[1])
    #cv2.imshow("BW image", black)

    top = black[:(HEIGHT // 2), :]
    
    is_full, rect_full = get_rect_full(black, cv_image)
    
    is_top, rect_top = get_rect_top(top, cv_image)

    # top_pub.publish(bridge.cv2_to_imgmsg(top, 'mono8'))

    if is_full: # just line
        #print("Lining")
        (x_min, y_min), (w_min, h_min), angle = rect_full

        center = cv_image.shape[1] / 2
        error = x_min - center

        #print(f"{x_min}, {y_min}, {w_min}, {h_min}, {round(angle, 2)}, {error}")

        cv2.circle(cv_image, (int(x_min), int(y_min)), 5, (0, 0, 255), 3)

        telem = get_telemetry(frame_id="aruco_map")

        #print(round(angle, 2), error)
        set_velocity(vx=SPEED_X, vy=error*(-0.008), vz=-(telem.z - FLY_HEIGHT)*0.5, yaw=float('nan'), yaw_rate=angle*(-0.008), frame_id='body')

    if not is_top: # no line, go home
        print("Staying")

        state = False

    #print("debug")
    debug.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    #print("end")
    #cv2.imshow("Original image", cv_image)

    #cv2.waitKey(1)


def defect_cb(data): # detecting defects

    global defects

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    spot_img = cv_image.copy()

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, (20, 80, 120), (40, 255, 255))
    #cv2.imshow("BW image", black)

    center = black[(HEIGHT // 2) - 30:(HEIGHT // 2) + 30, :]
    
    #top_pub.publish(bridge.cv2_to_imgmsg(top, 'mono8'))
    #cv2.imshow("center", center)

    yellowzero = black[60:180, :].copy()
    n1 = cv2.countNonZero(yellowzero[0:40, :])
    n2 = cv2.countNonZero(yellowzero[40:80, :])
    n3 = cv2.countNonZero(yellowzero[80:120, :])
    x = [n1,n2,n3] 

    telem = get_telemetry(frame_id="aruco_map")

    coords = [0, 0]

    spot = cv2.inRange(hsv, (0, 68, 66), (45, 191, 255))

    cnts, _ = cv2.findContours(spot, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts.sort(key=cv2.minAreaRect)

    cnt = None

    if len(cnts) > 0:
        cnt = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(cnt) > 100:
            cv2.drawContours(spot_img, [cnt], 0, (255, 0, 0), 2)

    if x[1] <= (x[0]+x[2])/3:
        #print("detected")
        set_velocity(vx=0, vy=0, vz=0, yaw=float('nan'), yaw_rate=0, frame_id='body')

        cnt_area = cv2.contourArea(cnt)

        S = k * cnt_area

        coords = [round(telem.x, 2), round(telem.y, 2)]
        
        defects.append([coords, S])
        print([coords, S])

        navigate_wait(x=0.3, y=0, z=0, speed=0.5, frame_id="body", yaw=float('nan'))

    #print(f"{x_min}, {y_min}, {w_min}, {h_min}, {round(angle, 2)}, {error}")

    # if not is_top:
    #     print("Staying")

    #     state = False

    print("debug")
    debug.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    spot_pub.publish(bridge.cv2_to_imgmsg(spot_img, 'bgr8'))
    print("end")
    #cv2.imshow("Original image", cv_image)

    #cv2.waitKey(1)

def rotate_to_line(): # navigating to line on start pos

    rect_full = None    
    while rect_full == None:
        data = rospy.wait_for_message('main_camera/image_raw_throttled', Image)

        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        black = cv2.inRange(hsv, (20, 80, 120), (40, 255, 255))
        is_full, rect_full = get_rect_full(black, cv_image)

    print(rect_full)

    box = cv2.boxPoints(tuple(rect_full))

    print(box)

    ang = -math.pi * rect_full[2] / 180

    print(ang)    

    if abs(abs(ang) - 1.57) < 0.3 or (abs(ang) < 0.1 and abs(box[0][1] - box[2][1]) < abs(box[0][0] - box[2][0])):
        if rect_full[0][0] > WIDTH // 2:
            ang = 1.57
        else:
            ang = -1.57

    if rect_full[0][1] > HEIGHT // 2:
        ang += 3.14

    telem = get_telemetry(frame_id="aruco_map")
    ang0 = telem.yaw

    print(f"Angle: {rect_full[2]}, {ang}, {ang0}")

    #cv2.imshow("test", cv_image)

    navigate_wait(x=telem.x, y=telem.y, z=FLY_HEIGHT, frame_id="aruco_map", yaw=ang+ang0)
    rospy.sleep(5)


print("Taking off")
navigate(x=0, y=0, z=1, frame_id="body", speed=0.7, auto_arm=True) # UPPING
rospy.sleep(4)
# telem = get_telemetry(frame_id="aruco_map")
# navigate_wait(x=telem.x, y=telem.y, z=telem.z-0.3, frame_id="aruco_map", speed=1, auto_arm=False)
# rospy.sleep(3)

start_telem = get_telemetry(frame_id="aruco_map") # getting start telemetry
S_X = start_telem.x 
S_Y = start_telem.x

print(f"Start/land: {S_X}, {S_Y}")

print("Finding QR")

res = get_qr()
start_line = list(map(float, str(res)[2:-1].split())) # getting qr
print("Found QR!")
print(f"Start line is {start_line[0]} {start_line[1]}")

#navigate_wait(x=S_X, y=S_Y, z=1.5, frame_id="aruco_map")

print("Navigating to start line")
navigate_wait(x=start_line[0], y=start_line[1], z=FLY_HEIGHT, speed=0.3, frame_id="aruco_map") # go to line
rospy.sleep(2)
print("Got to start line")

print("Rotating to line")
rotate_to_line()
print("Rotated")

rospy.sleep(4)

print("Starting line")

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, img_cb)
defect_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, defect_cb)

state = True

while state:

    rospy.sleep(0.1)

image_sub.unregister()
defect_sub.unregister()


# navigate_wait(x=S_X, y=S_Y, z=2, frame_id="aruco_map")
# rospy.sleep(2)

home([S_X, S_Y])

print(defects)
# home((S_X, S_Y))

print(f"Navigation area x={S_X}, y={S_Y}")