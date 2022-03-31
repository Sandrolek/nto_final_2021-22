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
top_pub = rospy.Publisher('top', Image, queue_size=1)
black_pub = rospy.Publisher('black', Image, queue_size=1)

bridge = CvBridge()

HEIGHT = 240
WIDTH = 320

FLY_HEIGHT = 1.2

# REAL
# mask = ((34, 57, 61), (47, 255, 255))
# mask = ((34, 57, 61), (47, 255, 255))

# GAZEBO
mask = ((20, 120, 0), (40, 140, 255))

k = 0.25 / 2300

SPEED_X = 0.1

def navigate_wait(x=0, y=0, z=FLY_HEIGHT, yaw=1.57, speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def home_simple(xy):    
    s = list(map(float, xy)) 
    navigate_wait(x = s[0], y= s[1], z = 2)
    rospy.sleep(3)
    print("got to land point")
    land()

def home(xy):    
    s = list(map(float, xy)) 
    navigate_wait(x = s[0], y= s[1], z = 2)
    rospy.sleep(3)
    while rospy.wait_for_message('rangefinder/range', Range).range > 0.3:
        navigate_wait(frame_id='body', z = -0.2, speed = 1)
        print('land')         
    if rospy.wait_for_message('rangefinder/range', Range).range <= 0.3:
        print('disarm')
        arming(False)

def convert_angle(angle=0, w_min=320, h_min=240):

    if angle < -45:
        angle = 90 + angle
    if w_min < h_min and angle > 0:
        angle = (90 - angle) * -1
    if w_min > h_min and angle < 0:
        angle = 90 + angle

    return angle

def get_qr(): 

    qr = None
    while qr == None:
        data = rospy.wait_for_message('main_camera/image_raw_throttled', Image)
    
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')

        barcodes = decode(cv_image) 
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect  
            qr = barcode.data

    return qr

def get_rect_top(img, orig):
    
    cnts, _ = cv2.findContours(img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts.sort(key=cv2.minAreaRect)

    (x_min, y_min), (w_min, h_min), angle = (0, 0), (0, 0), 0

    if len(cnts) > 0:
        cnt = max(cnts, key=cv2.contourArea)

        if cv2.contourArea(cnt) > 100:
            #state = 
            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect

            #print(angle)

            #y_min += TOP_X
            #x_min += TOP_X - 40

            box = cv2.boxPoints(((x_min, y_min), (w_min, h_min), angle)) # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(orig, [box], 0, (255, 0, 0), 2)

            angle = convert_angle(angle=angle, w_min=w_min, h_min=h_min)

            return True, [(x_min, y_min), (w_min, h_min), angle]
        
    
    return False, []

def get_rect_full(img, orig):
    
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

def get_square(img):
    S = 0

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, mask[0], mask[1])
    cv2.imshow("BW image", black)

    cnts, _ = cv2.findContours(black.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts.sort(key=cv2.minAreaRect)

    if len(cnts) > 0:
        cnt = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(cnt) > 100:
            cv2.drawContours(img, [cnt], 0, (0, 255, 0), 2)

            cnt_area = cv2.contourArea(cnt)

            S = round(k * cnt_area, 2)

            print(cnt_area, S)

    return S

def img_cb(data):
    global state

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    orig = cv_image.copy()

    # undistort camera

    img = cv2.undistort(
    cv_image,np.array([[166.23942373073172,0,162.19011246829268],
    [0,166.5880923974026,109.82227735714285], [0,0,1]]), np.array([
    2.15356885e-01, -1.17472846e-01, -3.06197672e-04,-1.09444025e-04,
    -4.53657258e-03, 5.73090623e-01,-1.27574577e-01, -2.86125589e-02,
    0.00000000e+00,0.00000000e+00, 0.00000000e+00,
    0.00000000e+00,0.00000000e+00, 0.00000000e+00]),
    np.array([[166.23942373073172,0,162.19011246829268],
    [0,166.5880923974026,109.82227735714285], [0,0,1]]))

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, (20, 80, 120), (40, 255, 255))
    #cv2.imshow("BW image", black)

    # detecting defects

    '''

    ############# DEFECT #############
    cnts, hierarchy = cv2.findContours(black.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts.sort(key=cv2.minAreaRect)
    cnts = list(filter(lambda x: True if cv2.contourArea(x) > 30 else False, cnts))

    try:
        hierarchy = hierarchy[0] # get the actual inner list of hierarchy descriptions
    except Exception as e:
        return

    if len(cnts) < 2:
        return

    # For each contour, find the bounding rectangle and draw it
    for component in zip(cnts, hierarchy):
        currentContour = component[0]
        currentHierarchy = component[1]
        x, y, w, h = cv2.boundingRect(currentContour)
        if currentHierarchy[2] < 0:
            # these are the innermost child components
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),3)

            y0 = (y + h) / 2

            if y0 < (HEIGHT // 2 - 40) or y0 > (HEIGHT // 2 + 40):
                return

            print("Detected defect")

            set_velocity(vx=0, vy=0, vz=0, yaw=float('nan'), yaw_rate=0, frame_id='body')
            rospy.sleep(1)

            telem = get_telemetry(frame_id="aruco_map")

            navigate_wait(x=telem.x, y=telem.y, z=FLY_HEIGHT + 0.3, frame_id="aruco_map", speed=0.5)

            square = get_square(bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw_throttled', Image), 'bgr8'))

            print(f"Defect: x={telem.x}, y={telem.y}, S={square}")

            navigate_wait(x=0.3, y=0, z=0, speed=0.5, frame_id="body")
            rospy.sleep(2)
            
            print("went out")

        elif currentHierarchy[3] < 0:
            # these are the outermost parent components
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),3)

            

    ############# DEFECT #############

    '''

    top = black[:(HEIGHT // 2), :]
    top_right = black[:(HEIGHT // 2), :WIDTH // 2]
    top_left = black[:(HEIGHT // 2), WIDTH // 2:]
    
    is_full, rect_full = get_rect_full(black, cv_image)
    
    is_top, rect_top = get_rect_top(top, cv_image)

    # top_pub.publish(bridge.cv2_to_imgmsg(top, 'mono8'))

    telem = get_telemetry(frame_id="aruco_map")

    # following line

    if is_full:
        print("Lining")
        (x_min, y_min), (w_min, h_min), angle = rect_full

        center = cv_image.shape[1] / 2
        error = x_min - center

        #print(f"{x_min}, {y_min}, {w_min}, {h_min}, {round(angle, 2)}, {error}")

        cv2.circle(cv_image, (int(x_min), int(y_min)), 5, (0, 0, 255), 3)

        #print(round(angle, 2), error)
        set_velocity(vx=SPEED_X, vy=error*(-0.005), vz=-(telem.z - FLY_HEIGHT)*0.5, yaw=float('nan'), yaw_rate=angle*(-0.008), frame_id='body')

    # if no line, go home
    if not is_top:
        print("Staying")

        state = False

    #print("debug")
    debug.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    black_pub.publish(bridge.cv2_to_imgmsg(black, 'mono8'))
    #print("end")
    #cv2.imshow("Original image", cv_image)

    #cv2.waitKey(1)
'''
def defect_cb(data):

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    img = cv2.undistort(
    cv_image,np.array([[166.23942373073172,0,162.19011246829268],
    [0,166.5880923974026,109.82227735714285], [0,0,1]]), np.array([
    2.15356885e-01, -1.17472846e-01, -3.06197672e-04,-1.09444025e-04,
    -4.53657258e-03, 5.73090623e-01,-1.27574577e-01, -2.86125589e-02,
    0.00000000e+00,0.00000000e+00, 0.00000000e+00,
    0.00000000e+00,0.00000000e+00, 0.00000000e+00]),
    np.array([[166.23942373073172,0,162.19011246829268],
    [0,166.5880923974026,109.82227735714285], [0,0,1]]))

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, (20, 80, 120), (40, 255, 255))
    #cv2.imshow("BW image", black)

    cnts, hierarchy = cv2.findContours(black.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts.sort(key=cv2.minAreaRect)

    try:
        hierarchy = hierarchy[0] # get the actual inner list of hierarchy descriptions
    except Exception as e:
        return

    if len(cnts) < 2:
        return

    # For each contour, find the bounding rectangle and draw it
    for component in zip(cnts, hierarchy):
        currentContour = component[0]
        currentHierarchy = component[1]
        x, y, w, h = cv2.boundingRect(currentContour)
        if currentHierarchy[2] < 0:
            # these are the innermost child components
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),3)

            print("detected defect")
        elif currentHierarchy[3] < 0:
            # these are the outermost parent components
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),3)

    # for cnt in cnts:
    #     cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)

    #     #state = 
    #     #print("FULL", random.randint(0, 100))

    #     rect = cv2.minAreaRect(cnt)
    #     (x_min, y_min), (w_min, h_min), angle = rect

    #     # x_min += FULL_X_B
    #     # y_min += FULL_Y_B

    #     box = cv2.boxPoints(((x_min, y_min), (w_min, h_min), angle)) # cv2.boxPoints(rect) for OpenCV 3.x
    #     box = np.int0(box)
        

    #     angle = convert_angle(angle=angle, w_min=w_min, h_min=h_min)

    #print("debug")
    debug.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    #print("end")
    #cv2.imshow("Original image", cv_image)

    #cv2.waitKey(1)
'''


def rotate_to_line(): # rotated to line in start_line_point

    telem = get_telemetry(frame_id="aruco_map")

    #navigate_wait(x=telem.x, y=telem.y, z=FLY_HEIGHT + 0.3, frame_id="aruco_map", speed=0.5)

    navigate_wait(x=0, y=0, z=0.3, frame_id="body", speed=0.5)

    #print("Upped")

    #rospy.sleep(2)

    rect_full = None
    while rect_full == None:
        data = rospy.wait_for_message('main_camera/image_raw_throttled', Image)

        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        black = cv2.inRange(hsv, (20, 80, 120), (40, 255, 255))
        is_full, rect_full = get_rect_full(black, cv_image)

    debug.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    #rospy.sleep(3)

    print(rect_full)

    box = cv2.boxPoints(tuple(rect_full))

    print(box)

    ang = -math.pi * (rect_full[2] + 0.03) / 180

    print(f"orig: {ang}")    

    if abs(abs(ang) - 1.57) < 0.3 or (abs(ang) < 0.1 and abs(box[0][1] - box[2][1]) < abs(box[0][0] - box[2][0])): # deciding, where line is
        if rect_full[0][0] > WIDTH // 2:
            print("> x")
            ang = 1.57
        else:
            print("< x")
            ang = -1.57

    if rect_full[0][1] > HEIGHT // 2:
        print("> y")
        ang += 3.14

    telem = get_telemetry(frame_id="aruco_map")
    ang0 = telem.yaw

    print(f"Angle: {rect_full[2]}, {ang}, {ang0}")

    #cv2.imshow("test", cv_image)

    navigate_wait(x=0, y=0, z=0, frame_id="body", yaw=ang)
    rospy.sleep(3)


print("Taking off")
navigate(x=0, y=0, z=0.8, frame_id="body", speed=0.7, auto_arm=True)
rospy.sleep(4)
# telem = get_telemetry(frame_id="aruco_map")
# navigate_wait(x=telem.x, y=telem.y, z=telem.z-0.3, frame_id="aruco_map", speed=1, auto_arm=False)
# rospy.sleep(3)


# remember out start point
start_telem = get_telemetry(frame_id="aruco_map")
S_X = start_telem.x
S_Y = start_telem.y

print(f"Start/land: {S_X}, {S_Y}")

print("Finding QR")

# finding qr
res = get_qr()
start_line = list(map(float, str(res)[2:-1].split()))
print("Found QR!")
print(f"Start line is {start_line[0]} {start_line[1]}")

#navigate_wait(x=S_X, y=S_Y, z=1.5, frame_id="aruco_map")

print("Navigating to start line")
navigate_wait(x=start_line[0], y=start_line[1], z=FLY_HEIGHT, speed=0.3, frame_id="aruco_map")
rospy.sleep(2)
print("Got to start line")

# moving to line
print("Rotating to line")
rotate_to_line()
print("Rotated")

rospy.sleep(1)

print("Starting line")

# starting moving on line

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, img_cb)

# defect_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, defect_cb)

# rospy.spin()

state = True

while state:

    rospy.sleep(0.1)

image_sub.unregister()
# defect_sub.unregister()

print("Navigating to start")
# navigate_wait(x=S_X, y=S_Y, z=2, frame_id="aruco_map")
# rospy.sleep(2)

home_simple([S_X, S_Y])
# home((S_X, S_Y))
