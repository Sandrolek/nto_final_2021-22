# импортируем нужные либы

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

# создаем нужные сервисы прокси
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

# создаем публикаторов отладочных и требуемых по заданию
debug = rospy.Publisher('debug', Image, queue_size=1)
top_pub = rospy.Publisher('top', Image, queue_size=1)
black_pub = rospy.Publisher('black', Image, queue_size=1)
oil_pub = rospy.Publisher('oil_detect', Image, queue_size=1)

# объект для конвертации типов изображения
bridge = CvBridge()

HEIGHT = 240
WIDTH = 320

FLY_HEIGHT = 1.2

# gazebo:
#mask = ((20, 120, 0), (40, 140, 255))
#mask_spot = ((0, 76, 73), (13, 250, 245))
#mask_defect = ((0, 0, 35), (255, 255, 134))

# набор диапазонов(масок) для линии, пятная, дефекта
mask = ((34, 57, 61), (47, 255, 255))
mask_spot = ((0, 10, 46), (193, 159, 255))
mask_defect = ((37, 132, 26), (62, 241, 71))

# кэф для определения площади разлива
k = 0.25 / 2300

# скорость
SPEED_X = 0.08

# функция полета в точку озера и опускание к нему
def go_to_lake(point):
    navigate_wait(x=point[0], y=point[1], z=FLY_HEIGHT, frame_id="aruco_map")
    navigate_wait(x=point[0], y=point[1], z=0.6, frame_id="aruco_map")
    rospy.sleep(5)
    print("Successful water withdrawal")
    navigate_wait(x=point[0], y=point[1], z=FLY_HEIGHT, frame_id="aruco_map")

# функция полета в точку и ожиданиия прилета в нее
def navigate_wait(x=0, y=0, z=FLY_HEIGHT, yaw=1.57, speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

# летим на запомненную координату тумбы
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

# конвертим углы для лучшего восприятия логического
def convert_angle(angle=0, w_min=320, h_min=240):

    if angle < -45:
        angle = 90 + angle
    if w_min < h_min and angle > 0:
        angle = (90 - angle) * -1
    if w_min > h_min and angle < 0:
        angle = 90 + angle

    return angle

# получаем данные с qr кода в байтах, с помощью либы pyzbar
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

# получаем minAreaRect для верхней части изображения, также детектим наличие конца линии
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

# то же самое, толкьо для всего изображения, по этому прямоугльнику летим по линии
def get_rect_full(img, orig):
    
    cnts, _ = cv2.findContours(img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # контуры на маске
    cnts.sort(key=cv2.minAreaRect)

    if len(cnts) > 0:
        #cnt = cnts[0]
        cnt = max(cnts, key=cv2.contourArea) # отсеиваем маленькие контуры
        if cv2.contourArea(cnt) > 50:
            #state = 
            #print("FULL", random.randint(0, 100))

            rect = cv2.minAreaRect(cnt) # описанный вокруг контура прямоугольник минимально возможной площади
            (x_min, y_min), (w_min, h_min), angle = rect

            # x_min += FULL_X_B
            # y_min += FULL_Y_B

            box = cv2.boxPoints(((x_min, y_min), (w_min, h_min), angle)) # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(orig, [box], 0, (0, 255, 0), 2)

            angle = convert_angle(angle=angle, w_min=w_min, h_min=h_min)

            return True, [(x_min, y_min), (w_min, h_min), angle] 
            
    return False, []

# получаем площадь пятна
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

# главный колбэк, тут летим по линии
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

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # переводим в hsv
    black = cv2.inRange(hsv, (20, 80, 120), (40, 255, 255))
    #cv2.imshow("BW image", black)

    top = black[:(HEIGHT // 2), :] # делаем срез изображения
    top_right = black[:(HEIGHT // 2), :WIDTH // 2]
    top_left = black[:(HEIGHT // 2), WIDTH // 2:]
    
    is_full, rect_full = get_rect_full(black, cv_image)
    
    is_top, rect_top = get_rect_top(top, cv_image)

    # top_pub.publish(bridge.cv2_to_imgmsg(top, 'mono8'))

    telem = get_telemetry(frame_id="aruco_map") # плолучаем телеметрию коптера

    # following line

    if is_full: # если есть линия на картинке
        print("Lining")
        (x_min, y_min), (w_min, h_min), angle = rect_full

        center = cv_image.shape[1] / 2
        error = x_min - center # считаем отклонение от центра линии

        #print(f"{x_min}, {y_min}, {w_min}, {h_min}, {round(angle, 2)}, {error}")

        cv2.circle(cv_image, (int(x_min), int(y_min)), 5, (0, 0, 255), 3)

        #print(round(angle, 2), error)
        set_velocity(vx=SPEED_X, vy=error*(-0.005), vz=-(telem.z - FLY_HEIGHT)*0.5, yaw=float('nan'), yaw_rate=angle*(-0.008), frame_id='body') # П-регулятор по vy и также по высоте, поддерживаем заданную

    # if no line, go home
    if not is_top:
        print("Staying")

        state = False

    debug.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    black_pub.publish(bridge.cv2_to_imgmsg(black, 'mono8'))

# определяем разливы
def spot_cb(data):

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    img = cv_image

    hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, mask_spot[0], mask_spot[1])
    
    cnts, _ = cv2.findContours(black, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = list(filter(lambda x: True if cv2.contourArea(x) > 50 else False, cnts)) # фильтруем контуры по площади

    #print(len(cnts))

    cv2.drawContours(img, cnts, -1, (255, 0, 0), 3)

    for cnt in cnts:
        #cv2.drawContours(img, cnt, -1, (255, 0, 0), 3)
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect) # cv2.boxPoints(rect) for OpenCV 3.x
        box = np.int0(box)
        #cv2.drawContours(img, [box], 0, (255, 0, 0), 2)

        S = round(cv2.contourArea(cnt) * k, 2) # кэф k посчитан экспериментально 

        print(f"oil area: {S}")

    oil_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

# наводимся на линиию в самом начале движения
def rotate_to_line(): # rotated to line in start_line_point

    print("Upped")
    rospy.sleep(1)

    rect_full = None
    while rect_full == None:
        data = rospy.wait_for_message('main_camera/image_raw_throttled', Image) #получаем изображение

        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        black = cv2.inRange(hsv, (20, 80, 120), (40, 255, 255))
        is_full, rect_full = get_rect_full(black, cv_image)

    debug.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    print("Published")
    rospy.sleep(2)
    box = cv2.boxPoints(tuple(rect_full)) # получаем корды всех 4 точек прямоугольника
    ang = -math.pi * (rect_full[2] + 0.03) / 180

    print(rect_full)
    print(box)
    print(f"orig: {ang}")    

    if abs(abs(ang) - 1.57) < 0.1 or (abs(ang) < 0.05 and abs(box[0][1] - box[2][1]) < abs(box[0][0] - box[2][0])) and ang > 0: # если линия горизонтальная, то задаем фиксированный угол поворота
        if rect_full[0][0] > WIDTH // 2:
            print("> x")
            ang = 1.57
        else:
            print("< x")
            ang = -1.57

    if rect_full[0][1] > HEIGHT // 2: # если линия вниз  идет, добавляем pi 
        print("> y")
        ang += 3.14

    print(f"Angle: {rect_full[2]}, {ang}")

    navigate_wait(x=0, y=0, z=0, frame_id="body", yaw=ang)
    rospy.sleep(3)

# взлет
print("Taking off")
navigate(x=0, y=0, z=0.8, frame_id="body", speed=0.7, auto_arm=True)
rospy.sleep(6)

# remember out start point
start_telem = get_telemetry(frame_id="aruco_map")
S_X = start_telem.x
S_Y = start_telem.y

print(f"Start/land: {S_X}, {S_Y}")

print("Finding QR")

# finding qr
res = get_qr()
print(res)
res = str(res)[2:-1].replace('\\n', ' ').replace('\\r', ' ')
print(res)
data = list(map(float, res.split()))
print(data)
start_line = data[:2]
lake_point = data[2:]
print("Found QR!")

print(f"Start line is {start_line[0]} {start_line[1]}")
print(f"Lake: {lake_point[0]}, {lake_point[1]}")

#navigate_wait(x=S_X, y=S_Y, z=1.5, frame_id="aruco_map")

# направляемя к озеру
print("go lake")
go_to_lake(lake_point)

# летим к началу линии
print("Navigating to start line")
navigate_wait(x=start_line[0], y=start_line[1], z=FLY_HEIGHT, speed=0.3, frame_id="aruco_map")
rospy.sleep(2)
print("Got to start line")

# turning to line
print("Rotating to line")
rotate_to_line()
print("Rotated")

rospy.sleep(1)

print("Starting line")

# делаем подписчиков для определения разливов и общего движения
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, img_cb)
spot_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, spot_cb, queue_size=1)

# defect_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, defect_cb)

# rospy.spin()

state = True

while state: # пока летим по линии, пока она из кадра не исчезла

    rospy.sleep(0.1)

image_sub.unregister()
spot_sub.unregister()
# defect_sub.unregister()

# возвращаемся домой
print("Navigating to start")

home_simple([S_X, S_Y])