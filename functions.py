
import cv2
import numpy as np


def classify_circle(possible_ball):
    hull = cv2.convexHull(possible_ball)
    rect = cv2.minAreaRect(hull)
    circ = cv2.minEnclosingCircle(hull)

    width, height = rect[1]
    radius = circ[1]

    S_hull = cv2.contourArea(hull)
    S_rect = width*height
    S_circ = np.pi * radius**2
    
    circle_hull = (S_hull/S_circ) + 0.1
    rectangle_hull = S_hull/S_rect

    if(circle_hull > rectangle_hull):
        return True
    else:
        return False

def obj_cooords(hull):
    moments = cv2.moments(hull)

    # Těžiště
    if moments["m00"] != 0:  # Ověření, že plocha není nulová
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        centroid = (cx, cy)
    else:
        centroid = None

    return centroid

def classify_pillar(possible_pillar):
    hull = cv2.convexHull(possible_pillar)
    x, y, width, height = cv2.boundingRect(hull)

    ratio = width/height

    if (1/10 < ratio < 1/2):
        return True
    else:
        return False

def finding_midlle_of_contours(cnt):
    x_coords = [point[0][0] for point in cnt]
    y_coords = [point[0][1] for point in cnt]

    center_x = int(round(sum(x_coords) / len(x_coords)))
    center_y = int(round(sum(y_coords) / len(y_coords)))

    #print(f"Střed kontury na ose X: {center_x}")
    return center_x, center_y

 
def find_soccer_net_array(hsv_image):
    lower = np.array([75, 80, 50])
    upper = np.array([130, 255, 255])
    bin_mask = cv2.inRange(hsv_image, lower, upper)

    #plt.imshow(bin_mask)
    #plt.show()

    contours, hierarchy = cv2.findContours(bin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    soccer_net_array = []
    possible_pillar = None
    x_pillar = []
    y_pillar = []

    for cnt in contours:

        actual_area = cv2.contourArea(cnt)
        if actual_area < 500 or actual_area > 14000:
            continue

        possible_pillar = cnt

        if not classify_pillar(possible_pillar):
            continue
        
        soccer_net_array.append(cnt)
        x_coord_of_pillar, y_coord_of_pillar = finding_midlle_of_contours(cnt)
        x_pillar.append(x_coord_of_pillar)
        y_pillar.append(y_coord_of_pillar)

        continue

    if(len(x_pillar) < 2):
        x_pillar.append(None)
        x_pillar.append(None)

    if(len(y_pillar) < 2):
        y_pillar.append(None)
        y_pillar.append(None)


    #print(f"Střed kontury1: {x_pillar[0]},Střed kontury2: {x_pillar[1]} ")

    if(len(soccer_net_array) < 2):
        soccer_net_array.append(None)
        soccer_net_array.append(None)

    return soccer_net_array, x_pillar, y_pillar


def count_angular_speed(rad, curr_speed):
    speed = 0 
    
    if rad < 1:
        speed = max(curr_speed*0.97, 0.3)
    else:
        speed = min(curr_speed*1.1, 3)

    return speed


def find_ball(hsv_image):

    lower = np.array([18, 130, 45])
    upper = np.array([130, 255, 255])
    bin_mask = cv2.inRange(hsv_image, lower, upper)
    
    contours, hierarchy = cv2.findContours(bin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    ball = None
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 3000:
            continue
        possible_ball = cnt

        if (classify_circle(possible_ball) == True):
           ball = cnt
        else:
           continue

    return ball


def find_ball_center(ball):

    if ball is not None:
        return obj_cooords(cv2.convexHull(ball))

    return [None, None]


def depth_of_pixel(point_cloud, x, y):
    "Returns the depth of a given pixel in range 640(x) 480(y)"
    #Returns point cloud shape
    h, w, _ = point_cloud.shape

    #Out of bounds
    if not (0 <= x < w and 0 <= y < h):
        return None

    point = point_cloud[y, x]  # OpenCV/NumPy indexing (column,row)

    #Given pixel is without value
    if np.isnan(point).any():
        return None

    #Get depth
    depth = point[2] #point = [h,w,d]

    return depth

