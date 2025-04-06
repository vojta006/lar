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

def find_ball_xcenter(hsv_img):
    ball = find_ball(hsv_img)
    
    if (ball is not None):
        print("Length of ball", len(ball))
        b_coords = find_ball_center(ball)
        bx = b_coords[0]
        by = b_coords[1]
    else: 
        print("bx = None")
        bx = None
    return bx,ball

def finding_leftmost_point(cnt):
    x_coords = [point[0][0] for point in cnt]  # Extrahujeme X souřadnice
    min_x = min(x_coords)  # Najdeme nejmenší X souřadnici
    return min_x

def finding_righttmost_point(cnt):
    x_coords = [point[0][0] for point in cnt]  # Extrahujeme X souřadnice
    max_x = max(x_coords)  # Najdeme nejmenší X souřadnici
    return max_x

def find_width_of_ball(cnt):
    x_coords = [point[0][0] for point in cnt]  # Extrahujeme X souřadnice
    min_x = min(x_coords)  # Najdeme nejmenší X souřadnici
    max_x = max(x_coords)  # Najdeme nejmenší X souřadnici
    width = max_x-min_x
    return width

def goalnet_center_by_one_pillar(p1x,bx,cnt, net_width):
    if(p1x>bx):
        min_x = finding_leftmost_point(cnt)
        half_of_pillar = p1x - min_x
        middle_goalnet = p1x-net_width/2.5*half_of_pillar#11 in case, that pillar width is 7cm and goalnet width is 70 cm
        return middle_goalnet
    else:
        max_x = finding_righttmost_point(cnt)
        half_of_pillar = max_x-p1x
        middle_goalnet = p1x+net_width/2.5*half_of_pillar
        return middle_goalnet

def find_goalnet_center(hsv_img,bx):
    soccer_net_array, x_pillar, y_pillar = find_soccer_net_array(hsv_img)
    if(x_pillar[0] != None and x_pillar[1] != None):
        xp1 = x_pillar[0]
        xp2 = x_pillar[1]
        mpx = (xp1+xp2)/2
    elif (x_pillar[0] != None and x_pillar[1] == None):
        mpx = goalnet_center_by_one_pillar(x_pillar[0],bx,soccer_net_array[0])
    else:
        mpx = None
    return(mpx)



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

    return center_x, center_y

#TODO function that takes two points x,y and counts their distance


def find_soccer_net_array(hsv_image):
    #This fuction sort pillars by their mass of conotur area
    lower = np.array([75, 80, 50])
    upper = np.array([130, 255, 255])
    bin_mask = cv2.inRange(hsv_image, lower, upper)

    contours, hierarchy = cv2.findContours(bin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    soccer_net_array = []
    possible_pillar = None
    x_pillar = []
    y_pillar = []

    # Filtrovat a seřadit kontury podle velikosti (od největší po nejmenší)
    sorted_contours = sorted(
        [cnt for cnt in contours if 500 <= cv2.contourArea(cnt) <= 14000 and classify_pillar(cnt)],
        key=cv2.contourArea,
        reverse=True
    )

    for cnt in sorted_contours:
        soccer_net_array.append(cnt)
        x_coord_of_pillar, y_coord_of_pillar = finding_midlle_of_contours(cnt)
        x_pillar.append(x_coord_of_pillar)
        y_pillar.append(y_coord_of_pillar)

    # Zajištění minimálního počtu prvků v seznamu
    if len(x_pillar) < 2:
        x_pillar.append(None)
        x_pillar.append(None)

    if len(soccer_net_array) < 2:
        soccer_net_array.append(None)
        soccer_net_array.append(None)
    
    if(len(y_pillar) < 2):
        y_pillar.append(None)
        y_pillar.append(None)


    if None not in x_pillar[:2] and x_pillar[0] > x_pillar[1]:
        # Prohodíme pořadí, pokud první kontura je víc vpravo než druhá
        soccer_net_array[0], soccer_net_array[1] = soccer_net_array[1], soccer_net_array[0]
        x_pillar[0], x_pillar[1] = x_pillar[1], x_pillar[0]
        y_pillar[0], y_pillar[1] = y_pillar[1], y_pillar[0]

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
    upper = np.array([30, 255, 255])
    bin_mask = cv2.inRange(hsv_image, lower, upper)
    
    contours, hierarchy = cv2.findContours(bin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    ball = None
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 1000:
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


def two_pts_real_dst(rgb_coords1, rgb_coords2, point_cloud, threshold=0.20, window_size=4):
    h, w, _ = point_cloud.shape

    #Out of bounds
    if( not (0 <= rgb_coords1[0] < w and 0 <= rgb_coords1[1] < h)
        or not (0 <= rgb_coords2[0] < w and 0 <= rgb_coords2[1] < h)
    ):
        return None
    
    res = []

    for x, y in [rgb_coords1, rgb_coords2]:
        #Find limits of a given window (window ie.: Area in which we look for pixels)
        half_size = window_size // 2
        x_min, x_max = max(0, x - half_size), min(w, x + half_size + 1)
        y_min, y_max = max(0, y - half_size), min(h, y + half_size + 1)
        
        #Compute neighborhood
        neighborhood = point_cloud[y_min:y_max, x_min:x_max, :]
        x_neighborhood = neighborhood[...,0]
        x_valid_values = x_neighborhood[~np.isnan(x_neighborhood)]
        x_valid_values = x_valid_values[x_valid_values > 0]

        z_neighborhood = neighborhood[...,2]
        z_valid_values = z_neighborhood[~np.isnan(z_neighborhood)]
        z_valid_values = z_valid_values[z_valid_values > 0]
        #x_neighborhood = neighborhood[ neighborhood[~np.isnan(neighborhood[...,0])] > 0]
        #z_neighborhood = neighborhood[ neighborhood[~np.isnan(neighborhood[...,2])] > 0]
        

        if x_neighborhood.size == 0 or z_neighborhood.size == 0:
            return None

        x_median = np.median(x_neighborhood)
        z_median = np.median(z_neighborhood)

        #Compute threshold
        x_sim_values = x_neighborhood[np.abs(x_neighborhood - x_median) < threshold]
        z_sim_values = z_neighborhood[np.abs(z_neighborhood - z_median) < threshold]

        res.append([np.mean(x_sim_values), np.mean(z_sim_values)])

    print(*res)
    return np.sqrt((res[0][0] - res[1][0])**2 + (res[0][1] - res[1][1])**2)


def hood_depth(point_cloud, x, y, threshold=0.20, window_size=4):
    "Returns the depth of a neighborhood of a given pixel in range 640(x) 480(y)"
    #Returns point cloud shape
    h, w, _ = point_cloud.shape

    #Out of bounds
    if not (0 <= x < w and 0 <= y < h):
        print(f"Out of bounds")
        return None

    #Find limits of a given window (window ie.: Area in which we look for pixels)
    half_size = window_size // 2
    x_min, x_max = max(0, x - half_size), min(w, x + half_size + 1)
    y_min, y_max = max(0, y - half_size), min(h, y + half_size + 1)
    

    #Compute neighborhood
    neighborhood = point_cloud[y_min:y_max, x_min:x_max, :]
    neighborhood = np.sqrt(neighborhood[..., 0]**2 + neighborhood[..., 2]**2)
    #Filter out all NaN pixels
    valid_values = neighborhood[~np.isnan(neighborhood)]
    #Filter out pixels with depth 0
    valid_values = valid_values[valid_values > 0]

    #No valid values
    if valid_values.size == 0:
        print(f"Window too small or in an NaN lake")
        return None

    #Find median among valid values
    median_depth = np.median(valid_values)
    #Compute threshold
    similar_values = valid_values[np.abs(valid_values - median_depth) < threshold]
    
    if similar_values.size > 0:
        return np.mean(similar_values)
    else:
        print("Threshold too narrow")
        return None
