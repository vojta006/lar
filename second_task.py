from __future__ import print_function
import cv2
import numpy as np
import threading
from robolab_turtlebot import Turtlebot, Rate, get_time
from functions import find_ball, find_ball_center, count_angular_speed, find_soccer_net_array, depth_of_pixel
from math import sqrt, pi, cos, acos, asin, sin, atan
from time import sleep
from simple_pid import PID

# Initialize the Turtlebot instance
turtle = Turtlebot(rgb=True, depth=True, pc=True)

def incremental_go_straight(wanted_dist, max_speed):
    #Acceleration speed const
    alpha = 0.15
    #Minimal movement speed const
    min_speed = 0.1  
    #Boolean, true if turtle travelled more than dis_comp_const of distance
    tquarts = False
    #Constant for setting a point at which turtle start slowing down
    dis_comp_const = 0.6

    turtle.reset_odometry() 
    turtle.wait_for_odometry()
    rate = Rate(100)  # 100 Hz smyčka řízení

    peak_speed = max_speed
    #Iteration variable
    t = 0.1

    #PID init 
    #PID regulator for angle
    Kp = 1.0   
    Ki = 0.01
    Kd = 0.05
    pid_angle = PID(Kp, Ki, Kd, setpoint=0)
    pid_angle.output_limits = (-0.5, 0.5)

    while True:
        #Iteration value
        t += 0.01

        x, y, angle = turtle.get_odometry()
        travelled_dist = sqrt(x**2 + y**2)

        if abs(wanted_dist - travelled_dist) < 0.01:
            turtle.cmd_velocity(linear=0, angular=0)
            break

        #If traveled less than dis_comp_const of distance SPEED UP
        if travelled_dist < (dis_comp_const * wanted_dist):
            set_speed = max_speed * (1 - np.exp(-alpha * t))
            peak_speed = set_speed
        #If traveled more.. SLOW DOWN 
        else:
            #First time over, reinit t
            if not tquarts:
                tquarts = True
                t = 0.1 
            set_speed = peak_speed * np.exp(-2 * alpha * t)

        #Minimal movement speed is preset
        set_speed = max(set_speed, min_speed)

        #PID angle correction
        angle_correction = pid_angle(angle)  # Výpočet korekce na základě úhlu

        print(f"Speed: {set_speed:.3f}, Angle: {angle:.2f}, Correction: {angle_correction:.2f}")
        print("x:",x,"y:",y)
        turtle.cmd_velocity(linear=set_speed, angular=angle_correction)

        rate.sleep()   

def button_event(msg):
    global event
    event.set()

def take_hsv_picture():
    while True:
        turtle.wait_for_rgb_image()
        bgr_image = turtle.get_rgb_image()
        print(bgr_image.shape)
        break
    return cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

def count_rotation(coords):
    # 320 - half of the picture, 6 - Bulgarian constant
    return (coords[0] - 320)*pi / (6*180)

def rotate(rot_rad):

    turtle.reset_odometry()
    turtle.wait_for_odometry()

    rate = Rate(100)
    print(f"rotating for {rot_rad}")
    speed = 0.3
    direction = np.sign(rot_rad)
    print(direction)

    while True:
        x, y, alpha = turtle.get_odometry()
        speed = count_angular_speed(abs(alpha-rot_rad), speed)
        
        if abs(rot_rad) - abs(alpha) < 0.01:
            turtle.cmd_velocity(angular=0)
            break

        turtle.cmd_velocity(angular=speed*direction)
        rate.sleep()

def go_straight(meters):
    t = get_time()
    time = abs(meters/0.1)
    dr = np.sign(meters)
    rate = Rate(40)
    
    while get_time() - t < time:
        turtle.cmd_velocity(linear=0.1*dr)
        rate.sleep()

def go_to_pos(curr_pos, pos):
    
    rotate(-curr_pos[2])
    print('y dst:', curr_pos[1] - pos[1])
    incremental_go_straight(curr_pos[1] - pos[1], 1)
    flag = False
    if curr_pos[0] - pos[0] > 0:
        rotate(pi/2)
        flag = True
    else:
        rotate(-pi/2)
    incremental_go_straight(abs(curr_pos[0] - pos[0]), 1)
        
    if flag:
        rotate(-pi/2)
    else:
        rotate(pi/2)

    rotate(-pos[2])


def second_task():
    event.wait()  # wait for button push
    pos = count_robot_pos() #x, y, rot; or None - then call this function in loop with rotating

    if pos is None:
        print("Nevidíme sloupky")
        return

    print(*pos)
    return
    dest = [0, 2, 0]
    go_to_pos(pos, dest)
    
    #rel_ball_pos = scan_ball_pos(dest) #if it cannot see the ball, it returns None
    #if rel_ball_pos is None:
    #    print("Nevidíme míč.")
    #    return
    #
    #ball_pos = [rel_ball_pos[0], 2 + rel_ball_pos[0], 0]
    #print(*ball_pos)

    #go_to_pos(dest , pos_behind_the_ball(ball_pos, 0.5)) #TODO count new precise pos instead of dest

    
def pos_behind_the_ball(ball_pos, dst):
    alpha = atan(ball_pos[0]/ball_pos[1])
    radius = sqrt(ball_pos[0]**2 + ball_pos[1]**2)
    return [(radius+dst)*sin(alpha), (radius + dst)*cos(alpha), 0]

#stop on bounce
def follow_ball_line():
    pass

def pixel_to_angle(x_pixel):
    return -(x_pixel - 320)*pi/(320*4)

#TODO rozchodit funkci na detekci pozice míče
#TODO pohyb po hřišti moc nefunguje

def count_relative_ball_pos(b_coords, curr_pos):
    turtle.wait_for_point_cloud()
    point_cloud = turtle.get_point_cloud()
    ball_dst = depth_of_pixel(point_cloud, *b_coords) + 0.15 #+ diameter
    print("estimated ball dst:", ball_dst)
    alpha = pixel_to_angle(b_coords[0]) #TODO odladit převod
    print("alpha:", alpha)
    beta  = curr_pos[2]
    #rel_rob_x = ball_dst*sin(alpha) #relative x coordinate to robot coord system
    #rel_rob_y = ball_dst*cos(alpha)
    angle = beta - alpha
    return [ball_dst*cos(angle), ball_dst*sin(angle), 0]


#we assume that the rotation is known and noted in curr_pos
def scan_ball_pos(curr_pos): 
    hsv_img = take_hsv_picture()
    ball = find_ball(hsv_img)

    if ball is None:
        return None
    
    #we can compute the position of the ball
    b_coords = find_ball_center(ball)
    print("souřadnice míče v obrázku", *b_coords)

    return count_relative_ball_pos(b_coords, curr_pos)

def count_pos(dl, dr, x_coords):
    a = 0.75
    b = dl + 0.025
    c = dr + 0.025
    eq = lambda x, y: abs(x-y) < 0.1

    beta = acos((b**2 - c**2 - a**2)/(-2*c*a))
    alpha = acos((a**2 - b**2 - c**2)/(-2*b*c))
    print("alpha:", alpha)
    print("beta:", beta)

    y = c*sin(beta)
    xr = sqrt(c**2 - y**2)
    xl = sqrt(b**2 - y**2)
    print("y",y)
    print('xl', xl)
    print('xr', xr)
    x = 0

    if eq(xl+xr,0.7):
        x = - xr
    elif eq(xr - xl, 0.7):
        x = -xr
    elif eq(xl - xr, 0.7):
        x = xr
    else:
        print("Chyba, výpočet souřadnic je špatně.")
    
    print('x', x+0.35)
    return [x + 0.35, y, count_rotation(alpha, beta, x_coords[0], x_coords[1]) ]


def count_rotation(alpha, beta, xl, xr):
    rot = None
    if beta > pi/2:
        rot = beta - pi/2
        rot += (xr - 320)*alpha/(xr-xl)
    else:
        rot = pi/2 - beta
        rot += (xr - 320)*alpha/(xr-xl)

    return rot

#jsou sloupky dostatečně ve středu obrázku?
def centered(x1, x2):
    m1 = min(x1, 641-x1)
    m2 = min(x2, 641-x2)
    mx = min(m1, m2)
    print("Centered:", mx)
    return mx > 20 

#TODO wait for odometry sleep
def rotate_to_center_net():
    
    parts = 12
    for i in range(parts):
        hsv_image = take_hsv_picture()
        net, x_c, y_c = find_soccer_net_array(hsv_image)

        if net[0] is not None and net[1] is not None and centered(*x_c):
            return i*2*pi/parts #how much had we rotate 

        rotate(2*pi/parts)

    #We could not find pillars
    return None

#from one position, without moving in space
def count_robot_pos():
    successful = 0
    pictures_to_take = 6

    if rotate_to_center_net() is None:
        return None

    overall_pos = [0, 0, 0]

    for i in range(pictures_to_take):
        pos = get_robot_pos()
        if pos is not None:
            overall_pos[0] += pos[0]
            overall_pos[1] += pos[1]
            overall_pos[2] += pos[2]
            successful += 1
    print("Úspěchů:", successful) 
    if successful/pictures_to_take < 2/3:
        print("Malý podíl úspěšných fotek.")
        return None

    return [overall_pos[0]/pictures_to_take,
            overall_pos[1]/pictures_to_take,
            overall_pos[2]/pictures_to_take]

def get_robot_pos(): #or None if it cannot see the pillars

    hsv_image = take_hsv_picture()
    turtle.wait_for_point_cloud()
    point_cloud = turtle.get_point_cloud()
    
    net, x_c, y_c = find_soccer_net_array(hsv_image)
    
    if net[0] is None or net[1] is None:
        return None
    
    dl = depth_of_pixel(point_cloud, x_c[0], y_c[0])
    dr = depth_of_pixel(point_cloud, x_c[1], y_c[1])

    if x_c[0] > x_c[1]: #cones are switched
        dl, dr = dr, dl
        x_c[0], x_c[1] = x_c[1], x_c[0]
    print(*x_c)

    print(dl, dr)
    
    return count_pos(dl, dr, x_c)


def main():
    global event
    event = threading.Event()
    turtle.register_bumper_event_cb(button_event)
    event.set()
    
    # Start the second task in a separate thread
    program_thread = threading.Thread(target=second_task)
    program_thread.start()
    
    rate = Rate(5)
    while not turtle.is_shutting_down():
        rate.sleep()
    
    # Ensure the thread stops when the program ends
    event.set()
    program_thread.join()

if __name__ == "__main__":
    main()

