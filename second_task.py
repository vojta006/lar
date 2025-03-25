from __future__ import print_function
import cv2
import numpy as np
import threading
from robolab_turtlebot import Turtlebot, Rate, get_time
from math import sqrt, pi, cos, acos, asin, sin, atan
from time import sleep

from calc import *
from movements import * 
from functions import find_ball, find_ball_center, count_angular_speed, find_soccer_net_array, hood_depth

# Initialize the Turtlebot instance
turtle = Turtlebot(rgb=True, depth=True, pc=True)

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

    
#stop on bounce
def follow_ball_line():
    pass

#TODO rozchodit funkci na detekci pozice míče
#TODO pohyb po hřišti moc nefunguje


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

#TODO wait for odometry sleep
def rotate_to_center_net():
    
    parts = 12
    for i in range(parts):
        hsv_image = take_hsv_picture()
        net, x_c, y_c = find_soccer_net_array(hsv_image)

        if net[0] is not None and net[1] is not None and centered(*x_c):
            return i*2*pi/parts #how much had we rotate 

        rotate(2*pi/parts, turtle)

    #We could not find pillars
    return None

#from one position, without moving in space
def count_robot_pos():
    successful = 0
    pictures_to_take = 1

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
    
    dl = hood_depth(point_cloud, x_c[0], y_c[0])
    dr = hood_depth(point_cloud, x_c[1], y_c[1])

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

