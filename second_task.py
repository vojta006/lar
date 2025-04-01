from __future__ import print_function
import cv2
import numpy as np
import threading
from robolab_turtlebot import Turtlebot, Rate, get_time
from math import pi
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
        break
    return cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

#alternative
#def second_task():
#
#    event.wait()  # wait for button push
#    
#    success = False
#
#    while not success:
#
#        if rotate_to_center_ball() is None:
#            pos = count_robot_pos()
#            if pos is None:
#                #We can't see the ball nor the pillars
#                #TODO run some safety function
#                return None
#            go_to_pos(pos, [0, 2, 0], turtle)
#            rotate_to_center_ball() #this may fail
#    
#        #go up to 1 meter from the ball
#        ball_dst = measure_ball_dst() #it can return None
#        if ball_dst > 1
#            incremental_go_straight(ball_dst - 1, 1, turtle)
#    
#        pos = count_robot_pos()
#        go_to_pos(pos, [0, 2, 0], turtle)
#    
#        pos = count_robot_pos()
#        rot = rotate_to_center_ball()
#        pos[2] += rot[2] #we have to update rotation
#        rel_ball_pos = scan_ball_pos(pos, turtle) #this may fail
#        ball_pos = [pos[0] + rel_ball_pos[0], pos[1] - rel_ball_pos[1], 0]
#        pos_behind_the_ball = pos_behind_the_ball(ball_pos, 1.5)
#        go_to_pos(pos, pos_behind_the_ball, turtle)
#    
#        #TODO call PID function
#        if pid_regulator():
#            success = True

def second_task():
    event.wait()  # wait for button push

    pos = count_robot_pos()

    if pos is None:
        print("Nevidím sloupky")
        return

    print(*pos)

    go_to_pos(pos, [0, 2, 0], turtle)

    pos = get_robot_pos()
    rot = rotate_to_center_ball()
    pos[2] += rot[2]

    #pos = count_robot_pos() #x, y, rot; or None - then call this function in loop with rotating

    #dest = [0, 3, 0]
    #go_to_pos(pos, dest, turtle)
    #pos = get_robot_pos()

    #rotate to be aligned
    rel_ball_pos = scan_ball_pos(pos, turtle) #if it cannot see the ball, it returns None
    print("relative pos", *rel_ball_pos)

    if rel_ball_pos is None:
        print("Nevidíme míč.")
        return
    
    ball_pos = [pos[0] + rel_ball_pos[0], pos[1] - rel_ball_pos[1], 0]
    print("Absolute ball pos:", *ball_pos)
    behind_the_ball = pos_behind_the_ball(ball_pos, 1.5)
    print("Pos behind_the_ball:", *behind_the_ball)
    go_to_pos(pos, behind_the_ball, turtle)

    #go_to_pos(dest , pos_behind_the_ball(ball_pos, 0.5)) #TODO count new precise pos instead of dest


#TODO return None if it rotates more time around and cannot see the ball
def rotate_to_center_ball(precision = 10): 
    
    rate = Rate(100)
    turtle.reset_odometry()
    turtle.wait_for_odometry()

    while True:
        hsv_img = take_hsv_picture()
        ball = find_ball(hsv_img)

        if ball is None:
            turtle.cmd_velocity(angular=0.6)
            rate.sleep()
        else:
            b_coords = find_ball_center(ball)
            print(b_coords[0]-320)
            if abs(320 - b_coords[0]) < precision:
                turtle.cmd_velocity(angular=0)
                break
            else:
                speed = 0.3
                if abs(b_coords[0] - 320) < 40:
                    speed = 0.15

                elif abs(b_coords[0] - 320) < 20:
                    speed = 0.05

                if b_coords[0] > 320:
                    speed = -speed

                turtle.cmd_velocity(angular=speed)

    return turtle.get_odometry()

#we assume that the rotation is known and noted in curr_pos
def scan_ball_pos(curr_pos, turtle): 
    hsv_img = take_hsv_picture()
    ball = find_ball(hsv_img)

    if ball is None:
        return None
    
    #we can compute the position of the ball
    b_coords = find_ball_center(ball)
    print("souřadnice míče v obrázku", *b_coords)

    return count_relative_ball_pos(b_coords, curr_pos, turtle)

def measure_ball_dst():
    hsv_img = take_hsv_picture()
    ball = find_ball(hsv_image)
    if ball is None:
        return None
    b_center = find_ball_center(ball)
    point_cloud = turtle.get_point_cloud()
    dst = hood_depth(point_cloud, *b_coords)
    return dst


#TODO wait for odometry sleep
def rotate_to_center_net():
    
    #rotation parts
    parts = 12
    for i in range(parts):
        hsv_image = take_hsv_picture()
        net, x_c, y_c = find_soccer_net_array(hsv_image)

        if net[0] is not None and net[1] is not None:
            return i*2*pi/parts #how much had we rotate 

        rotate(2*pi/parts, turtle)

    #We could not find pillars
    return None

#from one position, without moving in space
def count_robot_pos():
    successful = 0
    pictures_to_take = 3 #take three pictures

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

