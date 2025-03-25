from functions import find_ball, find_ball_center, count_angular_speed, find_soccer_net_array, depth_of_pixel
from robolab_turtlebot import Turtlebot, Rate, get_time
from simple_pid import PID
import numpy as np

def incremental_go_straight(wanted_dist, max_speed, turtle: Turtlebot):
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

def rotate(rot_rad, turtle: Turtlebot):

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

def go_straight(meters, turtle:Turtlebot):
    t = get_time()
    time = abs(meters/0.1)
    dr = np.sign(meters)
    rate = Rate(40)
    
    while get_time() - t < time:
        turtle.cmd_velocity(linear=0.1*dr)
        rate.sleep()

def go_to_pos(curr_pos, pos, turtle:Turtlebot):
    
    rotate(-curr_pos[2], turtle)
    print('y dst:', curr_pos[1] - pos[1])
    incremental_go_straight(curr_pos[1] - pos[1], 1, turtle)
    flag = False
    if curr_pos[0] - pos[0] > 0:
        rotate(pi/2, turtle)
        flag = True
    else:
        rotate(-pi/2)
    incremental_go_straight(abs(curr_pos[0] - pos[0]), 1, turtle)
        
    if flag:
        rotate(-pi/2, turtle)
    else:
        rotate(pi/2, turtle)

    rotate(-pos[2], turtle)


