from math import sqrt, pi, cos, acos, asin, sin, atan
from functions import hood_depth

#TODO problemove hodnoty pro vypocet
#87 201
#1.3899600505828857 4.033840217590332

def count_pos(dl, dr, x_coords):
    a = 0.75
    b = dl + 0.025
    c = dr + 0.025
    eq = lambda x, y: abs(x-y) < 0.1

    beta = acos((b**2 - c**2 - a**2)/(-2*c*a))
    alpha = acos((a**2 - b**2 - c**2)/(-2*b*c))
    print("alpha:", alpha*57)
    print("beta:", beta*57)

    y = c*sin(beta)
    xr = sqrt(c**2 - y**2)
    xl = sqrt(b**2 - y**2)
    x = 0

    if eq(xl+xr,0.7):
        x = - xr
    elif eq(xr - xl, 0.7):
        x = -xr
    elif eq(xl - xr, 0.7):
        x = xr
    else:
        print("Chyba, výpočet souřadnic je špatně.")
    
    return [x + 0.35, y, count_rotation(alpha, beta, x_coords[0], x_coords[1])]


#jsou sloupky dostatečně ve středu obrázku?
def centered(x1, x2):
    m1 = min(x1, 641-x1)
    m2 = min(x2, 641-x2)
    mx = min(m1, m2)
    print("Centered:", mx)
    return mx > 20 

def count_rotation(coords):
    # 320 - half of the picture, 6 - Bulgarian constant
    return (coords[0] - 320)*pi / (6*180)

def pos_behind_the_ball(ball_pos, dst):
    alpha = atan(ball_pos[0]/ball_pos[1])
    radius = sqrt(ball_pos[0]**2 + ball_pos[1]**2)
    return [(radius+dst)*sin(alpha), (radius + dst)*cos(alpha), 0]

def pixel_to_angle(x_pixel):
    return -(x_pixel - 320)*pi/(320*4)

#we assume that the ball is directly in front of the robot
def count_relative_ball_pos(b_coords, curr_pos, turtle):
    turtle.wait_for_point_cloud()
    point_cloud = turtle.get_point_cloud()
    ball_dst = hood_depth(point_cloud, *b_coords) + 0.15 #+ diameter
    print("estimated ball dst:", ball_dst)

    #alpha = pixel_to_angle(b_coords[0]) #TODO odladit převod
    alpha = 0
    beta  = curr_pos[2]
    print("Beta:", beta*57)
    angle = -beta
    print(ball_dst*sin(angle))
    return [ball_dst*sin(angle), ball_dst*cos(angle), 0]

def count_rotation(alpha, beta, xl, xr):
    delta = alpha*(xr-320)/(xr-xl) #rad
    print("delta:", delta*57)
    gamma = acos(sin(beta))
    print("gamma:", gamma*57)
    rot = gamma - delta
    print("rot:", rot*57)
    #we counted rot neccessary to be straigth, but we want it in opposite way
    return -rot

    
