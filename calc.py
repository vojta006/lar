from math import sqrt, pi, cos, acos, asin, sin, atan
from functions import hood_depth

def count_pos(dl, dr, dp, x_coords):
    a = dp
    b = dl + 0.025
    c = dr + 0.025

    print('a:', a)
    print('b:', b)
    print('c:', c)

    beta = acos((b**2 - c**2 - a**2)/(-2*c*a))
    alpha = acos((a**2 - b**2 - c**2)/(-2*b*c))

    print("alpha:", alpha*57)
    print("beta:", beta*57)

    y = c*sin(beta)

    #x dst from right pillar
    xr = sqrt(c**2 - y**2)
    
    if beta < pi/2:
        xr = -xr
   
    return [xr + dp/2, y, count_rotation(alpha, beta, x_coords[0], x_coords[1])]


#jsou sloupky dostatečně ve středu obrázku?
def centered(x1, x2):
    m1 = min(x1, 641-x1)
    m2 = min(x2, 641-x2)
    mx = min(m1, m2)
    print("Centered:", mx)
    return mx > 20 


def pos_behind_the_ball(ball_pos, dst):
    alpha = atan(ball_pos[0]/ball_pos[1])
    radius = sqrt(ball_pos[0]**2 + ball_pos[1]**2)
    return [(radius+dst)*sin(alpha), (radius + dst)*cos(alpha), 0]

def pixel_to_angle(x_pixel):
    return -(x_pixel - 343)*0.0015

def count_relative_ball_pos(b_coords, curr_pos, turtle):
    turtle.wait_for_point_cloud()
    point_cloud = turtle.get_point_cloud()
    ball_dst = hood_depth(point_cloud, *b_coords) + 0.15 #+ diameter
    print("estimated ball dst:", ball_dst)

    alpha = pixel_to_angle(b_coords[0])
    beta = curr_pos[2]
    print("Beta:", beta*57)
    print("angle:", alpha)
    angle = -(beta + alpha)
    print(ball_dst*sin(angle))
    return [ball_dst*sin(angle), ball_dst*cos(angle), 0]

def count_rotation(alpha, beta, xl, xr):
    #print(alpha/(xr-xl)) #= 0.0015 rad/pixel
    #how much are we rotated from to be aligned with the right pillar
    delta = alpha*(xr-343)/(xr-xl) #rad
    pos = delta + beta - pi/2

    return pos
    
