from math import sqrt, pi, cos, acos, asin, sin, atan

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

def count_relative_ball_pos(b_coords, curr_pos):
    turtle.wait_for_point_cloud()
    point_cloud = turtle.get_point_cloud()
    ball_dst = hood_depth(point_cloud, *b_coords) + 0.15 #+ diameter
    print("estimated ball dst:", ball_dst)
    alpha = pixel_to_angle(b_coords[0]) #TODO odladit převod
    print("alpha:", alpha)
    beta  = curr_pos[2]
    #rel_rob_x = ball_dst*sin(alpha) #relative x coordinate to robot coord system
    #rel_rob_y = ball_dst*cos(alpha)
    angle = beta - alpha
    return [ball_dst*cos(angle), ball_dst*sin(angle), 0]

def count_rotation(alpha, beta, xl, xr):
    rot = None
    if beta > pi/2:
        rot = beta - pi/2
        rot += (xr - 320)*alpha/(xr-xl)
    else:
        rot = pi/2 - beta
        rot += (xr - 320)*alpha/(xr-xl)

    return rot


