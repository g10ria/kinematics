import matplotlib.pyplot as plt
import numpy as np
import math
from IPython.display import display, clear_output

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.set_xlim(0, 6)
ax.set_ylim(0, 6)

# travel speed
speed = 0.25
# time interval between plotting points (in seconds)
interval = 0.125

def distance(x1, x2):
    d = [x2[0]-x1[0], x2[1]-x1[1]]
    return math.sqrt(d[0]*d[0] + d[1]*d[1])

def plot(x):
    ax.plot(x[0], x[1], ".")
    display(fig)
    clear_output(wait=True)
    plt.pause(interval)

def midpoint(x1, x2):
    return [(x1[0]+x2[0])/2, (x1[1]+x2[1])/2]

# returns the angle between the x-axis and the given vector. assumes the vector is not [0,0]
def vector_angle(x):
    unit_vector = x / np.linalg.norm(x) 
    x_axis = [1,0]
    dot_product = np.dot(unit_vector, x_axis)
    angle = np.arccos(dot_product)
    if x[1] < 0: angle = 2*math.pi - angle
    return angle

# returns a vector from x1 to x2
def vector_from(x1, x2):
    return [x2[0]-x1[0], x2[1]-x1[1]]

def path(start, end):
    dir_vector = [end[0]-start[0], end[1]-start[1]]
    path_distance = distance(start, end)
    total_time = path_distance/speed
    offset_vector = [dir_vector[0]/total_time, dir_vector[1]/total_time]

    curr_position = [start[0], start[1]]
    for t in range(int(total_time)+1):
        plot(curr_position)
        curr_position[0] += offset_vector[0]
        curr_position[1] += offset_vector[1]
    
    if(path_distance % speed != 0):
        plot(end)

def polygon(points):
    for x in range(len(points)-1):
        path(points[x], points[x+1])
    path(points[-1], points[0])
    plt.show()

# only travels through a given angle (in radians). set direction to 1 for clockwise and -1 for counter
def centerpoint_circle(center, point, travel_angle, direction, show):
    radius = distance(center, point)
    # center to point
    ctp_vector = vector_from(center, point)
    # calculated counter-clockwise from x-axis
    start_angle =  vector_angle(ctp_vector)
    curr_angle = start_angle
    total_time = travel_angle*radius / speed
    angle_offset = travel_angle / total_time * -direction

    for t in range(int(total_time)+1):
        xcoord = center[0] + radius * math.cos(curr_angle)
        ycoord = center[1] + radius * math.sin(curr_angle)
        plot([xcoord, ycoord])  
        curr_angle += angle_offset
    if(travel_angle*radius % speed != 0):
        plot([center[0] + radius * math.cos(start_angle + travel_angle), center[1] + radius * math.sin(start_angle + travel_angle)])
    if show: plt.show

# makes a loop using the rectangle formed by the 4 given points
# semicircles are attached to the sides formed by x2-x3 and x4-x1
def loop(x1, x2, x3, x4):
    path(x1, x2)
    #determine circle direction

    # tilted
    if (x1[0] != x2[0] and x1[1] != x2[1]):
        b1 = vector_from(x2, x1)
        b2 = vector_from(x2, x3)
        angle1 = vector_angle(b1) - math.pi
        angle2 = vector_angle(b2) - math.pi
        sign_change = angle1 * angle2 < 0
        theta = vector_angle(b2)
        direction = -1 if sign_change else 1
        if (0 < theta and theta < math.pi/2 or math.pi < theta and theta < math.pi * 3/2):
            direction*=-1 
    elif x1[0] == x2[0]:
        direction = -1 if x3[0]<x2[0] else 1
        if x2[1]<x1[1]: direction *= -1
    else:
        direction = -1 if x3[1]<x2[1] else 1
        if x2[0]>x1[0]: direction *= -1

    centerpoint_circle(midpoint(x2, x3), x2, math.pi, direction, False)
    path(x3, x4)
    centerpoint_circle(midpoint(x4,x1), x4, math.pi, direction, True)


# --------- TESTS ---------

# PLOTTING POLYGON
# quad1 = [[3, 2], [5, 3], [4, 5], [1, 5]]
# polygon(quad1)

# PLOTTING CIRCLE
# circle_center = [3,3]
# circle_point = [1,2]
# centerpoint_circle(circle_center, circle_point, 2*math.pi, -1, True)

# LOOP TEST CASES

# tilted1
x1 = [1,2]
x2 = [2,1]
x3 = [3,2]
x4 = [2,3]

#tilted2
# x1 = [3,2]
# x2 = [2,1]
# x3 = [1,2]
# x4 = [2,3]

# not tilted
# x1 = [2,2]
# x2 = [2,1]
# x3 = [3,1]
# x4 = [3,2]

loop(x1, x2, x3, x4)