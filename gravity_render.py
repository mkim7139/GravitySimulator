import cv2 as cv
from numpy import *

####### GLOBAL
G=200              # gravitational const
TSTEP=0.1           # time increment (amount of simulation time that passes per frame)
FRAME_WIDTH=1500
FRAME_HEIGHT=850
FRAME_COUNT = 200
PLANET_COUNT = 7

####### PLANET CLASS
class Planet:
    def __init__(self, size, color, mass, position, velocity):
        self.size = size
        self.color = color
        self.mass = mass
        self.position = position
        self.velocity = velocity


####### DRAWING
def draw_planets(img, obj_array):
    thickness=-1
    line_type=8

    for i in range(len(obj_array)):
        position = (int(obj_array[i].position[0]), int(obj_array[i].position[1]))
        cv.circle(img, position, obj_array[i].size, obj_array[i].color, thickness, line_type)

####### PLANET GENERATOR
def random_planet():
    global planets

    mass=random.random()*2 + 1
    size=int(mass)

    # +/-
    s1 = random.randint(0, 2, 1)[0]
    s2 = random.randint(0, 2, 1)[0]

    # position
    x_center=FRAME_WIDTH/2
    y_center=FRAME_HEIGHT/2

    xcor= x_center + (random.random()*FRAME_WIDTH*0.01)*(-1)**s1
    ycor= y_center + (random.random()*FRAME_HEIGHT*0.02 + 100)#*(-1)**s2

    # r = random.random()*3 + 340
    # ycor = random.random()*3*(-1)**p - 339.1*(-1)**p
    # xcor = (r**2 - (ycor)**2)**0.5
    position=array([xcor, ycor])

    # velocity
    v_x=random.random()*2.3 + 2
    v_y=0#random.random()*10 - 5
    velocity=array([v_x, v_y])

    # colors
    b_channel=int(random.random()*124 + 130)
    g_channel=10 #int(random.random()*210 + 44)
    r_channel=int(random.random()*124 + 30)
    color=(b_channel, g_channel, r_channel)

    return Planet(size, color, mass, position, velocity)


############## POSITION UPDATE FUNCTIONS
##############

def distance(obj_1, obj_2):
    return ((obj_2.position[0]-obj_1.position[0])**2 + (obj_2.position[1]-obj_1.position[1])**2)**0.5

def g_force(obj_1, obj_2):
    global G
    r = distance(obj_1, obj_2)
    if r<20:    # catch division by 0
        r=20

    m1 = obj_1.mass
    m2 = obj_2.mass
    return (G*m1*m2) / ((r*15)**2)

def force_vectors(obj_array):
    force_vecs = []

    for i in range(len(obj_array)):
        object_force = array([0, 0])
        for k in range(len(obj_array)):
            if (k!=i):
                r = distance(obj_array[i], obj_array[k])    # distance

                if r<0.1:  # catch division by 0
                    r=0.1

                u = array([obj_array[k].position[0] - obj_array[i].position[0], obj_array[k].position[1] - obj_array[i].position[1]]) / r # direction vector

                f_g = g_force(obj_array[i], obj_array[k])    # gravitational force
                object_force = object_force + (u * f_g)

        force_vecs = force_vecs + [object_force]

    return force_vecs

def accel_vectors(obj_array):
    f_vecs = force_vectors(obj_array)
    a_vecs = []

    for i in range(len(f_vecs)):
        a_vecs = a_vecs + [f_vecs[i] / obj_array[i].mass]

    return a_vecs

def update_velocities(obj_array):
    global TSTEP

    a_vecs = accel_vectors(obj_array)

    for i in range(len(obj_array)):
        obj_array[i].velocity = obj_array[i].velocity + a_vecs[i] * TSTEP

def update_coords(obj_array):
    global TSTEP

    update_velocities(obj_array)

    for i in range(len(obj_array)):
        disp_vec = obj_array[i].velocity * TSTEP
        obj_array[i].position = obj_array[i].position + disp_vec


##############
##############

####### PLANETS
#######
# p1_size = 12
# p1_color = (130, 40, 200)
# p1_mass = 12
# p1_pos = array([0, 0])
# p1_vel = array([10, 9])
# p1 = Planet(p1_size, p1_color, p1_mass, p1_pos, p1_vel)
#
# p2_size = 4
# p2_color = (230, 40, 130)
# p2_mass = 3
# p2_pos = array([FRAME_WIDTH/4, FRAME_HEIGHT/2])
# p2_vel = array([0, 200])
# p2 = Planet(p2_size, p2_color, p2_mass, p2_pos, p2_vel)

p3_size = 18
p3_color = (130, 200, 40)
p3_mass = 2000
p3_pos = array([FRAME_WIDTH/2, FRAME_HEIGHT/2])
p3_vel = array([0, 0])
p3 = Planet(p3_size, p3_color, p3_mass, p3_pos, p3_vel)

planets = array([p3])
for i in range(PLANET_COUNT):
    planets=append(planets, random_planet())

#######
#######


####### RENDERING
#######
#######
# video writer object
frame_size=(FRAME_HEIGHT, FRAME_WIDTH, 3)
video_size=(frame_size[1], frame_size[0])  ### reverse of image frame size
out = cv.VideoWriter('outpy.avi',cv.VideoWriter_fourcc('M','J','P','G'), 60, video_size)


for i in range(FRAME_COUNT):
    if i%4==0:
        # clear buffer
        frame = zeros(frame_size, dtype=uint8)

        # draw frame and write to output file
        draw_planets(frame, planets)
        out.write(frame)

    # render next frame
    update_coords(planets)
    # f_vecs = force_vectors(planets)
    #
    # p1.position = p1.position + f_vecs[0]
    # p2.position = p2.position + f_vecs[1]
    # p3.position = p3.position + f_vecs[2]


out.release()

cv.destroyAllWindows()