# ----------
# Part Three
#
# Now you'll actually track down and recover the runaway Traxbot.
# In this step, your speed will be about twice as fast the runaway bot,
# which means that your bot's distance parameter will be about twice that
# of the runaway. You can move less than this parameter if you'd
# like to slow down your bot near the end of the chase.
#
# ----------
# YOUR JOB
#
# Complete the next_move function. This function will give you access to
# the position and heading of your bot (the hunter); the most recent
# measurement received from the runaway bot (the target), the max distance
# your bot can move in a given timestep, and another variable, called
# OTHER, which you can use to keep track of information.
#
# Your function will return the amount you want your bot to turn, the
# distance you want your bot to move, and the OTHER variable, with any
# information you want to keep track of.
#
# ----------
# GRADING
#
# We will make repeated calls to your next_move function. After
# each call, we will move the hunter bot according to your instructions
# and compare its position to the target bot's true position
# As soon as the hunter is within 0.01 stepsizes of the target,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.
#
# As an added challenge, try to get to the target bot as quickly as
# possible.

from robot import *
from math import *
#from matrix import *
import random



class matrix:

    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0

    def setValue(self, lst):
        self.value = lst

    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError("Invalid size of matrix")
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]

    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError("Invalid size of matrix")
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1

    def show(self):
        for i in range(self.dimx):
            print
            self.value[i]
        print
        ' '

    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError("Matrices must be of equal dimension to add")
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res

    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError("Matrices must be of equal dimension to subtract")
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res

    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError("Matrices must be m*n and n*p to multiply")
        else:
            # multiply if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
        return res

    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res

    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        # This code is based on http://adorio-research.org/wordpress/?p=4560
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        for i in range(self.dimx):
            S = sum([(res.value[k][i]) ** 2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError("Matrix not positive-definite")
                res.value[i][i] = sqrt(d)
            for j in range(i + 1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(i)])
                if abs(S) < ztol:
                    S = 0.0
                try:
                    res.value[i][j] = (self.value[i][j] - S) / res.value[i][i]
                except:
                    raise ValueError("Zero diagonal")
        return res

    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        # This code is based on http://adorio-research.org/wordpress/?p=4560

        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k] * res.value[j][k] for k in range(j + 1, self.dimx)])
            res.value[j][j] = 1.0 / tjj ** 2 - S / tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum(
                    [self.value[i][k] * res.value[k][j] for k in range(i + 1, self.dimx)]) / self.value[i][i]
        return res

    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res

    def __repr__(self):
        return repr(self.value)

def gridToPolar(point):
    x, y = point
    angle = atan(float(y)/x)
    d = distance_between((0,0), point)
    return (angle, d)

def polarToGrid(point):
    angle, d = point
    x = cos(angle) * d
    y = sin(angle) * d
    return (x, y)

def calcPolarChangeBtw2Points(point1, point2):
    x0, y0 = point1
    x1, y1 = point2


    opp = y1 - y0
    adj = x1 - x0

    if opp > 0 and adj > 0:
        angle = atan(float(opp)/adj)
    if opp > 0 and adj < 0:
        angle = atan(float(opp) / adj)
        angle = pi + angle
    if opp < 0 and adj < 0:
        angle = atan(float(opp) / adj)
        angle = pi + angle
    if opp < 0 and adj > 0:
        angle = atan(float(opp) / adj)
        angle = 2 * pi + angle

    d = distance_between(point1, point2)
    return (angle, d)




def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
    # This function will be called after each time the target moves.

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.

    # OTHER = [x, P, measurement_l, angle_l, hunter_position, hunter_bearing, initial_distance]


    # KALMAN MATRICIES
    # [angle, bent, distance]

    measurement = target_measurement

    move_measurement = measurement


    u = matrix([[0.], [0.], [0.]])  # external motion
    F = matrix([[1., 1., 0.], [0., 1., 0.], [0., 0., 1.]])  # next state function
    H = matrix([[1., 0., 0.], [0., 1., 0.], [0., 0., 1., ]])  # measurement function
    R = matrix([[1.0, 0.3, 0.3], [1.0, 0.3, 0.3], [0.3, 0.3, 1.0]])  # measurement uncertainty
    I = matrix([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])  # identity matrix

    if OTHER == None:
        measurement_l = measurement
        initial_distance = distance_between(hunter_position, measurement)
        OTHER = [measurement_l, hunter_position, hunter_heading, initial_distance]


    elif len(OTHER) <= 4:
        angle, distance = calcPolarChangeBtw2Points(OTHER[0], measurement)
        angle = angle_trunc(angle)

        x = matrix([[angle], [0], [0]])  # initial state (location and velocity)
        P = matrix([[5.0, 1.0, 1.0], [1.0, 5.0, 1.0], [1.0, 1.0, 5.0]])  # initial uncertainty
        OTHER = [x, P, measurement, angle, hunter_position, hunter_heading, OTHER[3]]


    elif len(OTHER) <= 7:

        angle, distance = calcPolarChangeBtw2Points(OTHER[2], measurement)
        #print('unaltered actual angle', angle)
        angle = angle_trunc(angle)
        #print(angle, 'actual angle')

        a0 = angle_trunc(angle - OTHER[3])
        d0 = distance_between(OTHER[2], measurement)
        #print('a0', a0)
        #print('d0', d0)

        x = OTHER[0]
        P = OTHER[1]


        resid = x.value[0][0] - angle

        if abs(resid) > abs(4 * a0):
            #print('SKIP')

            x_n = (F * x) + u
            P_n = F * P * F.transpose()

            OTHER = [x_n, P_n, measurement, angle, hunter_position, hunter_heading, OTHER[6]]

            vals = x_n.value
            vals[0][0] = angle_trunc(vals[0][0])
            x_n.setValue(vals)

            X_n, Y_n = measurement
            #print('xn angle', x_n.value[0][0])
            #print('xn angle trunc', angle_trunc(x_n.value[0][0]))

            X_n += x_n.value[2][0] * cos(x_n.value[0][0])
            Y_n += x_n.value[2][0] * sin(x_n.value[0][0])

        else:
            Z = matrix([[angle], [a0], [d0]])

            y = Z - (H * x)
            #print('y', y)
            S = H * P * H.transpose() + R
            K = P * H.transpose() * S.inverse()
            x = x + (K * y)
            #print('Kalman Gain', (K * y))
            P = (I - (K * H)) * P

            # prediction
            x_n = (F * x) + u
            P_n = F * P * F.transpose()

            vals = x_n.value
            vals[0][0] = angle_trunc(vals[0][0])
            x_n.setValue(vals)

            OTHER = [x_n, P_n, measurement, angle, hunter_position, hunter_heading, OTHER[6]]

            X_n, Y_n = measurement
            #print('xn angle', x_n.value[0][0])
            #print('xn angle trunc', angle_trunc(x_n.value[0][0]))

            X_n += x_n.value[2][0] * cos(x_n.value[0][0])
            Y_n += x_n.value[2][0] * sin(x_n.value[0][0])

        target_distance = distance_between(hunter_position, measurement)

        '''
        print('target position', measurement)
        print('hunter position', hunter_position)
        print('distance btw bots', target_distance)
        steps_ahead = 1
        if target_distance >= hunter.distance * 2:
            steps_ahead = 2
        elif target_distance >= hunter.distance * 1.0:
            steps_ahead = 2
        elif target_distance >= 0.0:
            steps_ahead = 1
        

        print('steps ahead', steps_ahead)
        X_sim, Y_sim = measurement
        bearing_sim = x.value[0][0]
        turn_sim = x.value[1][0]
        distance_sim = x.value[2][0]
        simbot = robot(X_sim, Y_sim, bearing_sim, turn_sim, distance_sim)
        for i in range(0, steps_ahead):
            print('simbot moving!')
            simbot.move_in_circle()
        move_measurement = simbot.sense()
        print('------')
        '''

        #print('target position', measurement)
        #print('hunter position', hunter_position)
        #print('distance btw bots', target_distance)
        steps_ahead = 1

        X_sim, Y_sim = measurement
        bearing_sim = x.value[0][0]
        turn_sim = x.value[1][0]
        distance_sim = x.value[2][0]
        simbot = robot(X_sim, Y_sim, bearing_sim, turn_sim, distance_sim)
        #print('simbot moving!')
        for i in range(1, 100):
            simbot.move_in_circle()
            simxy = simbot.sense()
            dis = distance_between(hunter_position, simxy)
            if (i * float(hunter.distance)) >= dis:
                move_measurement = simxy
                steps_ahead = i
                break


        #print('steps ahead', steps_ahead)
        #print('------')

    heading_to_target, move_distance = calcPolarChangeBtw2Points(hunter_position, move_measurement)
    heading_to_target = angle_trunc(heading_to_target)

    #print('current hunter heading', hunter_heading)
    #print('current angle target', heading_to_target)
    '''
    if heading_to_target * hunter_heading >= 0:
        heading_difference = heading_to_target - hunter_heading
    else:
        if heading_to_target < 0 and hunter_heading > 0:
            heading_difference = heading_to_target + 2*pi - hunter_heading
        elif heading_to_target > 0 and hunter_heading < 0:
            heading_difference = heading_to_target - 2 * pi - hunter_heading
    
    turning = heading_difference  # turn towards the target
    '''
    turning = heading_to_target - hunter_heading
    turning = angle_trunc(turning)
    #print('how much the robot should turn', turning)
    distance = min(max_distance, move_distance)  # full speed ahead!

    return turning, distance, OTHER


def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER=None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance  # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance  # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance,
                                                 OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        #print('Hunter Bot Moving')
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        #print('Target Bot Moving')
        target_bot.move_in_circle()

        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught


def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi


def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading


def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all
    the target measurements, hunter positions, and hunter headings over time, but it doesn't
    do anything with that information."""
    if not OTHER:  # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings)  # now I can keep track of history
    else:  # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER  # now I can always refer to these variables

    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference  # turn towards the target
    distance = max_distance  # full speed ahead!
    return turning, distance, OTHER


GLOBAL_PARAMETERS = [None,

        {'test_case': 1,
     'target_x': 6.38586153722,
     'target_y': 13.4105567386,
     'target_heading': 2.47241215877,
     'target_period': -25,
     'target_speed': 3.79845282159,
     'hunter_x': -4.15461096841,
     'hunter_y': -0.3225704554,
     'hunter_heading': 2.53575760878
    },
    {'test_case': 2,
     'target_x': 16.585052609,
     'target_y': 9.0679044122,
     'target_heading': -1.35786342037,
     'target_period': -37,
     'target_speed': 1.28476921126,
     'hunter_x': 10.8662448888,
     'hunter_y': 14.7856356957,
     'hunter_heading': 0.356152836897
    },
    {'test_case': 3,
     'target_x': 14.2062592559,
     'target_y': -18.0245447208,
     'target_heading': -2.38262617883,
     'target_period': -49,
     'target_speed': 1.83862303037,
     'hunter_x': -2.82628668059,
     'hunter_y': -8.94637942004,
     'hunter_heading': -0.220346285164
    },
    {'test_case': 4,
     'target_x': -11.8110077747,
     'target_y': -18.6564535804,
     'target_heading': -1.96611401851,
     'target_period': 43,
     'target_speed': 1.63703150728,
     'hunter_x': -11.6275149175,
     'hunter_y': 5.79288354591,
     'hunter_heading': -0.167236690344
    },
    {'test_case': 5,
     'target_x': 15.6527729222,
     'target_y': -0.647477557818,
     'target_heading': 2.53763865986,
     'target_period': -25,
     'target_speed': 3.30090641473,
     'hunter_x': 4.89061164952,
     'hunter_y': -3.67364934482,
     'hunter_heading': 0.69375353171
    },
    {'test_case': 6,
     'target_x': 4.19064615709,
     'target_y': -1.18147110409,
     'target_heading': -1.64836474843,
     'target_period': 15,
     'target_speed': 3.83139058798,
     'hunter_x': 1.58465033057,
     'hunter_y': -11.608873745,
     'hunter_heading': -1.71836625476
    },
    {'test_case': 7,
     'target_x': -14.9126298507,
     'target_y': 9.77381651339,
     'target_heading': -2.6049812496,
     'target_period': 15,
     'target_speed': 1.87228826655,
     'hunter_x': -1.73542429642,
     'hunter_y': 15.2209669071,
     'hunter_heading': -3.11279669928
    },
    {'test_case': 8,
     'target_x': -7.36186590331,
     'target_y': -16.8073975689,
     'target_heading': -0.521095102947,
     'target_period': 16,
     'target_speed': 1.99556521539,
     'hunter_x': -12.4391297878,
     'hunter_y': -17.4403250837,
     'hunter_heading': -2.7562509168
    },
    {'test_case': 9,
     'target_x': 8.12973829475,
     'target_y': -10.7703982486,
     'target_heading': -1.99007409394,
     'target_period': 50,
     'target_speed': 2.79327564984,
     'hunter_x': -6.10424606902,
     'hunter_y': -18.9750820343,
     'hunter_heading': -0.0275542431845
    },
    {'test_case': 10,
     'target_x': -18.2934552906,
     'target_y': 16.3903453417,
     'target_heading': 0.345582694568,
     'target_period': -16,
     'target_speed': 3.99258090205,
     'hunter_x': -18.1103477129,
     'hunter_y': 5.2801933801,
     'hunter_heading': 1.29663175758
    },
]





NUM = 7

#target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
target = robot(GLOBAL_PARAMETERS[NUM]['target_x'], GLOBAL_PARAMETERS[NUM]['target_y'], GLOBAL_PARAMETERS[NUM]['target_heading'], 2*pi / GLOBAL_PARAMETERS[NUM]['target_period'], GLOBAL_PARAMETERS[NUM]['target_speed'])

measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(GLOBAL_PARAMETERS[NUM]['hunter_x'], GLOBAL_PARAMETERS[NUM]['hunter_y'], GLOBAL_PARAMETERS[NUM]['hunter_heading'])

demo_grading(hunter, target, next_move)





