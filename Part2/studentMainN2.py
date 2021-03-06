# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position.
#
# ----------
# GRADING
#
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
import random
import numpy as np

#from matrix import * # Check the matrix.py tab to see how this works.


# This is the function you have to write. Note that measurement is a
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.


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



def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi


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

def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    #OTHER = [x, P, measurement_l, angle_l]


    #KALMAN MATRICIES
    #[angle, bent, distance]


    u = matrix([[0.], [0.], [0.]])  # external motion
    F = matrix([[1., 1., 0.], [0., 1., 0.], [0., 0., 1.]])  # next state function
    H = matrix([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.,]])  # measurement function
    R = matrix([[3.7, 1.0, 2.0], [2.5, 2.0, 2.0], [2.5, 2.0, 3.0]])  # measurement uncertainty
    I = matrix([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])  # identity matrix



    if OTHER == None:
        measurement_l = measurement
        OTHER = [measurement_l]
        return (0,0), OTHER

    elif len(OTHER) <= 1:
        angle, distance = calcPolarChangeBtw2Points(OTHER[0], measurement)
        angle =  angle_trunc(angle)


        x = matrix([[angle], [0], [0]])  # initial state (location and velocity)
        P = matrix([[7.0, 5.0, 2.0], [3.0, 5.0, 3.0], [4.0, 4.0, 3.0]])  # initial uncertainty
        OTHER = [x, P, measurement, angle]
        return (0,0), OTHER

    elif len(OTHER) <= 4:

        angle, distance = calcPolarChangeBtw2Points(OTHER[2], measurement)
        print('unaltered actual angle', angle)
        angle = angle_trunc(angle)
        print(angle, 'actual angle')

        a0 = angle_trunc(angle - OTHER[3])
        d0 = distance_between(OTHER[2], measurement)
        print('a0', a0)
        print('d0', d0)

        x = OTHER[0]
        P = OTHER[1]
        print('x', x)

        resid = x.value[0][0] - angle

        if abs(resid) > pi:
            print('SKIP')

            x_n = (F * x) + u
            P_n = F * P * F.transpose()

            OTHER = [x_n, P_n, measurement, angle]

            vals = x_n.value
            vals[0][0] = angle_trunc(vals[0][0])
            x_n.setValue(vals)


            X_n, Y_n = measurement
            print('xn angle', x_n.value[0][0])
            print('xn angle trunc', angle_trunc(x_n.value[0][0]))
            print('------')
            X_n += x_n.value[2][0] * cos(x_n.value[0][0])
            Y_n += x_n.value[2][0] * sin(x_n.value[0][0])
            xy_estimate = (X_n, Y_n)

            return xy_estimate, OTHER

        Z = matrix([[angle], [a0], [d0]])


        y = Z - (H * x)
        print('y', y)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        print('Kalman Gain', (K * y))
        P = (I - (K * H)) * P

        #prediction
        x_n = (F * x) + u
        P_n = F * P * F.transpose()

        vals  = x_n.value
        vals[0][0] = angle_trunc(vals[0][0])
        x_n.setValue(vals)

        #print('xn', x_n.value)
        OTHER = [x_n, P_n, measurement, angle]

        #print('angle', x_n.value[0][0])
        #d = x_n.value[1][0]
        #print('d', x_n.value[1][0])

        X_n, Y_n = measurement
        print('xn angle', x_n.value[0][0])
        print('xn angle trunc', angle_trunc(x_n.value[0][0]))
        print('------')
        X_n += x_n.value[2][0] * cos(x_n.value[0][0])
        Y_n += x_n.value[2][0] * sin(x_n.value[0][0])
        xy_estimate = (X_n, Y_n)

        # You must return xy_estimate (x, y), and OTHER (even if it is None)
        # in this order for grading purposes.

        return xy_estimate, OTHER

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any
# information that you want.
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        #print('true pos', true_position)
        #print('guess', position_guess)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
    return localized

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
print('noise', measurement_noise)
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(estimate_next_pos, test_target)

