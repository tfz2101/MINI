# ----------
# Background
#
# A robotics company named Trax has created a line of small self-driving robots
# designed to autonomously traverse desert environments in search of undiscovered
# water deposits.
#
# A Traxbot looks like a small tank. Each one is about half a meter long and drives
# on two continuous metal tracks. In order to maneuver itself, a Traxbot can do one
# of two things: it can drive in a straight line or it can turn. So to make a
# right turn, A Traxbot will drive forward, stop, turn 90 degrees, then continue
# driving straight.
#
# This series of questions involves the recovery of a rogue Traxbot. This bot has
# gotten lost somewhere in the desert and is now stuck driving in an almost-circle: it has
# been repeatedly driving forward by some step size, stopping, turning a certain
# amount, and repeating this process... Luckily, the Traxbot is still sending all
# of its sensor data back to headquarters.
#
# In this project, we will start with a simple version of this problem and
# gradually add complexity. By the end, you will have a fully articulated
# plan for recovering the lost Traxbot.
#
# ----------
# Part One
#
# Let's start by thinking about circular motion (well, really it's polygon motion
# that is close to circular motion). Assume that Traxbot lives on
# an (x, y) coordinate plane and (for now) is sending you PERFECTLY ACCURATE sensor
# measurements.
#
# With a few measurements you should be able to figure out the step size and the
# turning angle that Traxbot is moving with.
# With these two pieces of information, you should be able to
# write a function that can predict Traxbot's next location.
#
# You can use the robot class that is already written to make your life easier.
# You should re-familiarize yourself with this class, since some of the details
# have changed.
#
# ----------
# YOUR JOB
#
# Complete the estimate_next_pos function. You will probably want to use
# the OTHER variable to keep track of information about the runaway robot.
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
from robot import *
from math import *
from matrix import *
import random


# This is the function you have to write. The argument 'measurement' is a
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.


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




def estimate_next_pos(measurement, OTHER=None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    # OTHER = [(x0,y0), d, angle_0, r, hemisphere]

    xy_estimate = ''
    if OTHER == None:
        xy_estimate = measurement
        OTHER = [measurement]
    elif len(OTHER) == 1:
        d = distance_between(OTHER[0], measurement)

        x0, y0 = OTHER[0]
        x1, y1 = measurement

        opposite = y1 - y0
        angle_0 = asin(float(opposite) / d)
        xy_estimate = measurement
        OTHER = [measurement, d, angle_0]

    elif len(OTHER) >=  3:
        d = distance_between(OTHER[0], measurement)

        x0, y0 = OTHER[0]
        x1, y1 = measurement

        opposite = y1 - y0
        #angle_0 = asin(float(opposite) / d)

        angle_0, d = calcPolarChangeBtw2Points(OTHER[0],measurement)

        print('angle_0', angle_0)
        r = angle_0 - OTHER[2]

        angle_n = angle_0 + r
        x_n = cos(angle_n) * d + x1
        y_n = sin(angle_n) * d + y1
        xy_estimate = (x_n, y_n)
        OTHER = [measurement, d, angle_0, r]
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
    while not localized and ctr <= 10:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        print('true pos', true_position)
        print('guess', position_guess)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print("You got it right! It took you ", ctr, " steps to localize.")
            localized = True
        if ctr == 10:
            print("Sorry, it took you too many steps to localize the target.")
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

NUM = 2
test_target = robot(GLOBAL_PARAMETERS[NUM]['target_x'], GLOBAL_PARAMETERS[NUM]['target_x'], GLOBAL_PARAMETERS[NUM]['target_heading'], 2*pi / GLOBAL_PARAMETERS[NUM]['target_period'], GLOBAL_PARAMETERS[NUM]['target_speed'])
#test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)

test_target.set_noise(0.0, 0.0, 0.0)

demo_grading(estimate_next_pos, test_target)
