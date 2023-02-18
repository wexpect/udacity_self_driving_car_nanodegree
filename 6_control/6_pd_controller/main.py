# -----------
# User Instructions
#
# Implement a PD controller by running 100 iterations
# of robot motion. The steering angle should be set
# by the parameter tau_p and tau_d so that:
#
# steering = -tau_p * CTE - tau_d * diff_CTE
# where differential crosstrack error (diff_CTE)
# is given by CTE(t) - CTE(t-1)
#
#
# Only modify code at the bottom! Look for the TODO
# ------------

import random
import numpy as np
import matplotlib.pyplot as plt


# ------------------------------------------------
#
# this is the Robot class
#

class Robot(object):
    def __init__(self, length=20.0):
        """
        Creates robot and initializes location/orientation to 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters.
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift
        print('steering_drift', robot.steering_drift, 'steering2', steering2)

        # Execute motion
        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)


############## ADD / MODIFY CODE BELOW ####################
# ------------------------------------------------------------------------
#
# run - does a single control run

# previous P controller
def run_p(robot, tau, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    for i in range(n):
        cte = robot.y
        steer = -tau * cte
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
    return x_trajectory, y_trajectory




robot = Robot()
robot.set(0, 1, 0)

# NOTE: add 10 degrees of drift
import math
# robot.set_steering_drift(10 / 180 * math.pi)


def run(robot, tau_p, tau_d, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []

    # TODO: your code here
    print('start')
    print(robot)

    x_trajectory.append(robot.x)
    y_trajectory.append(robot.y)

    time_interval = 1
    dist = speed * time_interval

    cross_track_err_t_minus_1 = robot.y - 0

    for i in range(n):
        print('iteration', i)

        cross_track_err = robot.y - 0

        steering_p = -tau_p * cross_track_err

        diff_cte = (cross_track_err - cross_track_err_t_minus_1) / time_interval
        steering_d = -tau_d * diff_cte

        steering = steering_p + steering_d
        print('cross_track_err', cross_track_err, ', steering_p', steering_p, ', steering_d', steering_d, ', steering', steering)

        robot.move(steering, dist)
        print('after move,', robot)

        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)

        cross_track_err_t_minus_1 = cross_track_err

    return x_trajectory, y_trajectory

x_trajectory, y_trajectory = run(robot, 0.2, 3.0)
# x_trajectory, y_trajectory = run(robot, 2, 30, n=1000)

n = len(x_trajectory)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.plot(x_trajectory, y_trajectory, 'g', label='PD controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')

# TODO: your code here
ax1.legend()
plt.show()


'''
(env) Ruis-MacBook-Pro-15:6_pd_controller ruiwang$ python main.py
start
[x=0.00000 y=1.00000 orient=0.00000]
iteration 0
cross_track_err 1 , steering_p -0.2 , steering_d -0.0 , steering -0.2
after move, [x=0.99998 y=0.99493 orient=6.27305]
iteration 1
cross_track_err 0.9949322924956192 , steering_p -0.19898645849912386 , steering_d 0.015203122513142375 , steering -0.18378333598598148
after move, [x=1.99987 y=0.98015 orient=6.26376]
iteration 2
cross_track_err 0.9801503609607352 , steering_p -0.19603007219214705 , steering_d 0.044345794604652156 , steering -0.1516842775874949
after move, [x=2.99960 y=0.95690 orient=6.25611]
iteration 3
cross_track_err 0.9569015077820211 , steering_p -0.19138030155640423 , steering_d 0.06974655953614217 , steering -0.12163374202026206
after move, [x=3.99914 y=0.92678 orient=6.25000]
iteration 4
cross_track_err 0.9267777186062176 , steering_p -0.18535554372124352 , steering_d 0.09037136752741048 , steering -0.09498417619383304
after move, [x=4.99851 y=0.89122 orient=6.24524]
iteration 5
cross_track_err 0.891219155561572 , steering_p -0.17824383111231443 , steering_d 0.10667568913393666 , steering -0.07156814197837777
after move, [x=5.99772 y=0.85149 orient=6.24165]
iteration 6
cross_track_err 0.8514895056499086 , steering_p -0.17029790112998172 , steering_d 0.1191889497349905 , steering -0.05110895139499122
after move, [x=6.99680 y=0.80869 orient=6.23910]
iteration 7
cross_track_err 0.8086913609982957 , steering_p -0.16173827219965917 , steering_d 0.1283944339548384 , steering -0.03334383824482076
after move, [x=7.99579 y=0.76378 orient=6.23743]
iteration 8
cross_track_err 0.763782500297566 , steering_p -0.15275650005951322 , steering_d 0.13472658210218924 , steering -0.018029917957323977
after move, [x=8.99475 y=0.71804 orient=6.23653]
iteration 9
cross_track_err 0.7180405864126257 , steering_p -0.14360811728252515 , steering_d 0.13722574165482093 , steering -0.006382375627704218
after move, [x=9.99366 y=0.67140 orient=6.23621]
iteration 10
cross_track_err 0.6713980413510542 , steering_p -0.13427960827021085 , steering_d 0.13992763518471452 , steering 0.005648026914503668
after move, [x=10.99255 y=0.62444 orient=6.23649]
iteration 11
cross_track_err 0.6244367228748758 , steering_p -0.12488734457497518 , steering_d 0.14088395542853505 , steering 0.01599661085355987
after move, [x=11.99146 y=0.57776 orient=6.23729]
iteration 12
cross_track_err 0.5777574990419605 , steering_p -0.1155514998083921 , steering_d 0.14003767149874602 , steering 0.024486171690353914
after move, [x=12.99044 y=0.53249 orient=6.23851]
iteration 13
cross_track_err 0.5324889601827181 , steering_p -0.10649779203654362 , steering_d 0.13580561657772727 , steering 0.02930782454118365
after move, [x=13.98947 y=0.48856 orient=6.23998]
iteration 14
cross_track_err 0.48856426610484505 , steering_p -0.09771285322096901 , steering_d 0.13177408223361908 , steering 0.034061229012650074
after move, [x=14.98858 y=0.44622 orient=6.24168]
iteration 15
cross_track_err 0.44622286381832055 , steering_p -0.08924457276366411 , steering_d 0.1270242068595735 , steering 0.037779634095909384
after move, [x=15.98775 y=0.40568 orient=6.24357]
iteration 16
cross_track_err 0.40567671910923764 , steering_p -0.08113534382184753 , steering_d 0.12163843412724873 , steering 0.040503090305401196
after move, [x=16.98701 y=0.36709 orient=6.24560]
iteration 17
cross_track_err 0.36708711305681163 , steering_p -0.07341742261136233 , steering_d 0.11576881815727802 , steering 0.04235139554591569
after move, [x=17.98634 y=0.33057 orient=6.24772]
iteration 18
cross_track_err 0.3305685945812229 , steering_p -0.06611371891624458 , steering_d 0.1095555554267662 , steering 0.04344183651052162
after move, [x=18.98575 y=0.29619 orient=6.24989]
iteration 19
cross_track_err 0.29619487495472185 , steering_p -0.059238974990944375 , steering_d 0.10312115887950313 , steering 0.04388218388855875
after move, [x=19.98523 y=0.26400 orient=6.25209]
iteration 20
cross_track_err 0.2640044333219862 , steering_p -0.052800886664397244 , steering_d 0.09657132489820697 , steering 0.04377043823380972
after move, [x=20.98478 y=0.23401 orient=6.25428]
iteration 21
cross_track_err 0.23400565010365426 , steering_p -0.04680113002073086 , steering_d 0.08999634965499581 , steering 0.04319521963426495
after move, [x=21.98440 y=0.20618 orient=6.25644]
iteration 22
cross_track_err 0.20618146937937354 , steering_p -0.04123629387587471 , steering_d 0.08347254217284217 , steering 0.042236248296967455
after move, [x=22.98407 y=0.18049 orient=6.25855]
iteration 23
cross_track_err 0.18049360967000894 , steering_p -0.036098721934001786 , steering_d 0.0770635791280938 , steering 0.04096485719409202
after move, [x=23.98379 y=0.15689 orient=6.26060]
iteration 24
cross_track_err 0.15688634587831984 , steering_p -0.03137726917566397 , steering_d 0.07082179137506728 , steering 0.03944452219940331
after move, [x=24.98355 y=0.13529 orient=6.26257]
iteration 25
cross_track_err 0.1352898867940553 , steering_p -0.02705797735881106 , steering_d 0.06478937725279366 , steering 0.037731399893982595
after move, [x=25.98336 y=0.11562 orient=6.26446]
iteration 26
cross_track_err 0.11562337363079678 , steering_p -0.023124674726159356 , steering_d 0.05899953948977554 , steering 0.03587486476361619
after move, [x=26.98320 y=0.09780 orient=6.26626]
iteration 27
cross_track_err 0.09779752563986222 , steering_p -0.019559505127972443 , steering_d 0.05347754397280369 , steering 0.033918038844831244
after move, [x=27.98307 y=0.08172 orient=6.26795]
iteration 28
cross_track_err 0.08171695897101472 , steering_p -0.016343391794202945 , steering_d 0.04824170000654249 , steering 0.03189830821233954
after move, [x=28.98297 y=0.06728 orient=6.26955]
iteration 29
cross_track_err 0.06728220469392454 , steering_p -0.01345644093878491 , steering_d 0.043304262831270535 , steering 0.029847821892485624
after move, [x=29.98288 y=0.05439 orient=6.27104]
iteration 30
cross_track_err 0.054391451305718874 , steering_p -0.010878290261143776 , steering_d 0.038672260164617 , steering 0.027793969903473225
after move, [x=30.98282 y=0.04294 orient=6.27243]
iteration 31
cross_track_err 0.04294203621395809 , steering_p -0.008588407242791618 , steering_d 0.03434824527528235 , steering 0.025759838032490733
after move, [x=31.98277 y=0.03283 orient=6.27372]
iteration 32
cross_track_err 0.03283170964959936 , steering_p -0.006566341929919873 , steering_d 0.030330979693076188 , steering 0.023764637763156314
after move, [x=32.98273 y=0.02396 orient=6.27491]
iteration 33
cross_track_err 0.02395969329734271 , steering_p -0.004791938659468542 , steering_d 0.026616049056769953 , steering 0.02182411039730141
after move, [x=33.98270 y=0.01623 orient=6.27600]
iteration 34
cross_track_err 0.016227554672127553 , steering_p -0.003245510934425511 , steering_d 0.02319641587564547 , steering 0.01995090494121996
after move, [x=34.98267 y=0.00904 orient=6.27700]
iteration 35
cross_track_err 0.009041089873143171 , steering_p -0.0018082179746286343 , steering_d 0.021559394396953147 , steering 0.01975117642232451
after move, [x=35.98265 y=0.00285 orient=6.27798]
iteration 36
cross_track_err 0.0028522803441938293 , steering_p -0.0005704560688387659 , steering_d 0.018566428586848024 , steering 0.01799597251800926
after move, [x=36.98264 y=-0.00235 orient=6.27888]
iteration 37
cross_track_err -0.002348857982055093 , steering_p 0.00046977159641101857 , steering_d 0.015603414978746767 , steering 0.016073186575157786
after move, [x=37.98263 y=-0.00665 orient=6.27969]
iteration 38
cross_track_err -0.0066501107225334245 , steering_p 0.001330022144506685 , steering_d 0.012903758221434995 , steering 0.01423378036594168
after move, [x=38.98262 y=-0.01015 orient=6.28040]
iteration 39
cross_track_err -0.010147641051460434 , steering_p 0.0020295282102920868 , steering_d 0.010492590986781028 , steering 0.012522119197073115
after move, [x=39.98262 y=-0.01293 orient=6.28103]
iteration 40
cross_track_err -0.012933437822815851 , steering_p 0.0025866875645631704 , steering_d 0.008357390314066251 , steering 0.010944077878629423
after move, [x=40.98262 y=-0.01509 orient=6.28157]
iteration 41
cross_track_err -0.015093097831527031 , steering_p 0.0030186195663054064 , steering_d 0.00647898002613354 , steering 0.009497599592438948
after move, [x=41.98262 y=-0.01671 orient=6.28205]
iteration 42
cross_track_err -0.016705533078656024 , steering_p 0.003341106615731205 , steering_d 0.00483730574138698 , steering 0.008178412357118185
after move, [x=42.98262 y=-0.01784 orient=6.28246]
iteration 43
cross_track_err -0.017843074520272163 , steering_p 0.0035686149040544328 , steering_d 0.003412624324848415 , steering 0.006981239228902848
after move, [x=43.98261 y=-0.01857 orient=6.28281]
iteration 44
cross_track_err -0.0185716864075716 , steering_p 0.00371433728151432 , steering_d 0.0021858356618983076 , steering 0.005900172943412628
after move, [x=44.98261 y=-0.01895 orient=6.28310]
iteration 45
cross_track_err -0.018951230717844176 , steering_p 0.0037902461435688354 , steering_d 0.0011386329308177325 , steering 0.004928879074386568
after move, [x=45.98261 y=-0.01904 orient=0.00016]
iteration 46
cross_track_err -0.019035762966626242 , steering_p 0.0038071525933252486 , steering_d 0.0002535967463461984 , steering 0.004060749339671447
after move, [x=46.98261 y=-0.01887 orient=0.00036]
iteration 47
cross_track_err -0.018873849266787063 , steering_p 0.003774769853357413 , steering_d -0.0004857410995175379 , steering 0.003289028753839875
after move, [x=47.98261 y=-0.01851 orient=0.00053]
iteration 48
cross_track_err -0.018508896991342866 , steering_p 0.003701779398268573 , steering_d -0.0010948568263325908 , steering 0.0026069225719359823
after move, [x=48.98261 y=-0.01798 orient=0.00066]
iteration 49
cross_track_err -0.017979492701836033 , steering_p 0.003595898540367207 , steering_d -0.0015882128685204971 , steering 0.0020076856718467097
after move, [x=49.98261 y=-0.01732 orient=0.00076]
iteration 50
cross_track_err -0.01731974201158467 , steering_p 0.0034639484023169342 , steering_d -0.001979252070754091 , steering 0.0014846963315628433
after move, [x=50.98261 y=-0.01656 orient=0.00083]
iteration 51
cross_track_err -0.016559606928204344 , steering_p 0.003311921385640869 , steering_d -0.0022804052501409766 , steering 0.0010315161354998922
after move, [x=51.98261 y=-0.01573 orient=0.00089]
iteration 52
cross_track_err -0.015725236997309308 , steering_p 0.003145047399461862 , steering_d -0.0025031097926851084 , steering 0.0006419376067767534
after move, [x=52.98261 y=-0.01484 orient=0.00092]
iteration 53
cross_track_err -0.014839291260432073 , steering_p 0.0029678582520864145 , steering_d -0.0026578372106317062 , steering 0.0003100210414547083
after move, [x=53.98261 y=-0.01392 orient=0.00093]
iteration 54
cross_track_err -0.013921248651865418 , steering_p 0.002784249730373084 , steering_d -0.0027541278256999636 , steering 3.012190467312041e-05
after move, [x=54.98261 y=-0.01299 orient=0.00094]
iteration 55
cross_track_err -0.012987704997372488 , steering_p 0.0025975409994744977 , steering_d -0.0028006309634787917 , steering -0.00020308996400429397
after move, [x=55.98261 y=-0.01205 orient=0.00092]
iteration 56
cross_track_err -0.012052655248302789 , steering_p 0.002410531049660558 , steering_d -0.0028051492472090968 , steering -0.00039461819754853895
after move, [x=56.98261 y=-0.01113 orient=0.00091]
iteration 57
cross_track_err -0.011127759993181816 , steering_p 0.0022255519986363634 , steering_d -0.0027746857653629196 , steering -0.0005491337667265562
after move, [x=57.98261 y=-0.01022 orient=0.00088]
iteration 58
cross_track_err -0.010222595640701992 , steering_p 0.0020445191281403987 , steering_d -0.0027154930574394692 , steering -0.0006709739292990705
after move, [x=58.98261 y=-0.00934 orient=0.00084]
iteration 59
cross_track_err -0.009344887968408128 , steering_p 0.0018689775936816255 , steering_d -0.0026331230168815944 , steering -0.0007641454231999689
after move, [x=59.98261 y=-0.00850 orient=0.00081]
iteration 60
cross_track_err -0.008500728985178996 , steering_p 0.0017001457970357995 , steering_d -0.0025324769496873935 , steering -0.000832331152651594
after move, [x=60.98261 y=-0.00769 orient=0.00076]
iteration 61
cross_track_err -0.007694777267540021 , steering_p 0.0015389554535080044 , steering_d -0.0024178551529169257 , steering -0.0008788996994089212
after move, [x=61.98261 y=-0.00693 orient=0.00072]
iteration 62
cross_track_err -0.006930442104313658 , steering_p 0.0013860884208627315 , steering_d -0.0022930054896790905 , steering -0.000906917068816359
after move, [x=62.98261 y=-0.00621 orient=0.00068]
iteration 63
cross_track_err -0.006210051925260427 , steering_p 0.0012420103850520855 , steering_d -0.002161170537159693 , steering -0.0009191601521076076
after move, [x=63.98261 y=-0.00554 orient=0.00063]
iteration 64
cross_track_err -0.005535007601039036 , steering_p 0.0011070015202078072 , steering_d -0.002025132972664172 , steering -0.0009181314524563649
after move, [x=64.98261 y=-0.00491 orient=0.00058]
iteration 65
cross_track_err -0.004905921287591181 , steering_p 0.000981184257518236 , steering_d -0.0018872589403435662 , steering -0.0009060746828253301
after move, [x=65.98261 y=-0.00432 orient=0.00054]
iteration 66
cross_track_err -0.004322741551228357 , steering_p 0.0008645483102456714 , steering_d -0.001749539209088471 , steering -0.0008849908988427996
after move, [x=66.98261 y=-0.00378 orient=0.00049]
iteration 67
cross_track_err -0.003784865554283592 , steering_p 0.0007569731108567184 , steering_d -0.0016136279908342955 , steering -0.0008566548799775771
after move, [x=67.98261 y=-0.00329 orient=0.00045]
iteration 68
cross_track_err -0.003291239107944396 , steering_p 0.0006582478215888792 , steering_d -0.001480879339017587 , steering -0.0008226315174287078
after move, [x=68.98261 y=-0.00284 orient=0.00041]
iteration 69
cross_track_err -0.002840445411303045 , steering_p 0.000568089082260609 , steering_d -0.0013523810899240532 , steering -0.0007842920076634441
after move, [x=69.98261 y=-0.00243 orient=0.00037]
iteration 70
cross_track_err -0.002430783296001808 , steering_p 0.00048615665920036157 , steering_d -0.0012289863459037115 , steering -0.00074282968670335
after move, [x=70.98261 y=-0.00206 orient=0.00033]
iteration 71
cross_track_err -0.0020603357861386055 , steering_p 0.00041206715722772114 , steering_d -0.001111342529589607 , steering -0.0006992753723618858
after move, [x=71.98261 y=-0.00173 orient=0.00030]
iteration 72
cross_track_err -0.0017270297651405652 , steering_p 0.00034540595302811304 , steering_d -0.000999918062994121 , steering -0.000654512109966008
after move, [x=72.98261 y=-0.00143 orient=0.00027]
iteration 73
cross_track_err -0.001428687516714042 , steering_p 0.00028573750334280845 , steering_d -0.0008950267452795693 , steering -0.0006092892419367608
after move, [x=73.98261 y=-0.00116 orient=0.00024]
iteration 74
cross_track_err -0.0011630708771563788 , steering_p 0.00023261417543127575 , steering_d -0.00079684991867299 , steering -0.0005642357432417142
after move, [x=74.98261 y=-0.00093 orient=0.00021]
iteration 75
cross_track_err -0.0009279187025092398 , steering_p 0.00018558374050184798 , steering_d -0.0007054565239414169 , steering -0.0005198727834395689
after move, [x=75.98261 y=-0.00072 orient=0.00018]
iteration 76
cross_track_err -0.0007209783173278701 , steering_p 0.00014419566346557402 , steering_d -0.0006208211555441092 , steering -0.0004766254920785352
after move, [x=76.98261 y=-0.00054 orient=0.00016]
iteration 77
cross_track_err -0.0005400315731706306 , steering_p 0.00010800631463412612 , steering_d -0.0005428402324717185 , steering -0.00043483391783759236
after move, [x=77.98261 y=-0.00038 orient=0.00014]
iteration 78
cross_track_err -0.0003829161050809046 , steering_p 7.658322101618092e-05 , steering_d -0.00047134640426917796 , steering -0.00039476318325299705
after move, [x=78.98261 y=-0.00025 orient=0.00012]
iteration 79
cross_track_err -0.0002475423340204405 , steering_p 4.95084668040881e-05 , steering_d -0.00040612131318139234 , steering -0.00035661284637730425
after move, [x=79.98261 y=-0.00013 orient=0.00010]
iteration 80
cross_track_err -0.000131906722992172 , steering_p 2.6381344598434403e-05 , steering_d -0.0003469068330848055 , steering -0.0003205254884863711
after move, [x=80.98261 y=-0.00003 orient=0.00008]
iteration 81
cross_track_err -3.4101754936851115e-05 , steering_p 6.820350987370223e-06 , steering_d -0.0002934149041659627 , steering -0.00028659455317859244
after move, [x=81.98261 y=0.00005 orient=0.00007]
iteration 82
cross_track_err 4.7676938210100954e-05 , steering_p -9.53538764202019e-06 , steering_d -0.0002453360794408562 , steering -0.0002548714670828764
after move, [x=82.98261 y=0.00012 orient=0.00005]
iteration 83
cross_track_err 0.00011512590334580347 , steering_p -2.3025180669160693e-05 , steering_d -0.00020234689540710754 , steering -0.00022537207607626822
after move, [x=83.98261 y=0.00017 orient=0.00004]
iteration 84
cross_track_err 0.00016983129487527926 , steering_p -3.396625897505585e-05 , steering_d -0.00016411617458842739 , steering -0.00019808243356348324
after move, [x=84.98261 y=0.00021 orient=0.00003]
iteration 85
cross_track_err 0.0002132680824237814 , steering_p -4.265361648475628e-05 , steering_d -0.0001303103626455064 , steering -0.00017296397913026267
after move, [x=85.98261 y=0.00025 orient=0.00002]
iteration 86
cross_track_err 0.00024680074817194934 , steering_p -4.936014963438987e-05 , steering_d -0.00010059799724450387 , steering -0.00014995814687889373
after move, [x=86.98261 y=0.00027 orient=0.00002]
iteration 87
cross_track_err 0.0002716852148810788 , steering_p -5.433704297621576e-05 , steering_d -7.46534001273883e-05 , steering -0.00012899044310360404
after move, [x=87.98261 y=0.00029 orient=0.00001]
iteration 88
cross_track_err 0.00028907177419175287 , steering_p -5.7814354838350576e-05 , steering_d -5.215967793202229e-05 , steering -0.00010997403277037286
after move, [x=88.98261 y=0.00030 orient=0.00001]
iteration 89
cross_track_err 0.0003000088113121345 , steering_p -6.00017622624269e-05 , steering_d -3.2811111361144855e-05 , steering -9.281287362357176e-05
after move, [x=89.98261 y=0.00031 orient=0.00000]
iteration 90
cross_track_err 0.00030544714677202106 , steering_p -6.108942935440421e-05 , steering_d -1.6315006379659695e-05 , steering -7.740443573406391e-05
after move, [x=90.98261 y=0.00031 orient=6.28318]
iteration 91
cross_track_err 0.0003062448385374306 , steering_p -6.124896770748612e-05 , steering_d -2.3930752962285644e-06 , steering -6.364204300371468e-05
after move, [x=91.98261 y=0.00030 orient=6.28318]
iteration 92
cross_track_err 0.0003031723085085655 , steering_p -6.0634461701713105e-05 , steering_d 9.21759008659515e-06 , steering -5.1416871615117955e-05
after move, [x=92.98261 y=0.00030 orient=6.28318]
iteration 93
cross_track_err 0.00029691767632551643 , steering_p -5.938353526510329e-05 , steering_d 1.8763896549147297e-05 , steering -4.061963871595599e-05
after move, [x=93.98261 y=0.00029 orient=6.28317]
iteration 94
cross_track_err 0.00028809220055919175 , steering_p -5.7618440111838354e-05 , steering_d 2.647642729897403e-05 , steering -3.114201281286432e-05
after move, [x=94.98261 y=0.00028 orient=6.28317]
iteration 95
cross_track_err 0.00027723574285567034 , steering_p -5.544714857113407e-05 , steering_d 3.2569373110564234e-05 , steering -2.287777546056984e-05
after move, [x=95.98261 y=0.00026 orient=6.28317]
iteration 96
cross_track_err 0.0002648221845109405 , steering_p -5.29644369021881e-05 , steering_d 3.724067503418953e-05 , steering -1.572376186799857e-05
after move, [x=96.98261 y=0.00025 orient=6.28317]
iteration 97
cross_track_err 0.00025126473739327075 , steering_p -5.0252947478654154e-05 , steering_d 4.067234135300923e-05 , steering -9.580606125644921e-06
after move, [x=97.98261 y=0.00024 orient=6.28317]
iteration 98
cross_track_err 0.000236921102182384 , steering_p -4.7384220436476804e-05 , steering_d 4.303090563266028e-05 , steering -4.353314803816523e-06
after move, [x=98.98261 y=0.00022 orient=6.28317]
iteration 99
cross_track_err 0.00022209843666545952 , steering_p -4.441968733309191e-05 , steering_d 4.446799655077343e-05 , steering 4.830921768152007e-08
after move, [x=99.98261 y=0.00021 orient=6.28317]
'''




'''
sterring_drift = 10

(env) Ruis-MacBook-Pro-15:6_pd_controller ruiwang$ python main.py
start
[x=0.00000 y=1.00000 orient=0.00000]
iteration 0
cross_track_err 1 , steering_p -0.2 , steering_d -0.0 , steering -0.2
steering_drift 0.17453292519943295 steering2 -0.025467074800567063
after move, [x=1.00000 y=0.99936 orient=6.28191]
iteration 1
cross_track_err 0.9993631855367084 , steering_p -0.1998726371073417 , steering_d 0.001910443389874672 , steering -0.19796219371746704
steering_drift 0.17453292519943295 steering2 -0.02342926851803409
after move, [x=2.00000 y=0.99750 orient=6.28074]
iteration 2
cross_track_err 0.9975037187042517 , steering_p -0.19950074374085036 , steering_d 0.0055784004973702395 , steering -0.19392234324348012
steering_drift 0.17453292519943295 steering2 -0.019389418044047174
after move, [x=2.99999 y=0.99506 orient=6.27977]
iteration 3
cross_track_err 0.9950584142189047 , steering_p -0.19901168284378096 , steering_d 0.007335913456040899 , steering -0.19167576938774006
steering_drift 0.17453292519943295 steering2 -0.01714284418830711
after move, [x=3.99999 y=0.99164 orient=6.27891]
iteration 4
cross_track_err 0.9916435215225692 , steering_p -0.19832870430451385 , steering_d 0.010244678089006642 , steering -0.1880840262155072
steering_drift 0.17453292519943295 steering2 -0.013551101016074263
after move, [x=4.99998 y=0.98737 orient=6.27824]
iteration 5
cross_track_err 0.9873714090000856 , steering_p -0.19747428180001714 , steering_d 0.01281633756745082 , steering -0.18465794423256632
steering_drift 0.17453292519943295 steering2 -0.01012501903313337
after move, [x=5.99997 y=0.98242 orient=6.27773]
iteration 6
cross_track_err 0.9824217071660896 , steering_p -0.1964843414332179 , steering_d 0.014849105501988036 , steering -0.18163523593122988
steering_drift 0.17453292519943295 steering2 -0.007102310731796929
after move, [x=6.99995 y=0.97697 orient=6.27737]
iteration 7
cross_track_err 0.9769657439377777 , steering_p -0.19539314878755554 , steering_d 0.016367889684935633 , steering -0.1790252591026199
steering_drift 0.17453292519943295 steering2 -0.004492333903186957
after move, [x=7.99994 y=0.97115 orient=6.27715]
iteration 8
cross_track_err 0.971154664838825 , steering_p -0.194230932967765 , steering_d 0.017433237296858195 , steering -0.1767976956709068
steering_drift 0.17453292519943295 steering2 -0.0022647704714738592
after move, [x=8.99992 y=0.96512 orient=6.27704]
iteration 9
cross_track_err 0.9651189714747402 , steering_p -0.19302379429494806 , steering_d 0.018107080092254146 , steering -0.17491671420269392
steering_drift 0.17453292519943295 steering2 -0.0003837890032609703
after move, [x=9.99990 y=0.95897 orient=6.27702]
iteration 10
cross_track_err 0.9589700414950535 , steering_p -0.19179400829901072 , steering_d 0.018446789939060237 , steering -0.17334721835995048
steering_drift 0.17453292519943295 steering2 0.0011857068394824644
after move, [x=10.99988 y=0.95280 orient=6.27708]
iteration 11
cross_track_err 0.9528019224281683 , steering_p -0.19056038448563367 , steering_d 0.018504357200655575 , steering -0.1720560272849781
steering_drift 0.17453292519943295 steering2 0.002476897914454851
after move, [x=11.99986 y=0.94669 orient=6.27720]
iteration 12
cross_track_err 0.9466930876140582 , steering_p -0.18933861752281167 , steering_d 0.018326504442330194 , steering -0.17101211308048148
steering_drift 0.17453292519943295 steering2 0.003520812118951472
after move, [x=12.99984 y=0.94071 orient=6.27738]
iteration 13
cross_track_err 0.940708095684627 , steering_p -0.1881416191369254 , steering_d 0.017954975788293748 , steering -0.17018664334863165
steering_drift 0.17453292519943295 steering2 0.004346281850801298
after move, [x=13.99983 y=0.93490 orient=6.27759]
iteration 14
cross_track_err 0.9348991420274435 , steering_p -0.1869798284054887 , steering_d 0.01742686097155044 , steering -0.16955296743393827
steering_drift 0.17453292519943295 steering2 0.004979957765494675
after move, [x=14.99981 y=0.92931 orient=6.27784]
iteration 15
cross_track_err 0.9293075003000624 , steering_p -0.18586150006001248 , steering_d 0.016774925182143408 , steering -0.16908657487786907
steering_drift 0.17453292519943295 steering2 0.005446350321563875
after move, [x=15.99980 y=0.92396 orient=6.27811]
iteration 16
cross_track_err 0.9239648547974241 , steering_p -0.18479297095948483 , steering_d 0.016027936507914897 , steering -0.16876503445156993
steering_drift 0.17453292519943295 steering2 0.005767890747863019
after move, [x=16.99978 y=0.91889 orient=6.27840]
iteration 17
cross_track_err 0.9188945258116289 , steering_p -0.18377890516232578 , steering_d 0.015210986957385542 , steering -0.16856791820494024
steering_drift 0.17453292519943295 steering2 0.005965006994492705
after move, [x=17.99977 y=0.91411 orient=6.27870]
iteration 18
cross_track_err 0.9141125910611646 , steering_p -0.18282251821223294 , steering_d 0.014345804251392869 , steering -0.16847671396084007
steering_drift 0.17453292519943295 steering2 0.0060562112385928735
after move, [x=18.99976 y=0.90963 orient=6.27900]
iteration 19
cross_track_err 0.9096289069960314 , steering_p -0.1819257813992063 , steering_d 0.01345105219539966 , steering -0.16847472920380663
steering_drift 0.17453292519943295 steering2 0.00605819599562632
after move, [x=19.99975 y=0.90545 orient=6.27931]
iteration 20
cross_track_err 0.9054480343521378 , steering_p -0.18108960687042758 , steering_d 0.012542617931680655 , steering -0.16854698893874692
steering_drift 0.17453292519943295 steering2 0.005985936260686026
after move, [x=20.99975 y=0.90157 orient=6.27961]
iteration 21
cross_track_err 0.9015700727536012 , steering_p -0.18031401455072027 , steering_d 0.011633884795609784 , steering -0.1686801297551105
steering_drift 0.17453292519943295 steering2 0.005852795444322462
after move, [x=21.99974 y=0.89799 orient=6.27990]
iteration 22
cross_track_err 0.8979914094615858 , steering_p -0.1795982818923172 , steering_d 0.010735989876046181 , steering -0.168862292016271
steering_drift 0.17453292519943295 steering2 0.005670633183161938
after move, [x=22.99973 y=0.89471 orient=6.28018]
iteration 23
cross_track_err 0.8947053875584513 , steering_p -0.17894107751169028 , steering_d 0.009858065709403618 , steering -0.16908301180228666
steering_drift 0.17453292519943295 steering2 0.005449913397146289
after move, [x=23.99973 y=0.89170 orient=6.28046]
iteration 24
cross_track_err 0.8917028989510892 , steering_p -0.17834057979021786 , steering_d 0.009007465822086225 , steering -0.16933311396813164
steering_drift 0.17453292519943295 steering2 0.00519981123130131
after move, [x=24.99973 y=0.88897 orient=6.28072]
iteration 25
cross_track_err 0.8889729075912876 , steering_p -0.17779458151825753 , steering_d 0.008189974079405027 , steering -0.1696046074388525
steering_drift 0.17453292519943295 steering2 0.004928317760580447
after move, [x=25.99972 y=0.88650 orient=6.28096]
iteration 26
cross_track_err 0.8865029082567809 , steering_p -0.1773005816513562 , steering_d 0.007409998003519935 , steering -0.16989058364783627
steering_drift 0.17453292519943295 steering2 0.004642341551596679
after move, [x=26.99972 y=0.88428 orient=6.28119]
iteration 27
cross_track_err 0.8842793261261434 , steering_p -0.17685586522522867 , steering_d 0.006670746391912674 , steering -0.170185118833316
steering_drift 0.17453292519943295 steering2 0.004347806366116952
after move, [x=27.99972 y=0.88229 orient=6.28141]
iteration 28
cross_track_err 0.8822878622245607 , steering_p -0.17645757244491214 , steering_d 0.0059743917047480055 , steering -0.17048318074016414
steering_drift 0.17453292519943295 steering2 0.0040497444592688114
after move, [x=28.99972 y=0.88051 orient=6.28161]
iteration 29
cross_track_err 0.8805137896253658 , steering_p -0.17610275792507316 , steering_d 0.005322217797584794 , steering -0.17078054012748836
steering_drift 0.17453292519943295 steering2 0.003752385071944586
after move, [x=29.99972 y=0.87894 orient=6.28180]
iteration 30
cross_track_err 0.8789422050724376 , steering_p -0.17578844101448754 , steering_d 0.004714753658784487 , steering -0.17107368735570305
steering_drift 0.17453292519943295 steering2 0.0034592378437298954
after move, [x=30.99971 y=0.87756 orient=6.28197]
iteration 31
cross_track_err 0.8775582404485562 , steering_p -0.17551164808971126 , steering_d 0.004151893871644052 , steering -0.1713597542180672
steering_drift 0.17453292519943295 steering2 0.0031731709813657416
after move, [x=31.99971 y=0.87635 orient=6.28213]
iteration 32
cross_track_err 0.8763472382609669 , steering_p -0.1752694476521934 , steering_d 0.003633006562767882 , steering -0.17163644108942552
steering_drift 0.17453292519943295 steering2 0.0028964841100074246
after move, [x=32.99971 y=0.87529 orient=6.28228]
iteration 33
cross_track_err 0.8752948950531974 , steering_p -0.1750589790106395 , steering_d 0.0031570296233087713 , steering -0.17190194938733072
steering_drift 0.17453292519943295 steering2 0.0026309758121022264
after move, [x=33.99971 y=0.87439 orient=6.28241]
iteration 34
cross_track_err 0.8743873763862746 , steering_p -0.17487747527725495 , steering_d 0.002722556000768206 , steering -0.17215491927648674
steering_drift 0.17453292519943295 steering2 0.002378005922946208
after move, [x=34.99971 y=0.87361 orient=6.28253]
iteration 35
cross_track_err 0.8736114067667878 , steering_p -0.17472228135335757 , steering_d 0.002327908858460348 , steering -0.17239437249489722
steering_drift 0.17453292519943295 steering2 0.0021385527045357233
after move, [x=35.99971 y=0.87295 orient=6.28264]
iteration 36
cross_track_err 0.8729543376369806 , steering_p -0.17459086752739614 , steering_d 0.001971207389421692 , steering -0.17261966013797445
steering_drift 0.17453292519943295 steering2 0.0019132650614585023
after move, [x=36.99971 y=0.87240 orient=6.28273]
iteration 37
cross_track_err 0.8724041962858787 , steering_p -0.17448083925717575 , steering_d 0.0016504240533056214 , steering -0.17283041520387013
steering_drift 0.17453292519943295 steering2 0.0017025099955628176
after move, [x=37.99971 y=0.87195 orient=6.28282]
iteration 38
cross_track_err 0.8719497182924725 , steering_p -0.1743899436584945 , steering_d 0.0013634339802187645 , steering -0.17302650967827574
steering_drift 0.17453292519943295 steering2 0.0015064155211572117
after move, [x=38.99971 y=0.87158 orient=6.28289]
iteration 39
cross_track_err 0.8715803658738439 , steering_p -0.1743160731747688 , steering_d 0.0011080572558856971 , steering -0.1732080159188831
steering_drift 0.17453292519943295 steering2 0.0013249092805498464
after move, [x=39.99971 y=0.87129 orient=6.28296]
iteration 40
cross_track_err 0.871286334284087 , steering_p -0.17425726685681742 , steering_d 0.0008820947692708137 , steering -0.1733751720875466
steering_drift 0.17453292519943295 steering2 0.0011577531118863449
after move, [x=40.99971 y=0.87106 orient=6.28302]
iteration 41
cross_track_err 0.8710585481948532 , steering_p -0.17421170963897065 , steering_d 0.0006833582677013839 , steering -0.17352835137126926
steering_drift 0.17453292519943295 steering2 0.0010045738281636851
after move, [x=41.99971 y=0.87089 orient=6.28307]
iteration 42
cross_track_err 0.870888649785925 , steering_p -0.174177729957185 , steering_d 0.0005096952267846788 , steering -0.17366803473040032
steering_drift 0.17453292519943295 steering2 0.0008648904690326253
after move, [x=42.99971 y=0.87077 orient=6.28311]
iteration 43
cross_track_err 0.8707689800847697 , steering_p -0.17415379601695394 , steering_d 0.00035900910346586823 , steering -0.17379478691348807
steering_drift 0.17453292519943295 steering2 0.0007381382859448782
after move, [x=43.99971 y=0.87069 orient=6.28315]
iteration 44
cross_track_err 0.8706925549176375 , steering_p -0.1741385109835275 , steering_d 0.00022927550139639763 , steering -0.1739092354821311
steering_drift 0.17453292519943295 steering2 0.0006236897173018396
after move, [x=44.99971 y=0.87065 orient=6.28318]
iteration 45
cross_track_err 0.8706530366714414 , steering_p -0.1741306073342883 , steering_d 0.00011855473858823906 , steering -0.17401205259570007
steering_drift 0.17453292519943295 steering2 0.0005208726037328748
after move, [x=45.99971 y=0.87064 orient=0.00002]
iteration 46
cross_track_err 0.8706447029151438 , steering_p -0.17412894058302877 , steering_d 2.5001268892821393e-05 , steering -0.17410393931413595
steering_drift 0.17453292519943295 steering2 0.0004289858852969952
after move, [x=46.99971 y=0.87066 orient=0.00004]
iteration 47
cross_track_err 0.8706624127913876 , steering_p -0.1741324825582775 , steering_d -5.3129628731163336e-05 , steering -0.17418561218700868
steering_drift 0.17453292519943295 steering2 0.0003473130124242718
after move, [x=47.99971 y=0.87070 orient=0.00006]
iteration 48
cross_track_err 0.8707015719632027 , steering_p -0.17414031439264055 , steering_d -0.00011747751544555207 , steering -0.1742577919080861
steering_drift 0.17453292519943295 steering2 0.0002751332913468463
after move, [x=48.99971 y=0.87076 orient=0.00007]
iteration 49
cross_track_err 0.8707580967863173 , steering_p -0.17415161935726348 , steering_d -0.0001695744693437362 , steering -0.1743211938266072
steering_drift 0.17453292519943295 steering2 0.0002117313728257353
after move, [x=49.99971 y=0.87083 orient=0.00008]
iteration 50
cross_track_err 0.8708283782743186 , steering_p -0.17416567565486374 , steering_d -0.00021084446400376766 , steering -0.1743765201188675
steering_drift 0.17453292519943295 steering2 0.00015640508056544156
after move, [x=50.99971 y=0.87091 orient=0.00009]
iteration 51
cross_track_err 0.8709092463310891 , steering_p -0.17418184926621783 , steering_d -0.00024260417031163328 , steering -0.17442445343652946
steering_drift 0.17453292519943295 steering2 0.00010847176290348925
after move, [x=51.99971 y=0.87100 orient=0.00009]
iteration 52
cross_track_err 0.8709979346419235 , steering_p -0.17419958692838472 , steering_d -0.0002660649325032516 , steering -0.17446565186088797
steering_drift 0.17453292519943295 steering2 6.727333854497641e-05
after move, [x=52.99971 y=0.87109 orient=0.00010]
iteration 53
cross_track_err 0.8710920465409018 , steering_p -0.17421840930818036 , steering_d -0.00028233569693469907 , steering -0.17450074500511506
steering_drift 0.17453292519943295 steering2 3.218019431788788e-05
after move, [x=53.99971 y=0.87119 orient=0.00010]
iteration 54
cross_track_err 0.8711895221067969 , steering_p -0.1742379044213594 , steering_d -0.0002924266976853662 , steering -0.17453033111904476
steering_drift 0.17453292519943295 steering2 2.594080388190756e-06
after move, [x=54.99971 y=0.87129 orient=0.00010]
iteration 55
cross_track_err 0.8712886066824007 , steering_p -0.17425772133648015 , steering_d -0.0002972537268114417 , steering -0.1745549750632916
steering_drift 0.17453292519943295 steering2 -2.204986385864749e-05
after move, [x=55.99971 y=0.87139 orient=0.00010]
iteration 56
cross_track_err 0.8713878209620233 , steering_p -0.17427756419240467 , steering_d -0.0002976428388676844 , steering -0.17457520703127236
steering_drift 0.17453292519943295 steering2 -4.2281831839408035e-05
after move, [x=56.99971 y=0.87149 orient=0.00010]
iteration 57
cross_track_err 0.8714859327484581 , steering_p -0.17429718654969162 , steering_d -0.000294335359304454 , steering -0.17459152190899607
steering_drift 0.17453292519943295 steering2 -5.859670956312457e-05
after move, [x=57.99971 y=0.87158 orient=0.00009]
iteration 58
cross_track_err 0.8715819304433096 , steering_p -0.17431638608866193 , steering_d -0.0002879930845544054 , steering -0.17460437917321633
steering_drift 0.17453292519943295 steering2 -7.145397378338636e-05
after move, [x=58.99971 y=0.87167 orient=0.00009]
iteration 59
cross_track_err 0.8716749983026927 , steering_p -0.17433499966053856 , steering_d -0.00027920357814936736 , steering -0.17461420323868793
steering_drift 0.17453292519943295 steering2 -8.127803925497834e-05
after move, [x=59.99971 y=0.87176 orient=0.00009]
iteration 60
cross_track_err 0.8717644934633955 , steering_p -0.17435289869267911 , steering_d -0.0002684854821083382 , steering -0.17462138417478745
steering_drift 0.17453292519943295 steering2 -8.845897535450509e-05
after move, [x=60.99971 y=0.87185 orient=0.00008]
iteration 61
cross_track_err 0.8718499247221421 , steering_p -0.17436998494442843 , steering_d -0.0002562937762398132 , steering -0.17462627872066824
steering_drift 0.17453292519943295 steering2 -9.335352123529539e-05
after move, [x=61.99971 y=0.87193 orient=0.00008]
iteration 62
cross_track_err 0.8719309330321247 , steering_p -0.17438618660642494 , steering_d -0.00024302492994776603 , steering -0.1746292115363727
steering_drift 0.17453292519943295 steering2 -9.628633693975486e-05
after move, [x=62.99971 y=0.87201 orient=0.00007]
iteration 63
cross_track_err 0.8720072736660465 , steering_p -0.1744014547332093 , steering_d -0.00022902190176543602 , steering -0.17463047663497475
steering_drift 0.17453292519943295 steering2 -9.755143554179835e-05
after move, [x=63.99971 y=0.87208 orient=0.00007]
iteration 64
cross_track_err 0.8720787999831195 , steering_p -0.17441575999662393 , steering_d -0.00021457895121923531 , steering -0.17463033894784316
steering_drift 0.17453292519943295 steering2 -9.741374841021333e-05
after move, [x=64.99971 y=0.87215 orient=0.00006]
iteration 65
cross_track_err 0.8721454487284117 , steering_p -0.17442908974568236 , steering_d -0.0001999462358764914 , steering -0.17462903598155885
steering_drift 0.17453292519943295 steering2 -9.611078212590218e-05
after move, [x=65.99971 y=0.87221 orient=0.00006]
iteration 66
cross_track_err 0.8722072267862779 , steering_p -0.1744414453572556 , steering_d -0.00018533417359867244 , steering -0.17462677953085426
steering_drift 0.17453292519943295 steering2 -9.385433142131139e-05
after move, [x=66.99971 y=0.87226 orient=0.00005]
iteration 67
cross_track_err 0.8722641993050316 , steering_p -0.17445283986100635 , steering_d -0.0001709175562610854 , steering -0.17462375741726743
steering_drift 0.17453292519943295 steering2 -9.083221783448558e-05
after move, [x=67.99971 y=0.87232 orient=0.00005]
iteration 68
cross_track_err 0.8723164791072074 , steering_p -0.1744632958214415 , steering_d -0.00015683940652744255 , steering -0.17462013522796893
steering_drift 0.17453292519943295 steering2 -8.721002853598336e-05
after move, [x=68.99971 y=0.87236 orient=0.00004]
iteration 69
cross_track_err 0.8723642172984848 , steering_p -0.17447284345969696 , steering_d -0.00014321457383204006 , steering -0.174616058033529
steering_drift 0.17453292519943295 steering2 -8.313283409605576e-05
after move, [x=69.99971 y=0.87241 orient=0.00004]
iteration 70
cross_track_err 0.8724075949883288 , steering_p -0.17448151899766576 , steering_d -0.0001301330695319125 , steering -0.17461165206719767
steering_drift 0.17453292519943295 steering2 -7.872686776472237e-05
after move, [x=70.99971 y=0.87245 orient=0.00004]
iteration 71
cross_track_err 0.8724468160364619 , steering_p -0.1744893632072924 , steering_d -0.00011766314439942693 , steering -0.1746070263516918
steering_drift 0.17453292519943295 steering2 -7.410115225886527e-05
after move, [x=71.99971 y=0.87248 orient=0.00003]
iteration 72
cross_track_err 0.8724821007412014 , steering_p -0.1744964201482403 , steering_d -0.00010585411421837332 , steering -0.17460227426245867
steering_drift 0.17453292519943295 steering2 -6.934906302571986e-05
after move, [x=72.99971 y=0.87251 orient=0.00003]
iteration 73
cross_track_err 0.8725136803883232 , steering_p -0.17450273607766464 , steering_d -9.47389413655797e-05 , steering -0.17459747501903022
steering_drift 0.17453292519943295 steering2 -6.454981959727601e-05
after move, [x=73.99971 y=0.87254 orient=0.00002]
iteration 74
cross_track_err 0.8725417925822898 , steering_p -0.17450835851645796 , steering_d -8.433658189976878e-05 , steering -0.17459269509835773
steering_drift 0.17453292519943295 steering2 -5.9769898924783016e-05
after move, [x=74.99971 y=0.87257 orient=0.00002]
iteration 75
cross_track_err 0.8725666772852732 , steering_p -0.17451333545705464 , steering_d -7.4654108950023e-05 , steering -0.17458798956600466
steering_drift 0.17453292519943295 steering2 -5.506436657171099e-05
after move, [x=75.99971 y=0.87259 orient=0.00002]
iteration 76
cross_track_err 0.8725885734933075 , steering_p -0.1745177146986615 , steering_d -6.568862410316623e-05 , steering -0.17458340332276467
steering_drift 0.17453292519943295 steering2 -5.047812333172641e-05
after move, [x=76.99971 y=0.87261 orient=0.00002]
iteration 77
cross_track_err 0.8726077164830112 , steering_p -0.17452154329660224 , steering_d -5.742896911087314e-05 , steering -0.17457897226571312
steering_drift 0.17453292519943295 steering2 -4.60470662801693e-05
after move, [x=77.99971 y=0.87262 orient=0.00001]
iteration 78
cross_track_err 0.8726243355665464 , steering_p -0.17452486711330928 , steering_d -4.985725060568935e-05 , steering -0.17457472436391497
steering_drift 0.17453292519943295 steering2 -4.179916448202037e-05
after move, [x=78.99971 y=0.87264 orient=0.00001]
iteration 79
cross_track_err 0.8726386522967663 , steering_p -0.17452773045935327 , steering_d -4.29501906596963e-05 , steering -0.17457068065001297
steering_drift 0.17453292519943295 steering2 -3.775545058001817e-05
after move, [x=79.99971 y=0.87265 orient=0.00001]
iteration 80
cross_track_err 0.872650879068761 , steering_p -0.17453017581375221 , steering_d -3.668031598424992e-05 , steering -0.17456685612973646
steering_drift 0.17453292519943295 steering2 -3.393093030351624e-05
after move, [x=80.99971 y=0.87266 orient=0.00001]
iteration 81
cross_track_err 0.872661218068226 , steering_p -0.17453224361364522 , steering_d -3.101699839502814e-05 , steering -0.17456326061204025
steering_drift 0.17453292519943295 steering2 -3.033541260730188e-05
after move, [x=81.99971 y=0.87267 orient=0.00001]
iteration 82
cross_track_err 0.8726698605211752 , steering_p -0.17453397210423505 , steering_d -2.5927358847543935e-05 , steering -0.1745598994630826
steering_drift 0.17453292519943295 steering2 -2.6974263649642838e-05
after move, [x=82.99971 y=0.87268 orient=0.00001]
iteration 83
cross_track_err 0.8726769862034937 , steering_p -0.17453539724069875 , steering_d -2.1377046955262102e-05 , steering -0.174556774287654
steering_drift 0.17453292519943295 steering2 -2.3849088221061798e-05
after move, [x=83.99971 y=0.87268 orient=0.00000]
iteration 84
cross_track_err 0.8726827631726293 , steering_p -0.17453655263452586 , steering_d -1.7330907406853946e-05 , steering -0.17455388354193271
steering_drift 0.17453292519943295 steering2 -2.0958342499766136e-05
after move, [x=84.99971 y=0.87269 orient=0.00000]
iteration 85
cross_track_err 0.8726873476873537 , steering_p -0.17453746953747074 , steering_d -1.375354417321173e-05 , steering -0.17455122308164395
steering_drift 0.17453292519943295 steering2 -1.829788221099915e-05
after move, [x=85.99971 y=0.87269 orient=0.00000]
iteration 86
cross_track_err 0.8726908842849529 , steering_p -0.17453817685699058 , steering_d -1.0609792797655615e-05 , steering -0.17454878664978823
steering_drift 0.17453292519943295 steering2 -1.5861450355286744e-05
after move, [x=86.99971 y=0.87269 orient=0.00000]
iteration 87
cross_track_err 0.8726935059884415 , steering_p -0.17453870119768833 , steering_d -7.865110465909986e-06 , steering -0.17454656630815424
steering_drift 0.17453292519943295 steering2 -1.3641108721290651e-05
after move, [x=87.99971 y=0.87270 orient=0.00000]
iteration 88
cross_track_err 0.8726953346194123 , steering_p -0.17453906692388246 , steering_d -5.485892912271417e-06 , steering -0.17454455281679473
steering_drift 0.17453292519943295 steering2 -1.1627617361786857e-05
after move, [x=88.99971 y=0.87270 orient=0.00000]
iteration 89
cross_track_err 0.8726964811949469 , steering_p -0.1745392962389894 , steering_d -3.439726603882143e-06 , steering -0.17454273596559328
steering_drift 0.17453292519943295 steering2 -9.81076616032861e-06
after move, [x=89.99971 y=0.87270 orient=0.00000]
iteration 90
cross_track_err 0.8726970463896134 , steering_p -0.1745394092779227 , steering_d -1.6955839995391742e-06 , steering -0.17454110486192223
steering_drift 0.17453292519943295 steering2 -8.179662489282702e-06
after move, [x=90.99971 y=0.87270 orient=6.28318]
iteration 91
cross_track_err 0.872697121045972 , steering_p -0.1745394242091944 , steering_d -2.2396907561894608e-07 , steering -0.17453964817827003
steering_drift 0.17453292519943295 steering2 -6.722978837081506e-06
after move, [x=91.99971 y=0.87270 orient=6.28318]
iteration 92
cross_track_err 0.8726967867192055 , steering_p -0.1745393573438411 , steering_d 1.0029802994804271e-06 , steering -0.17453835436354162
steering_drift 0.17453292519943295 steering2 -5.429164108672335e-06
after move, [x=92.99971 y=0.87270 orient=6.28318]
iteration 93
cross_track_err 0.8726961162434974 , steering_p -0.1745392232486995 , steering_d 2.0114271241933324e-06 , steering -0.1745372118215753
steering_drift 0.17453292519943295 steering2 -4.286622142357643e-06
after move, [x=93.99971 y=0.87270 orient=6.28318]
iteration 94
cross_track_err 0.8726951743095841 , steering_p -0.17453903486191683 , steering_d 2.8258017399362956e-06 , steering -0.1745362090601769
steering_drift 0.17453292519943295 steering2 -3.2838607439411582e-06
after move, [x=94.99971 y=0.87269 orient=6.28318]
iteration 95
cross_track_err 0.8726940180445633 , steering_p -0.17453880360891266 , steering_d 3.4687950625222896e-06 , steering -0.17453533481385014
steering_drift 0.17453292519943295 steering2 -2.4096144171925626e-06
after move, [x=95.99971 y=0.87269 orient=6.28318]
iteration 96
cross_track_err 0.8726926975865055 , steering_p -0.1745385395173011 , steering_d 3.961374173355736e-06 , steering -0.17453457814312776
steering_drift 0.17453292519943295 steering2 -1.6529436948076182e-06
after move, [x=96.99971 y=0.87269 orient=6.28318]
iteration 97
cross_track_err 0.8726912566477272 , steering_p -0.17453825132954545 , steering_d 4.322816334889623e-06 , steering -0.17453392851321056
steering_drift 0.17453292519943295 steering2 -1.003313777614423e-06
after move, [x=97.99971 y=0.87269 orient=6.28318]
iteration 98
cross_track_err 0.8726897330617638 , steering_p -0.17453794661235278 , steering_d 4.57075789017658e-06 , steering -0.1745333758544626
steering_drift 0.17453292519943295 steering2 -4.506550296545786e-07
after move, [x=98.99971 y=0.87269 orient=6.28318]
iteration 99
cross_track_err 0.8726881593101119 , steering_p -0.1745376318620224 , steering_d 4.721254955653009e-06 , steering -0.17453291060706674
steering_drift 0.17453292519943295 steering2 1.459236620426907e-08
after move, [x=99.99971 y=0.87269 orient=6.28318]
'''