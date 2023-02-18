# -----------
# User Instructions
#
# Implement a P controller by running 100 iterations
# of robot motion. The steering angle should be set
# by the parameter tau so that:
#
# steering = -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
#
# where the integrated crosstrack error (int_CTE) is
# the sum of all the previous crosstrack errors.
# This term works to cancel out steering drift.
#
# Only modify code at the bottom! Look for the TODO.
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

robot = Robot()
robot.set(0, 1, 0)

# NOTE: add 10 degrees of drift
robot.set_steering_drift(10 / 180 * np.pi)


def run(robot, tau_p, tau_d, tau_i, n=100, speed=1.0):
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

    sum_cte = 0

    for i in range(n):
        print('iteration', i)

        cross_track_err = robot.y - 0

        steering_p = -tau_p * cross_track_err

        diff_cte = (cross_track_err - cross_track_err_t_minus_1) / time_interval
        steering_d = -tau_d * diff_cte

        sum_cte += cross_track_err
        steering_i = -tau_i * sum_cte

        steering = steering_p + steering_d + steering_i
        print('cross_track_err', cross_track_err, ', steering_p', steering_p, ', steering_d', steering_d, 'steering_i', steering_i, ', steering', steering)

        robot.move(steering, dist)
        print('after move,', robot)

        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)

        cross_track_err_t_minus_1 = cross_track_err

    return x_trajectory, y_trajectory


x_trajectory, y_trajectory = run(robot, 0.2, 3.0, 0.004, n = 200)

n = len(x_trajectory)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8,8))
ax1.plot(x_trajectory, y_trajectory, 'g', label='PID controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')

# TODO: your code here
ax1.legend()
plt.show()


'''
(env) Ruis-MacBook-Pro-15:11_pid_implementation ruiwang$ python main.py
start
[x=0.00000 y=1.00000 orient=0.00000]
iteration 0
cross_track_err 1 , steering_p -0.2 , steering_d -0.0 steering_i -0.004 , steering -0.20400000000000001
steering_drift 0.17453292519943295 steering2 -0.029467074800567067
after move, [x=1.00000 y=0.99926 orient=6.28171]
iteration 1
cross_track_err 0.9992631099684104 , steering_p -0.1998526219936821 , steering_d 0.0022106700947688296 steering_i -0.007997052439873642 , steering -0.2056390043387869
steering_drift 0.17453292519943295 steering2 -0.031106079139353954
after move, [x=2.00000 y=0.99701 orient=6.28016]
iteration 2
cross_track_err 0.997011428876931 , steering_p -0.1994022857753862 , steering_d 0.0067550432744383215 steering_i -0.011985098155381365 , steering -0.20463234065632926
steering_drift 0.17453292519943295 steering2 -0.030099415456896317
after move, [x=2.99999 y=0.99323 orient=6.27865]
iteration 3
cross_track_err 0.9932291394267168 , steering_p -0.19864582788534335 , steering_d 0.01134686835064258 steering_i -0.015958014713088234 , steering -0.203256974247789
steering_drift 0.17453292519943295 steering2 -0.028724049048356043
after move, [x=3.99998 y=0.98798 orient=6.27721]
iteration 4
cross_track_err 0.9879758537164207 , steering_p -0.19759517074328414 , steering_d 0.015759857130888122 steering_i -0.019909918127953917 , steering -0.20174523174034995
steering_drift 0.17453292519943295 steering2 -0.027212306540916997
after move, [x=4.99995 y=0.98132 orient=6.27585]
iteration 5
cross_track_err 0.9813238185399769 , steering_p -0.1962647637079954 , steering_d 0.01995610552933158 steering_i -0.023835213402113822 , steering -0.20014387158077762
steering_drift 0.17453292519943295 steering2 -0.025610946381344674
after move, [x=5.99992 y=0.97335 orient=6.27457]
iteration 6
cross_track_err 0.9733509294829901 , steering_p -0.194670185896598 , steering_d 0.02391866717096036 steering_i -0.027728617120045783 , steering -0.19848013584568344
steering_drift 0.17453292519943295 steering2 -0.023947210646250494
after move, [x=6.99988 y=0.96414 orient=6.27337]
iteration 7
cross_track_err 0.9641388778377404 , steering_p -0.1928277755675481 , steering_d 0.02763615493574889 steering_i -0.03158517263139674 , steering -0.19677679326319594
steering_drift 0.17453292519943295 steering2 -0.022243868063762995
after move, [x=7.99983 y=0.95377 orient=6.27226]
iteration 8
cross_track_err 0.9537718984149706 , steering_p -0.19075437968299414 , steering_d 0.03110093826830962 steering_i -0.03540026022505663 , steering -0.19505370163974114
steering_drift 0.17453292519943295 steering2 -0.020520776440308197
after move, [x=8.99976 y=0.94234 orient=6.27124]
iteration 9
cross_track_err 0.9423357026804524 , steering_p -0.18846714053609048 , steering_d 0.03430858720355445 steering_i -0.03916960303577844 , steering -0.19332815636831446
steering_drift 0.17453292519943295 steering2 -0.018795231168881515
after move, [x=9.99969 y=0.93039 orient=6.27030]
iteration 10
cross_track_err 0.9303864500914759 , steering_p -0.1860772900182952 , steering_d 0.0358477577669295 steering_i -0.04289114883614434 , steering -0.19312068108751004
steering_drift 0.17453292519943295 steering2 -0.018587755888077095
after move, [x=10.99961 y=0.91750 orient=6.26937]
iteration 11
cross_track_err 0.9174973977862989 , steering_p -0.1834994795572598 , steering_d 0.03866715691553102 steering_i -0.04656113842728954 , steering -0.1913934610690183
steering_drift 0.17453292519943295 steering2 -0.016860535869585358
after move, [x=11.99951 y=0.90368 orient=6.26852]
iteration 12
cross_track_err 0.9036789335482424 , steering_p -0.1807357867096485 , steering_d 0.04145539271416965 steering_i -0.05017585416148251 , steering -0.18945624815696135
steering_drift 0.17453292519943295 steering2 -0.014923322957528407
after move, [x=12.99940 y=0.88902 orient=6.26778]
iteration 13
cross_track_err 0.8890174481336481 , steering_p -0.17780348962672965 , steering_d 0.043984456243782666 steering_i -0.0537319239540171 , steering -0.18755095733696409
steering_drift 0.17453292519943295 steering2 -0.013018032137531138
after move, [x=13.99928 y=0.87361 orient=6.26713]
iteration 14
cross_track_err 0.8736098255336909 , steering_p -0.1747219651067382 , steering_d 0.046222867799871614 steering_i -0.057226363256151866 , steering -0.18572546056301847
steering_drift 0.17453292519943295 steering2 -0.011192535363585526
after move, [x=14.99916 y=0.85755 orient=6.26657]
iteration 15
cross_track_err 0.8575513451346342 , steering_p -0.17151026902692684 , steering_d 0.04817544119717032 steering_i -0.0606565686366904 , steering -0.1839913964664469
steering_drift 0.17453292519943295 steering2 -0.009458471267013963
after move, [x=15.99902 y=0.84093 orient=6.26609]
iteration 16
cross_track_err 0.8409332893060673 , steering_p -0.16818665786121345 , steering_d 0.04985416748570071 steering_i -0.06402030179391467 , steering -0.1823527921694274
steering_drift 0.17453292519943295 steering2 -0.007819866969994455
after move, [x=16.99887 y=0.82384 orient=6.26570]
iteration 17
cross_track_err 0.8238423629944407 , steering_p -0.16476847259888816 , steering_d 0.05127277893487969 steering_i -0.06731567124589244 , steering -0.1808113649099009
steering_drift 0.17453292519943295 steering2 -0.006278439710467959
after move, [x=17.99872 y=0.80636 orient=6.26539]
iteration 18
cross_track_err 0.8063604937906056 , steering_p -0.16127209875812112 , steering_d 0.052445607611505296 steering_i -0.07054111322105486 , steering -0.17936760436767069
steering_drift 0.17453292519943295 steering2 -0.004834679168237738
after move, [x=18.99856 y=0.78856 orient=6.26515]
iteration 19
cross_track_err 0.7885647473169908 , steering_p -0.15771294946339817 , steering_d 0.053387239420844446 steering_i -0.07369537221032282 , steering -0.17802108225287655
steering_drift 0.17453292519943295 steering2 -0.0034881570534436024
after move, [x=19.99840 y=0.77053 orient=6.26497]
iteration 20
cross_track_err 0.7705273038043416 , steering_p -0.15410546076086834 , steering_d 0.054112330537947395 steering_i -0.0767774814255402 , steering -0.17677061164846114
steering_drift 0.17453292519943295 steering2 -0.0022376864490281922
after move, [x=20.99823 y=0.75232 orient=6.26486]
iteration 21
cross_track_err 0.752315480381047 , steering_p -0.15046309607620942 , steering_d 0.05463547026988391 steering_i -0.07978674334706437 , steering -0.17561436915338988
steering_drift 0.17453292519943295 steering2 -0.0010814439539569287
after move, [x=21.99806 y=0.73399 orient=6.26481]
iteration 22
cross_track_err 0.7339917911187137 , steering_p -0.14679835822374274 , steering_d 0.054971067786999894 steering_i -0.08272271051153923 , steering -0.17455000094828207
steering_drift 0.17453292519943295 steering2 -1.707574884912688e-05
after move, [x=22.99789 y=0.71561 orient=6.26481]
iteration 23
cross_track_err 0.715614038742758 , steering_p -0.1431228077485516 , steering_d 0.05513325712786721 steering_i -0.08558516666651027 , steering -0.17357471728719465
steering_drift 0.17453292519943295 steering2 0.0009582079122383014
after move, [x=23.99773 y=0.69724 orient=6.26485]
iteration 24
cross_track_err 0.6972354327235585 , steering_p -0.1394470865447117 , steering_d 0.055135818057598485 steering_i -0.0883741083974045 , steering -0.1726853768845177
steering_drift 0.17453292519943295 steering2 0.0018475483149152372
after move, [x=24.99756 y=0.67890 orient=6.26495]
iteration 25
cross_track_err 0.6789047290436007 , steering_p -0.13578094580872013 , steering_d 0.05499211103987345 steering_i -0.09108972731357891 , steering -0.17187856208242558
steering_drift 0.17453292519943295 steering2 0.002654363117007369
after move, [x=25.99739 y=0.66067 orient=6.26508]
iteration 26
cross_track_err 0.6606663874411728 , steering_p -0.13213327748823456 , steering_d 0.05471502480728363 steering_i -0.09373239286334359 , steering -0.17115064554429452
steering_drift 0.17453292519943295 steering2 0.0033822796551384315
after move, [x=26.99723 y=0.64256 orient=6.26525]
iteration 27
cross_track_err 0.6425607423911495 , steering_p -0.1285121484782299 , steering_d 0.05431693515006997 steering_i -0.0963026358329082 , steering -0.17049784916106814
steering_drift 0.17453292519943295 steering2 0.00403507603836481
after move, [x=27.99707 y=0.62462 orient=6.26545]
iteration 28
cross_track_err 0.6246241845054907 , steering_p -0.12492483690109815 , steering_d 0.0538096736569762 steering_i -0.09880113257093015 , steering -0.1699162958150521
steering_drift 0.17453292519943295 steering2 0.004616629384380844
after move, [x=28.99691 y=0.60689 orient=6.26568]
iteration 29
cross_track_err 0.6068893494235025 , steering_p -0.1213778698847005 , steering_d 0.05320450524596476 steering_i -0.10122868996862416 , steering -0.1694020546073599
steering_drift 0.17453292519943295 steering2 0.005130870592073039
after move, [x=29.99676 y=0.58939 orient=6.26594]
iteration 30
cross_track_err 0.5893853116169377 , steering_p -0.11787706232338756 , steering_d 0.052512113419694195 steering_i -0.10358623121509192 , steering -0.1689511801187853
steering_drift 0.17453292519943295 steering2 0.005581745080647654
after move, [x=30.99661 y=0.57214 orient=6.26622]
iteration 31
cross_track_err 0.5721377808597342 , steering_p -0.11442755617194685 , steering_d 0.0517425922716106 steering_i -0.10587478233853086 , steering -0.1685597462388671
steering_drift 0.17453292519943295 steering2 0.005973178960565845
after move, [x=31.99646 y=0.55517 orient=6.26651]
iteration 32
cross_track_err 0.5551692994085131 , steering_p -0.11103385988170263 , steering_d 0.05090544435366329 steering_i -0.1080954595361649 , steering -0.16822387506420422
steering_drift 0.17453292519943295 steering2 0.006309050135228728
after move, [x=32.99632 y=0.53850 orient=6.26683]
iteration 33
cross_track_err 0.5384994382097102 , steering_p -0.10769988764194205 , steering_d 0.05000958359640861 steering_i -0.11024945728900375 , steering -0.1679397613345372
steering_drift 0.17453292519943295 steering2 0.006593163864895751
after move, [x=33.99619 y=0.52214 orient=6.26716]
iteration 34
cross_track_err 0.5221449906941014 , steering_p -0.10442899813882028 , steering_d 0.04906334254682665 steering_i -0.11233803725178015 , steering -0.1677036928437738
steering_drift 0.17453292519943295 steering2 0.006829232355659159
after move, [x=34.99606 y=0.50612 orient=6.26750]
iteration 35
cross_track_err 0.5061201629411692 , steering_p -0.10122403258823386 , steering_d 0.048074483258796374 steering_i -0.11436251790354483 , steering -0.16751206723298231
steering_drift 0.17453292519943295 steering2 0.007020857966450633
after move, [x=35.99594 y=0.49044 orient=6.26785]
iteration 36
cross_track_err 0.4904367591958054 , steering_p -0.09808735183916109 , steering_d 0.04705021123609143 steering_i -0.11632426494032805 , steering -0.1673614055433977
steering_drift 0.17453292519943295 steering2 0.007171519656035241
after move, [x=36.99582 y=0.47510 orient=6.26821]
iteration 37
cross_track_err 0.4751043618997363 , steering_p -0.09502087237994727 , steering_d 0.045997191888207334 steering_i -0.118224682387927 , steering -0.16724836287966693
steering_drift 0.17453292519943295 steering2 0.007284562319766014
after move, [x=37.99571 y=0.46013 orient=6.26858]
iteration 38
cross_track_err 0.4601305055612583 , steering_p -0.09202610111225167 , steering_d 0.04492156901543409 steering_i -0.12006520441017203 , steering -0.1671697365069896
steering_drift 0.17453292519943295 steering2 0.007363188692443345
after move, [x=38.99560 y=0.44552 orient=6.26894]
iteration 39
cross_track_err 0.44552084393072466 , steering_p -0.08910416878614494 , steering_d 0.043828984891600864 steering_i -0.12184728778589493 , steering -0.167122471680439
steering_drift 0.17453292519943295 steering2 0.0074104535189939424
after move, [x=39.99550 y=0.43128 orient=6.26931]
iteration 40
cross_track_err 0.4312793100770388 , steering_p -0.08625586201540776 , steering_d 0.04272460156105762 steering_i -0.12357240502620309 , steering -0.16710366548055322
steering_drift 0.17453292519943295 steering2 0.007429259718879727
after move, [x=40.99541 y=0.41741 orient=6.26969]
iteration 41
cross_track_err 0.4174082690734581 , steering_p -0.08348165381469164 , steering_d 0.04161312301074199 steering_i -0.12524203810249693 , steering -0.16711056890644657
steering_drift 0.17453292519943295 steering2 0.007422356292986376
after move, [x=41.99531 y=0.40391 orient=6.27006]
iteration 42
cross_track_err 0.4039086631004359 , steering_p -0.08078173262008718 , steering_d 0.04049881791906673 steering_i -0.12685767275489865 , steering -0.1671405874559191
steering_drift 0.17453292519943295 steering2 0.00739233774351386
after move, [x=42.99523 y=0.39078 orient=6.27043]
iteration 43
cross_track_err 0.3907801488602001 , steering_p -0.07815602977204003 , steering_d 0.03938554272070727 steering_i -0.12842079335033946 , steering -0.16719128040167222
steering_drift 0.17453292519943295 steering2 0.007341644797760727
after move, [x=43.99515 y=0.37802 orient=6.27079]
iteration 44
cross_track_err 0.37802122727332127 , steering_p -0.07560424545466427 , steering_d 0.03827676476063657 steering_i -0.12993287825943275 , steering -0.16726035895346045
steering_drift 0.17453292519943295 steering2 0.0072725662459725005
after move, [x=44.99507 y=0.36563 orient=6.27116]
iteration 45
cross_track_err 0.3656293654926751 , steering_p -0.07312587309853502 , steering_d 0.03717558534193849 steering_i -0.13139539572140344 , steering -0.16734568347799997
steering_drift 0.17453292519943295 steering2 0.00718724172143298
after move, [x=45.99500 y=0.35360 orient=6.27152]
iteration 46
cross_track_err 0.3536011113259071 , steering_p -0.07072022226518142 , steering_d 0.03608476250030401 steering_i -0.13280980016670707 , steering -0.1674452599315845
steering_drift 0.17453292519943295 steering2 0.007087665267848459
after move, [x=46.99493 y=0.34193 orient=6.27187]
iteration 47
cross_track_err 0.3419322002046476 , steering_p -0.06838644004092952 , steering_d 0.03500673336377852 steering_i -0.13417752896752566 , steering -0.16755723564467667
steering_drift 0.17453292519943295 steering2 0.006975689554756276
after move, [x=47.99487 y=0.33062 orient=6.27222]
iteration 48
cross_track_err 0.3306176548781336 , steering_p -0.06612353097562672 , steering_d 0.033943635979541975 steering_i -0.1354999995870382 , steering -0.16767989458312293
steering_drift 0.17453292519943295 steering2 0.006853030616310013
after move, [x=48.99480 y=0.31965 orient=6.27256]
iteration 49
cross_track_err 0.31965187804135614 , steering_p -0.06393037560827124 , steering_d 0.03289733051033239 steering_i -0.13677860709920361 , steering -0.16781165219714245
steering_drift 0.17453292519943295 steering2 0.006721273002290501
after move, [x=49.99475 y=0.30903 orient=6.27290]
iteration 50
cross_track_err 0.30902873813408743 , steering_p -0.06180574762681749 , steering_d 0.031869419721806125 steering_i -0.13801472205173998 , steering -0.16795104995675134
steering_drift 0.17453292519943295 steering2 0.006581875242681612
after move, [x=50.99470 y=0.29874 orient=6.27323]
iteration 51
cross_track_err 0.298741648567818 , steering_p -0.0597483297135636 , steering_d 0.030861268698808342 steering_i -0.13920968864601124 , steering -0.1680967496607665
steering_drift 0.17453292519943295 steering2 0.006436175538666461
after move, [x=51.99465 y=0.28878 orient=6.27355]
iteration 52
cross_track_err 0.2887836406533769 , steering_p -0.05775672813067539 , steering_d 0.029874023743323208 steering_i -0.14036482320862476 , steering -0.16824752759597694
steering_drift 0.17453292519943295 steering2 0.0062853976034560055
after move, [x=52.99460 y=0.27915 orient=6.27386]
iteration 53
cross_track_err 0.2791474305134026 , steering_p -0.055829486102680514 , steering_d 0.02890863041992303 steering_i -0.14148141293067837 , steering -0.16840226861343585
steering_drift 0.17453292519943295 steering2 0.006130656585997096
after move, [x=53.99456 y=0.26983 orient=6.27417]
iteration 54
cross_track_err 0.2698254802713687 , steering_p -0.05396509605427374 , steering_d 0.027965850726101582 steering_i -0.14256071485176386 , steering -0.16855996017993602
steering_drift 0.17453292519943295 steering2 0.005972965019496929
after move, [x=54.99452 y=0.26081 orient=6.27447]
iteration 55
cross_track_err 0.26081005381308286 , steering_p -0.052162010762616576 , steering_d 0.02704627937485754 steering_i -0.1436039550670162 , steering -0.16871968645477525
steering_drift 0.17453292519943295 steering2 0.005813238744657695
after move, [x=55.99448 y=0.25209 orient=6.27476]
iteration 56
cross_track_err 0.25209326741785076 , steering_p -0.05041865348357016 , steering_d 0.026150359185696304 steering_i -0.1446123281366876 , steering -0.16888062243456145
steering_drift 0.17453292519943295 steering2 0.005652302764871497
after move, [x=56.99444 y=0.24367 orient=6.27504]
iteration 57
cross_track_err 0.2436671355552879 , steering_p -0.04873342711105758 , steering_d 0.025278395587688574 steering_i -0.14558699667890873 , steering -0.16904202820227773
steering_drift 0.17453292519943295 steering2 0.005490896997155215
after move, [x=57.99441 y=0.23552 orient=6.27532]
iteration 58
cross_track_err 0.235523612140388 , steering_p -0.0471047224280776 , steering_d 0.024430570244699717 steering_i -0.14652909112747028 , steering -0.16920324331084818
steering_drift 0.17453292519943295 steering2 0.005329681888584764
after move, [x=58.99438 y=0.22765 orient=6.27558]
iteration 59
cross_track_err 0.2276546275342818 , steering_p -0.04553092550685636 , steering_d 0.023606953818318566 steering_i -0.1474397096376074 , steering -0.1693636813261452
steering_drift 0.17453292519943295 steering2 0.005169243873287738
after move, [x=59.99435 y=0.22005 orient=6.27584]
iteration 60
cross_track_err 0.22005212157141762 , steering_p -0.04401042431428353 , steering_d 0.02280751788859256 steering_i -0.1483199181238931 , steering -0.1695228245495841
steering_drift 0.17453292519943295 steering2 0.005010100649848859
after move, [x=60.99432 y=0.21271 orient=6.27609]
iteration 61
cross_track_err 0.2127080728859476 , steering_p -0.042541614577189524 , steering_d 0.0220321460564101 steering_i -0.1491707504154369 , steering -0.16968021893621632
steering_drift 0.17453292519943295 steering2 0.004852706263216627
after move, [x=61.99430 y=0.20561 orient=6.27633]
iteration 62
cross_track_err 0.20561452480114037 , steering_p -0.04112290496022808 , steering_d 0.021280644254421666 steering_i -0.14999320851464146 , steering -0.16983546922044787
steering_drift 0.17453292519943295 steering2 0.004697455978985077
after move, [x=62.99427 y=0.19876 orient=6.27657]
iteration 63
cross_track_err 0.19876360803588872 , steering_p -0.03975272160717774 , steering_d 0.020552750295754957 steering_i -0.15078826294678502 , steering -0.1699882342582078
steering_drift 0.17453292519943295 steering2 0.0045446909412251435
after move, [x=63.99425 y=0.19215 orient=6.27680]
iteration 64
cross_track_err 0.19214756047199677 , steering_p -0.038429512094399355 , steering_d 0.01984814269167584 steering_i -0.15155685318867299 , steering -0.1701382225913965
steering_drift 0.17453292519943295 steering2 0.004394702608036455
after move, [x=64.99423 y=0.18576 orient=6.27702]
iteration 65
cross_track_err 0.18575874421513427 , steering_p -0.037151748843026855 , steering_d 0.019166448770587485 steering_i -0.15229988816553353 , steering -0.1702851882379729
steering_drift 0.17453292519943295 steering2 0.004247736961460052
after move, [x=65.99421 y=0.17959 orient=6.27723]
iteration 66
cross_track_err 0.1795896601712312 , steering_p -0.035917932034246244 , steering_d 0.0185072521317092 steering_i -0.15301824680621845 , steering -0.17042892670875548
steering_drift 0.17453292519943295 steering2 0.0041039984906774685
after move, [x=66.99419 y=0.17363 orient=6.27743]
iteration 67
cross_track_err 0.17363296034880807 , steering_p -0.034726592069761614 , steering_d 0.017870099467269418 steering_i -0.1537127786476137 , steering -0.1705692712501059
steering_drift 0.17453292519943295 steering2 0.003963653949327045
after move, [x=67.99418 y=0.16788 orient=6.27763]
iteration 68
cross_track_err 0.16788145808641247 , steering_p -0.0335762916172825 , steering_d 0.01725450678718679 steering_i -0.15438430447995935 , steering -0.17070608931005504
steering_drift 0.17453292519943295 steering2 0.0038268358893779053
after move, [x=68.99416 y=0.16233 orient=6.27782]
iteration 69
cross_track_err 0.1623281363930315 , steering_p -0.0324656272786063 , steering_d 0.01665996508014289 steering_i -0.15503361702553148 , steering -0.1708392792239949
steering_drift 0.17453292519943295 steering2 0.003693645975438059
after move, [x=69.99415 y=0.15697 orient=6.27801]
iteration 70
cross_track_err 0.156966154578194 , steering_p -0.0313932309156388 , steering_d 0.016085945444512556 steering_i -0.15566148164384425 , steering -0.1709687671149705
steering_drift 0.17453292519943295 steering2 0.0035641580844624488
after move, [x=70.99413 y=0.15179 orient=6.27819]
iteration 71
cross_track_err 0.1517888533374833 , steering_p -0.03035777066749666 , steering_d 0.01553190372213209 steering_i -0.15626863705719418 , steering -0.17109450400255877
steering_drift 0.17453292519943295 steering2 0.0034384211968741807
after move, [x=71.99412 y=0.14679 orient=6.27836]
iteration 72
cross_track_err 0.14678975844846373 , steering_p -0.029357951689692748 , steering_d 0.014997284667058686 steering_i -0.15685579609098804 , steering -0.1712164631136221
steering_drift 0.17453292519943295 steering2 0.0033164620858108507
after move, [x=72.99411 y=0.14196 orient=6.27852]
iteration 73
cross_track_err 0.141962583221592 , steering_p -0.0283925166443184 , steering_d 0.014481525680615198 steering_i -0.1574236464238744 , steering -0.17133463738757762
steering_drift 0.17453292519943295 steering2 0.003198287811855327
after move, [x=73.99410 y=0.13730 orient=6.27868]
iteration 74
cross_track_err 0.13730122984058735 , steering_p -0.02746024596811747 , steering_d 0.013984060143013954 steering_i -0.15797285134323677 , steering -0.17144903716834028
steering_drift 0.17453292519943295 steering2 0.003083888031092663
after move, [x=74.99409 y=0.13280 orient=6.27884]
iteration 75
cross_track_err 0.13279978971701292 , steering_p -0.026559957943402587 , steering_d 0.013504320370723272 steering_i -0.1585040505021048 , steering -0.1715596880747841
steering_drift 0.17453292519943295 steering2 0.0029732371246488365
after move, [x=75.99408 y=0.12845 orient=6.27899]
iteration 76
cross_track_err 0.12845254297448175 , steering_p -0.02569050859489635 , steering_d 0.013041740227593523 steering_i -0.15901786067400273 , steering -0.17166662904130556
steering_drift 0.17453292519943295 steering2 0.0028662961581273905
after move, [x=76.99407 y=0.12425 orient=6.27913]
iteration 77
cross_track_err 0.12425395716898115 , steering_p -0.024850791433796232 , steering_d 0.012595757416501804 steering_i -0.15951487650267865 , steering -0.1717699105199731
steering_drift 0.17453292519943295 steering2 0.0027630146794598576
after move, [x=77.99406 y=0.12020 orient=6.27927]
iteration 78
cross_track_err 0.12019868534329599 , steering_p -0.0240397370686592 , steering_d 0.012165815477055475 steering_i -0.15999567124405184 , steering -0.17186959283565556
steering_drift 0.17453292519943295 steering2 0.0026633323637773876
after move, [x=78.99406 y=0.11628 orient=6.27940]
iteration 79
cross_track_err 0.11628156350543561 , steering_p -0.023256312701087124 , steering_d 0.011751365513581122 steering_i -0.1604607974980736 , steering -0.1719657446855796
steering_drift 0.17453292519943295 steering2 0.002567180513853351
after move, [x=79.99405 y=0.11250 orient=6.27953]
iteration 80
cross_track_err 0.11249760761331834 , steering_p -0.02249952152266367 , steering_d 0.011351867676351823 steering_i -0.16091078792852687 , steering -0.1720584417748387
steering_drift 0.17453292519943295 steering2 0.002474483424594248
after move, [x=80.99404 y=0.10884 orient=6.27965]
iteration 81
cross_track_err 0.10884201014074629 , steering_p -0.02176840202814926 , steering_d 0.010966792417716142 steering_i -0.16134615596908986 , steering -0.17214776557952297
steering_drift 0.17453292519943295 steering2 0.0023851596199099823
after move, [x=81.99404 y=0.10531 orient=6.27977]
iteration 82
cross_track_err 0.10531013629290044 , steering_p -0.02106202725858009 , steering_d 0.010595621543537562 steering_i -0.16176739651426147 , steering -0.172233802229304
steering_drift 0.17453292519943295 steering2 0.002299122970128936
after move, [x=82.99403 y=0.10190 orient=6.27989]
iteration 83
cross_track_err 0.10189751993321304 , steering_p -0.020379503986642608 , steering_d 0.010237849079062195 steering_i -0.16217498659399432 , steering -0.17231664150157472
steering_drift 0.17453292519943295 steering2 0.002216283697858229
after move, [x=83.99402 y=0.09860 orient=6.28000]
iteration 84
cross_track_err 0.09859985927749006 , steering_p -0.019719971855498012 , steering_d 0.009892981967168951 steering_i -0.16256938603110427 , steering -0.17239637591943333
steering_drift 0.17453292519943295 steering2 0.002136549279999622
after move, [x=84.99402 y=0.09541 orient=6.28011]
iteration 85
cross_track_err 0.09541301240558712 , steering_p -0.019082602481117428 , steering_d 0.009560540615708796 steering_i -0.1629510380807266 , steering -0.17247309994613524
steering_drift 0.17453292519943295 steering2 0.0020598252532977035
after move, [x=85.99401 y=0.09233 orient=6.28021]
iteration 86
cross_track_err 0.09233299263574381 , steering_p -0.018466598527148763 , steering_d 0.009240059309529952 steering_i -0.16332037005126962 , steering -0.17254690926888844
steering_drift 0.17453292519943295 steering2 0.00198601593054451
after move, [x=86.99401 y=0.08936 orient=6.28031]
iteration 87
cross_track_err 0.08935596380186264 , steering_p -0.01787119276037253 , steering_d 0.008931086501643495 steering_i -0.16367779390647708 , steering -0.17261790016520612
steering_drift 0.17453292519943295 steering2 0.0019150250342268305
after move, [x=87.99401 y=0.08648 orient=6.28040]
iteration 88
cross_track_err 0.08647823546954163 , steering_p -0.017295647093908326 , steering_d 0.008633184996963042 steering_i -0.16402370684835524 , steering -0.17268616894530053
steering_drift 0.17453292519943295 steering2 0.0018467562541324145
after move, [x=88.99400 y=0.08370 orient=6.28050]
iteration 89
cross_track_err 0.08369625812255341 , steering_p -0.016739251624510683 , steering_d 0.008345932040964663 steering_i -0.16435849188084545 , steering -0.17275181146439147
steering_drift 0.17453292519943295 steering2 0.001781113735041473
after move, [x=89.99400 y=0.08101 orient=6.28058]
iteration 90
cross_track_err 0.08100661834765335 , steering_p -0.01620132366953067 , steering_d 0.008068919324700163 steering_i -0.16468251835423603 , steering -0.17281492269906654
steering_drift 0.17453292519943295 steering2 0.0017180025003664112
after move, [x=90.99399 y=0.07841 orient=6.28067]
iteration 91
cross_track_err 0.07840603404210347 , steering_p -0.015681206808420695 , steering_d 0.00780175291664964 steering_i -0.16499614249040445 , steering -0.1728755963821755
steering_drift 0.17453292519943295 steering2 0.0016573288172574419
after move, [x=91.99399 y=0.07589 orient=6.28075]
iteration 92
cross_track_err 0.07589134966509925 , steering_p -0.01517826993301985 , steering_d 0.007544053131012668 steering_i -0.16529970788906484 , steering -0.17293392469107202
steering_drift 0.17453292519943295 steering2 0.001599000508360926
after move, [x=92.99399 y=0.07346 orient=6.28083]
iteration 93
cross_track_err 0.0734595315513587 , steering_p -0.014691906310271742 , steering_d 0.007295454341221641 steering_i -0.1655935460152703 , steering -0.17298999798432038
steering_drift 0.17453292519943295 steering2 0.0015429272151125661
after move, [x=93.99399 y=0.07111 orient=6.28091]
iteration 94
cross_track_err 0.07110766330245984 , steering_p -0.014221532660491969 , steering_d 0.0070556047466966 steering_i -0.16587797666848014 , steering -0.1730439045822755
steering_drift 0.17453292519943295 steering2 0.0014890206171574438
after move, [x=94.99398 y=0.06883 orient=6.28099]
iteration 95
cross_track_err 0.06883294126909761 , steering_p -0.013766588253819524 , steering_d 0.006824166100086668 steering_i -0.16615330843355652 , steering -0.1730957305872894
steering_drift 0.17453292519943295 steering2 0.0014371946121435608
after move, [x=95.99398 y=0.06663 orient=6.28106]
iteration 96
cross_track_err 0.06663267013523445 , steering_p -0.013326534027046892 , steering_d 0.006600813401589481 steering_i -0.16641983911409747 , steering -0.17314555973955487
steering_drift 0.17453292519943295 steering2 0.0013873654598780816
after move, [x=96.99398 y=0.06450 orient=6.28113]
iteration 97
cross_track_err 0.06450425861313035 , steering_p -0.01290085172262607 , steering_d 0.00638523456631232 steering_i -0.16667785614855 , steering -0.17319347330486373
steering_drift 0.17453292519943295 steering2 0.0013394518945692158
after move, [x=97.99398 y=0.06245 orient=6.28119]
iteration 98
cross_track_err 0.06244521525646733 , steering_p -0.012489043051293468 , steering_d 0.006177130069989045 steering_i -0.16692763700957586 , steering -0.1732395499908803
steering_drift 0.17453292519943295 steering2 0.0012933752085526606
after move, [x=98.99397 y=0.06045 orient=6.28126]
iteration 99
cross_track_err 0.06045314439718232 , steering_p -0.012090628879436465 , steering_d 0.005976212577855029 steering_i -0.16716944958716456 , steering -0.173283865888746
steering_drift 0.17453292519943295 steering2 0.0012490593106869452
after move, [x=99.99397 y=0.05853 orient=6.28132]
iteration 100
cross_track_err 0.058525742210191085 , steering_p -0.011705148442038217 , steering_d 0.00578220656097371 steering_i -0.16740355255600534 , steering -0.17332649443706985
steering_drift 0.17453292519943295 steering2 0.0012064307623630977
after move, [x=100.99397 y=0.05666 orient=6.28138]
iteration 101
cross_track_err 0.05666079290892838 , steering_p -0.011332158581785677 , steering_d 0.005594847903788121 steering_i -0.16763019572764107 , steering -0.17336750640563864
steering_drift 0.17453292519943295 steering2 0.0011654187937943095
after move, [x=101.99397 y=0.05486 orient=6.28144]
iteration 102
cross_track_err 0.05485616507350548 , steering_p -0.010971233014701097 , steering_d 0.005413883506268702 steering_i -0.16784962038793508 , steering -0.17340696989636747
steering_drift 0.17453292519943295 steering2 0.0011259553030654734
after move, [x=102.99397 y=0.05311 orient=6.28150]
iteration 103
cross_track_err 0.053109808112299366 , steering_p -0.010621961622459875 , steering_d 0.005239070883618337 steering_i -0.16806205962038426 , steering -0.1734449503592258
steering_drift 0.17453292519943295 steering2 0.0010879748402071454
after move, [x=103.99397 y=0.05142 orient=6.28155]
iteration 104
cross_track_err 0.05141974885692794 , steering_p -0.010283949771385588 , steering_d 0.005070177766114278 steering_i -0.16826773861581196 , steering -0.17348151062108327
steering_drift 0.17453292519943295 steering2 0.0010514145783496742
after move, [x=104.99396 y=0.04978 orient=6.28160]
iteration 105
cross_track_err 0.0497840882898149 , steering_p -0.00995681765796298 , steering_d 0.004906981701339123 steering_i -0.16846687496897123 , steering -0.1735167109255951
steering_drift 0.17453292519943295 steering2 0.0010162142738378566
after move, [x=105.99396 y=0.04820 orient=6.28165]
iteration 106
cross_track_err 0.048200998402904 , steering_p -0.0096401996805808 , steering_d 0.004749269660732705 steering_i -0.16865967896258283 , steering -0.1735506089824309
steering_drift 0.17453292519943295 steering2 0.0009823162170020383
after move, [x=106.99396 y=0.04667 orient=6.28170]
iteration 107
cross_track_err 0.04666871918552701 , steering_p -0.009333743837105403 , steering_d 0.004596837652130967 steering_i -0.16884635383932495 , steering -0.17358326002429939
steering_drift 0.17453292519943295 steering2 0.0009496651751335627
after move, [x=107.99396 y=0.04519 orient=6.28175]
iteration 108
cross_track_err 0.04518555573896788 , steering_p -0.009037111147793576 , steering_d 0.004449490339677377 steering_i -0.1690270960622808 , steering -0.173614716870397
steering_drift 0.17453292519943295 steering2 0.0009182083290359411
after move, [x=108.99396 y=0.04375 orient=6.28180]
iteration 109
cross_track_err 0.0437498755148675 , steering_p -0.008749975102973501 , steering_d 0.0043070406723011365 steering_i -0.1692020955643403 , steering -0.17364502999501266
steering_drift 0.17453292519943295 steering2 0.0008878952044202859
after move, [x=109.99396 y=0.04236 orient=6.28184]
iteration 110
cross_track_err 0.042360105674303435 , steering_p -0.008472021134860687 , steering_d 0.004169309521692206 steering_i -0.1693715359870375 , steering -0.17367424760020597
steering_drift 0.17453292519943295 steering2 0.0008586775992269757
after move, [x=110.99396 y=0.04101 orient=6.28188]
iteration 111
cross_track_err 0.04101473056410805 , steering_p -0.00820294611282161 , steering_d 0.00403612533058615 steering_i -0.16953559490929393 , steering -0.1737024156915294
steering_drift 0.17453292519943295 steering2 0.0008305095079035596
after move, [x=111.99396 y=0.03971 orient=6.28192]
iteration 112
cross_track_err 0.03971228930679656 , steering_p -0.007942457861359312 , steering_d 0.003907323771934479 steering_i -0.16969444406652112 , steering -0.17372957815594595
steering_drift 0.17453292519943295 steering2 0.0008033470434870016
after move, [x=112.99396 y=0.03845 orient=6.28196]
iteration 113
cross_track_err 0.0384513735003173 , steering_p -0.00769027470006346 , steering_d 0.003782747419437779 steering_i -0.16984824956052239 , steering -0.17375577684114807
steering_drift 0.17453292519943295 steering2 0.0007771483582848793
after move, [x=113.99396 y=0.03723 orient=6.28200]
iteration 114
cross_track_err 0.03723062502372844 , steering_p -0.007446125004745688 , steering_d 0.003662245429766578 steering_i -0.16999717206061732 , steering -0.17378105163559643
steering_drift 0.17453292519943295 steering2 0.0007518735638365193
after move, [x=114.99396 y=0.03605 orient=6.28204]
iteration 115
cross_track_err 0.03604873394483482 , steering_p -0.007209746788966964 , steering_d 0.0035456732366808547 steering_i -0.17014136699639668 , steering -0.1738054405486828
steering_drift 0.17453292519943295 steering2 0.0007274846507501487
after move, [x=115.99395 y=0.03490 orient=6.28208]
iteration 116
cross_track_err 0.03490443652578673 , steering_p -0.006980887305157346 , steering_d 0.003432892257144271 steering_i -0.1702809847424998 , steering -0.17382897979051287
steering_drift 0.17453292519943295 steering2 0.0007039454089200758
after move, [x=116.99395 y=0.03380 orient=6.28211]
iteration 117
cross_track_err 0.03379651332262729 , steering_p -0.006759302664525459 , steering_d 0.0033237696094783123 steering_i -0.17041617079579033 , steering -0.17385170385083748
steering_drift 0.17453292519943295 steering2 0.0006812213485954677
after move, [x=117.99395 y=0.03272 orient=6.28215]
iteration 118
cross_track_err 0.032723787374804414 , steering_p -0.006544757474960883 , steering_d 0.003218177843468638 steering_i -0.17054706594528954 , steering -0.17387364557678178
steering_drift 0.17453292519943295 steering2 0.0006592796226511655
after move, [x=118.99395 y=0.03169 orient=6.28218]
iteration 119
cross_track_err 0.0316851224806978 , steering_p -0.00633702449613956 , steering_d 0.0031159946823198503 steering_i -0.17067380643521232 , steering -0.17389483624903204
steering_drift 0.17453292519943295 steering2 0.0006380889504009057
after move, [x=119.99395 y=0.03068 orient=6.28221]
iteration 120
cross_track_err 0.030679421555277246 , steering_p -0.00613588431105545 , steering_d 0.003017102776261653 steering_i -0.17079652412143345 , steering -0.17391530565622726
steering_drift 0.17453292519943295 steering2 0.0006176195432056886
after move, [x=120.99395 y=0.02971 orient=6.28224]
iteration 121
cross_track_err 0.029705625066078694 , steering_p -0.0059411250132157395 , steering_d 0.0029213894675956566 steering_i -0.17091534662169774 , steering -0.17393508216731782
steering_drift 0.17453292519943295 steering2 0.0005978430321151307
after move, [x=121.99395 y=0.02876 orient=6.28227]
iteration 122
cross_track_err 0.028762709543784598 , steering_p -0.00575254190875692 , steering_d 0.0028287465668822877 steering_i -0.1710303974598729 , steering -0.1739541928017475
steering_drift 0.17453292519943295 steering2 0.0005787323976854386
after move, [x=122.99395 y=0.02785 orient=6.28230]
iteration 123
cross_track_err 0.027849686163786042 , steering_p -0.005569937232757209 , steering_d 0.0027390701399956675 steering_i -0.17114179620452802 , steering -0.17397266329728955
steering_drift 0.17453292519943295 steering2 0.0005602619021433941
after move, [x=123.99395 y=0.02697 orient=6.28233]
iteration 124
cross_track_err 0.02696559939521928 , steering_p -0.005393119879043857 , steering_d 0.0026522603057002846 steering_i -0.1712496586021089 , steering -0.17399051817545247
steering_drift 0.17453292519943295 steering2 0.000542407023980479
after move, [x=124.99395 y=0.02611 orient=6.28236]
iteration 125
cross_track_err 0.02610952571408666 , steering_p -0.005221905142817332 , steering_d 0.002568221043397864 steering_i -0.17135409670496524 , steering -0.1740077808043847
steering_drift 0.17453292519943295 steering2 0.0005251443950482493
after move, [x=125.99395 y=0.02528 orient=6.28238]
iteration 126
cross_track_err 0.025280572377186457 , steering_p -0.0050561144754372915 , steering_d 0.0024868600107006075 steering_i -0.17145521899447397 , steering -0.17402447345921065
steering_drift 0.17453292519943295 steering2 0.0005084517402222932
after move, [x=126.99395 y=0.02448 orient=6.28241]
iteration 127
cross_track_err 0.024477876253713803 , steering_p -0.004895575250742761 , steering_d 0.002408088370417963 steering_i -0.1715531304994888 , steering -0.1740406173798136
steering_drift 0.17453292519943295 steering2 0.000492307819619342
after move, [x=127.99395 y=0.02370 orient=6.28243]
iteration 128
cross_track_err 0.023700602711509443 , steering_p -0.004740120542301889 , steering_d 0.002331820626613079 steering_i -0.17164793291033484 , steering -0.17405623282602364
steering_drift 0.17453292519943295 steering2 0.00047669237340930426
after move, [x=128.99395 y=0.02295 orient=6.28246]
iteration 129
cross_track_err 0.022947944555071605 , steering_p -0.004589588911014321 , steering_d 0.0022579744693135145 steering_i -0.17173972468855514 , steering -0.17407133913025594
steering_drift 0.17453292519943295 steering2 0.0004615860691770113
after move, [x=129.99395 y=0.02222 orient=6.28248]
iteration 130
cross_track_err 0.022219121012569714 , steering_p -0.004443824202513943 , steering_d 0.002186470627505672 steering_i -0.1718286011726054 , steering -0.17408595474761368
steering_drift 0.17453292519943295 steering2 0.0004469704518192674
after move, [x=130.99395 y=0.02151 orient=6.28250]
iteration 131
cross_track_err 0.021513376769228355 , steering_p -0.004302675353845671 , steering_d 0.002117232730024076 steering_i -0.1719146546796823 , steering -0.1741000973035039
steering_drift 0.17453292519943295 steering2 0.0004328278959290499
after move, [x=131.99395 y=0.02083 orient=6.28252]
iteration 132
cross_track_err 0.0208299810445751 , steering_p -0.004165996208915021 , steering_d 0.0020501871739597617 steering_i -0.17199797460386063 , steering -0.17411378363881588
steering_drift 0.17453292519943295 steering2 0.00041914156061706453
after move, [x=132.99395 y=0.02017 orient=6.28254]
iteration 133
cross_track_err 0.020168226711174624 , steering_p -0.004033645342234925 , steering_d 0.0019852630002014336 steering_i -0.17207864751070534 , steering -0.17412702985273884
steering_drift 0.17453292519943295 steering2 0.0004058953466941073
after move, [x=133.99395 y=0.01953 orient=6.28256]
iteration 134
cross_track_err 0.01952742945258694 , steering_p -0.003905485890517388 , steering_d 0.0019223917757630535 steering_i -0.1721567572285157 , steering -0.17413985134327004
steering_drift 0.17453292519943295 steering2 0.00039307385616291213
after move, [x=134.99395 y=0.01891 orient=6.28258]
iteration 135
cross_track_err 0.018906926958411986 , steering_p -0.0037813853916823974 , steering_d 0.0018615074825248587 steering_i -0.17223238493634935 , steering -0.1741522628455069
steering_drift 0.17453292519943295 steering2 0.00038066235392605896
after move, [x=135.99395 y=0.01831 orient=6.28260]
iteration 136
cross_track_err 0.018306078154391996 , steering_p -0.003661215630878399 , steering_d 0.0018025464120599716 steering_i -0.1723056092489669 , steering -0.17416427846778532
steering_drift 0.17453292519943295 steering2 0.0003686467316476316
after move, [x=136.99395 y=0.01772 orient=6.28262]
iteration 137
cross_track_err 0.017724262465659422 , steering_p -0.0035448524931318845 , steering_d 0.0017454470661977216 steering_i -0.17237650629882956 , steering -0.17417591172576372
steering_drift 0.17453292519943295 steering2 0.0003570134736692321
after move, [x=137.99395 y=0.01716 orient=6.28264]
iteration 138
cross_track_err 0.017160879111322153 , steering_p -0.003432175822264431 , steering_d 0.0016901500630118055 steering_i -0.17244514981527484 , steering -0.17418717557452745
steering_drift 0.17453292519943295 steering2 0.00034574962490549477
after move, [x=138.99395 y=0.01662 orient=6.28266]
iteration 139
cross_track_err 0.0166153464286825 , steering_p -0.0033230692857365 , steering_d 0.0016365980479189635 steering_i -0.17251161120098957 , steering -0.1741980824388071
steering_drift 0.17453292519943295 steering2 0.00033484276062584195
after move, [x=139.99395 y=0.01609 orient=6.28267]
iteration 140
cross_track_err 0.016087101225485043 , steering_p -0.003217420245097009 , steering_d 0.0015847356095923665 steering_i -0.1725759596058915 , steering -0.17420864424139615
steering_drift 0.17453292519943295 steering2 0.0003242809580367989
after move, [x=140.99395 y=0.01558 orient=6.28269]
iteration 141
cross_track_err 0.015575598158681717 , steering_p -0.0031151196317363433 , steering_d 0.0015345092004099802 steering_i -0.17263826199852625 , steering -0.17421887242985262
steering_drift 0.17453292519943295 steering2 0.00031405276958032524
after move, [x=141.99395 y=0.01508 orient=6.28271]
iteration 142
cross_track_err 0.015080309138293908 , steering_p -0.003016061827658782 , steering_d 0.0014858670611634268 steering_i -0.17269858323507942 , steering -0.17422877800157477
steering_drift 0.17453292519943295 steering2 0.0003041471978581789
after move, [x=142.99395 y=0.01460 orient=6.28272]
iteration 143
cross_track_err 0.014600722755035806 , steering_p -0.0029201445510071613 , steering_d 0.0014387591497743036 steering_i -0.17275698612609955 , steering -0.1742383715273324
steering_drift 0.17453292519943295 steering2 0.00029455367210054484
after move, [x=143.99395 y=0.01414 orient=6.28274]
iteration 144
cross_track_err 0.014136343730445795 , steering_p -0.002827268746089159 , steering_d 0.0013931370737700344 steering_i -0.17281353150102136 , steering -0.17424766317334048
steering_drift 0.17453292519943295 steering2 0.00028526202609246987
after move, [x=144.99395 y=0.01369 orient=6.28275]
iteration 145
cross_track_err 0.013686692388348163 , steering_p -0.0027373384776696328 , steering_d 0.0013489540262928961 steering_i -0.17286827827057474 , steering -0.17425666272195148
steering_drift 0.17453292519943295 steering2 0.00027626247748147037
after move, [x=145.99395 y=0.01325 orient=6.28276]
iteration 146
cross_track_err 0.013251304146545607 , steering_p -0.0026502608293091216 , steering_d 0.0013061647254076665 steering_i -0.1729212834871609 , steering -0.17426537959106236
steering_drift 0.17453292519943295 steering2 0.00026754560837058405
after move, [x=146.99395 y=0.01283 orient=6.28278]
iteration 147
cross_track_err 0.012829729027700692 , steering_p -0.0025659458055401385 , steering_d 0.0012647253565347457 steering_i -0.17297260240327172 , steering -0.1742738228522771
steering_drift 0.17453292519943295 steering2 0.0002591023471558429
after move, [x=147.99395 y=0.01242 orient=6.28279]
iteration 148
cross_track_err 0.012421531188441715 , steering_p -0.0024843062376883433 , steering_d 0.001224593517776932 steering_i -0.17302228852802548 , steering -0.17428200124793689
steering_drift 0.17453292519943295 steering2 0.0002509239514960626
after move, [x=148.99395 y=0.01203 orient=6.28280]
iteration 149
cross_track_err 0.012026288465785287 , steering_p -0.0024052576931570574 , steering_d 0.0011857281679692848 steering_i -0.17307039368188862 , steering -0.1742899232070764
steering_drift 0.17453292519943295 steering2 0.00024300199235655073
after move, [x=149.99395 y=0.01164 orient=6.28281]
iteration 150
cross_track_err 0.011643591940017644 , steering_p -0.0023287183880035286 , steering_d 0.0011480895773029291 steering_i -0.17311696804964868 , steering -0.1742975968603493
steering_drift 0.17453292519943295 steering2 0.0002353283390836569
after move, [x=150.99395 y=0.01127 orient=6.28283]
iteration 151
cross_track_err 0.011273045513245107 , steering_p -0.0022546091026490215 , steering_d 0.0011116392803176105 steering_i -0.17316206023170164 , steering -0.17430503005403306
steering_drift 0.17453292519943295 steering2 0.0002278951453998923
after move, [x=151.99395 y=0.01091 orient=6.28284]
iteration 152
cross_track_err 0.010914265502861424 , steering_p -0.0021828531005722847 , steering_d 0.001076340031151049 steering_i -0.1732057172937131 , steering -0.1743122303631343
steering_drift 0.17453292519943295 steering2 0.00022069483629863496
after move, [x=152.99395 y=0.01057 orient=6.28285]
iteration 153
cross_track_err 0.010566880249234765 , steering_p -0.002113376049846953 , steering_d 0.0010421557608799748 steering_i -0.17324798481471004 , steering -0.17431920510367702
steering_drift 0.17453292519943295 steering2 0.00021372009575593154
after move, [x=153.99395 y=0.01023 orient=6.28286]
iteration 154
cross_track_err 0.010230529736957148 , steering_p -0.00204610594739143 , steering_d 0.0010090515368328511 steering_i -0.17328890693365787 , steering -0.17432596134421643
steering_drift 0.17453292519943295 steering2 0.00020696385521651317
after move, [x=154.99395 y=0.00990 orient=6.28287]
iteration 155
cross_track_err 0.009904865229044314 , steering_p -0.001980973045808863 , steering_d 0.0009769935237385046 steering_i -0.17332852639457405 , steering -0.1743325059166444
steering_drift 0.17453292519943295 steering2 0.0002004192827885498
after move, [x=155.99395 y=0.00959 orient=6.28288]
iteration 156
cross_track_err 0.009589548913508774 , steering_p -0.0019179097827017547 , steering_d 0.0009459489466066198 steering_i -0.1733668845902281 , steering -0.17433884542632322
steering_drift 0.17453292519943295 steering2 0.0001940797731097288
after move, [x=156.99395 y=0.00928 orient=6.28289]
iteration 157
cross_track_err 0.009284253561764228 , steering_p -0.0018568507123528457 , steering_d 0.0009158860552336365 steering_i -0.17340402160447516 , steering -0.17434498626159436
steering_drift 0.17453292519943295 steering2 0.00018793893783858318
after move, [x=157.99395 y=0.00899 orient=6.28290]
iteration 158
cross_track_err 0.008988662198358649 , steering_p -0.0017977324396717298 , steering_d 0.0008867740902167385 steering_i -0.1734399762532686 , steering -0.17435093460272358
steering_drift 0.17453292519943295 steering2 0.00018199059670936935
after move, [x=158.99395 y=0.00870 orient=6.28291]
iteration 159
cross_track_err 0.008702467781558205 , steering_p -0.0017404935563116411 , steering_d 0.0008585832504013294 steering_i -0.17347478612439482 , steering -0.17435669643030513
steering_drift 0.17453292519943295 steering2 0.00017622876912781749
after move, [x=159.99395 y=0.00843 orient=6.28292]
iteration 160
cross_track_err 0.008425372894332601 , steering_p -0.0016850745788665204 , steering_d 0.0008312846616768121 steering_i -0.17350848761597215 , steering -0.17436227753316186
steering_drift 0.17453292519943295 steering2 0.00017064766627108985
after move, [x=160.99395 y=0.00816 orient=6.28293]
iteration 161
cross_track_err 0.008157089445327058 , steering_p -0.0016314178890654116 , steering_d 0.0008048503470166318 steering_i -0.17354111597375346 , steering -0.17436768351580223
steering_drift 0.17453292519943295 steering2 0.000165241683630718
after move, [x=161.99395 y=0.00790 orient=6.28293]
iteration 162
cross_track_err 0.007897338379420377 , steering_p -0.0015794676758840755 , steering_d 0.0007792531977200406 steering_i -0.17357270532727115 , steering -0.17437291980543518
steering_drift 0.17453292519943295 steering2 0.0001600053939977697
after move, [x=162.99395 y=0.00765 orient=6.28294]
iteration 163
cross_track_err 0.007645849397500494 , steering_p -0.001529169879500099 , steering_d 0.0007544669457596499 steering_i -0.17360328872486114 , steering -0.1743779916586016
steering_drift 0.17453292519943295 steering2 0.00015493354083134792
after move, [x=163.99395 y=0.00740 orient=6.28295]
iteration 164
cross_track_err 0.007402360685104016 , steering_p -0.0014804721370208034 , steering_d 0.0007304661371894334 steering_i -0.17363289816760155 , steering -0.17438290416743293
steering_drift 0.17453292519943295 steering2 0.00015002103200001393
after move, [x=164.99395 y=0.00717 orient=6.28296]
iteration 165
cross_track_err 0.007166618649588477 , steering_p -0.0014333237299176955 , steering_d 0.000707226106546617 steering_i -0.17366156464219992 , steering -0.174387662265571
steering_drift 0.17453292519943295 steering2 0.00014526293386193934
after move, [x=165.99395 y=0.00694 orient=6.28296]
iteration 166
cross_track_err 0.006938377665527323 , steering_p -0.0013876755331054648 , steering_d 0.0006847229521834619 steering_i -0.17368931815286204 , steering -0.17439227073378405
steering_drift 0.17453292519943295 steering2 0.00014065446564889839
after move, [x=166.99395 y=0.00672 orient=6.28297]
iteration 167
cross_track_err 0.006717399828027428 , steering_p -0.0013434799656054856 , steering_d 0.0006629335124996869 steering_i -0.17371618775217415 , steering -0.17439673420527996
steering_drift 0.17453292519943295 steering2 0.0001361909941529904
after move, [x=167.99395 y=0.00650 orient=6.28298]
iteration 168
cross_track_err 0.006503454713690256 , steering_p -0.0013006909427380512 , steering_d 0.0006418353430115153 steering_i -0.17374220157102888 , steering -0.1744010571707554
steering_drift 0.17453292519943295 steering2 0.00013186802867753977
after move, [x=168.99395 y=0.00630 orient=6.28298]
iteration 169
cross_track_err 0.006296319148952 , steering_p -0.0012592638297904 , steering_d 0.0006214066942147678 steering_i -0.1737673868476247 , steering -0.17440524398320031
steering_drift 0.17453292519943295 steering2 0.0001276812162326335
after move, [x=169.99395 y=0.00610 orient=6.28299]
iteration 170
cross_track_err 0.006095776985548664 , steering_p -0.0012191553971097329 , steering_d 0.0006016264902100072 steering_i -0.1737917699555669 , steering -0.17440929886246662
steering_drift 0.17453292519943295 steering2 0.00012362633696633152
after move, [x=170.99395 y=0.00590 orient=6.28300]
iteration 171
cross_track_err 0.005901618882867185 , steering_p -0.0011803237765734371 , steering_d 0.0005824743080444384 steering_i -0.17381537643109835 , steering -0.17441322589962735
steering_drift 0.17453292519943295 steering2 0.0001196992998055979
after move, [x=171.99395 y=0.00571 orient=6.28300]
iteration 172
cross_track_err 0.005713642096952548 , steering_p -0.0011427284193905097 , steering_d 0.0005639303577439098 steering_i -0.17383823099948614 , steering -0.17441702906113274
steering_drift 0.17453292519943295 steering2 0.00011589613830020884
after move, [x=172.99395 y=0.00553 orient=6.28301]
iteration 173
cross_track_err 0.005531650275954193 , steering_p -0.0011063300551908386 , steering_d 0.0005459754629950665 steering_i -0.17386035760058996 , steering -0.17442071219278574
steering_drift 0.17453292519943295 steering2 0.00011221300664721046
after move, [x=173.99395 y=0.00536 orient=6.28301]
iteration 174
cross_track_err 0.005355453261803858 , steering_p -0.0010710906523607715 , steering_d 0.0005285910424510046 steering_i -0.1738817794136372 , steering -0.17442427902354696
steering_drift 0.17453292519943295 steering2 0.00010864617588599024
after move, [x=174.99395 y=0.00518 orient=6.28302]
iteration 175
cross_track_err 0.005184866897925044 , steering_p -0.0010369733795850089 , steering_d 0.0005117590916364411 steering_i -0.1739025188812289 , steering -0.17442773316917748
steering_drift 0.17453292519943295 steering2 0.00010519203025546875
after move, [x=175.99395 y=0.00502 orient=6.28303]
iteration 176
cross_track_err 0.0050197128427857925 , steering_p -0.0010039425685571585 , steering_d 0.0004954621654177545 steering_i -0.17392259773260005 , steering -0.17443107813573946
steering_drift 0.17453292519943295 steering2 0.00010184706369348318
after move, [x=176.99395 y=0.00486 orient=6.28303]
iteration 177
cross_track_err 0.004859818389109152 , steering_p -0.0009719636778218305 , steering_d 0.0004796833610299211 steering_i -0.17394203700615649 , steering -0.1744343173229484
steering_drift 0.17453292519943295 steering2 9.860787648455216e-05
after move, [x=177.99395 y=0.00471 orient=6.28304]
iteration 178
cross_track_err 0.004705016288571696 , steering_p -0.0009410032577143393 , steering_d 0.00046440630161236757 steering_i -0.17396085707131076 , steering -0.17443745402741273
steering_drift 0.17453292519943295 steering2 9.547117202021727e-05
after move, [x=178.99395 y=0.00456 orient=6.28304]
iteration 179
cross_track_err 0.00455514458181689 , steering_p -0.0009110289163633781 , steering_d 0.0004496151202644178 steering_i -0.17397907764963802 , steering -0.17444049144573698
steering_drift 0.17453292519943295 steering2 9.243375369596962e-05
after move, [x=179.99395 y=0.00441 orient=6.28304]
iteration 180
cross_track_err 0.004410046433626108 , steering_p -0.0008820092867252217 , steering_d 0.00043529444457234695 steering_i -0.17399671783537252 , steering -0.17444343267752538
steering_drift 0.17453292519943295 steering2 8.949252190756929e-05
after move, [x=180.99395 y=0.00427 orient=6.28305]
iteration 181
cross_track_err 0.004269569973086533 , steering_p -0.0008539139946173067 , steering_d 0.00042142938161872454 steering_i -0.17401379611526488 , steering -0.17444628072826346
steering_drift 0.17453292519943295 steering2 8.664447116948915e-05
after move, [x=181.99395 y=0.00413 orient=6.28305]
iteration 182
cross_track_err 0.004133568138611175 , steering_p -0.0008267136277222351 , steering_d 0.00040800550342607374 steering_i -0.17403033038781934 , steering -0.17444903851211552
steering_drift 0.17453292519943295 steering2 8.388668731743043e-05
after move, [x=182.99395 y=0.00400 orient=6.28306]
iteration 183
cross_track_err 0.004001898527666229 , steering_p -0.0008003797055332458 , steering_d 0.0003950088328348398 steering_i -0.17404633798193 , steering -0.1744517088546284
steering_drift 0.17453292519943295 steering2 8.12163448045411e-05
after move, [x=183.99395 y=0.00387 orient=6.28306]
iteration 184
cross_track_err 0.003874423251061888 , steering_p -0.0007748846502123776 , steering_d 0.00038242582981302215 steering_i -0.17406183567493427 , steering -0.17445429449533362
steering_drift 0.17453292519943295 steering2 7.86307040993306e-05
after move, [x=184.99395 y=0.00375 orient=6.28307]
iteration 185
cross_track_err 0.0037510087916751665 , steering_p -0.0007502017583350334 , steering_d 0.00037024337816016434 steering_i -0.17407683971010096 , steering -0.17445679809027584
steering_drift 0.17453292519943295 steering2 7.612710915710919e-05
after move, [x=185.99395 y=0.00363 orient=6.28307]
iteration 186
cross_track_err 0.00363152586747239 , steering_p -0.000726305173494478 , steering_d 0.00035844877260832967 steering_i -0.17409136581357085 , steering -0.174459222214457
steering_drift 0.17453292519943295 steering2 7.370298497594296e-05
after move, [x=186.99395 y=0.00352 orient=6.28307]
iteration 187
cross_track_err 0.0035158492987087853 , steering_p -0.0007031698597417571 , steering_d 0.00034702970629081403 steering_i -0.17410542921076566 , steering -0.1744615693642166
steering_drift 0.17453292519943295 steering2 7.13558352163357e-05
after move, [x=187.99395 y=0.00340 orient=6.28308]
iteration 188
cross_track_err 0.003403857879176386 , steering_p -0.0006807715758352772 , steering_d 0.0003359742585971981 steering_i -0.17411904464228237 , steering -0.17446384195952044
steering_drift 0.17453292519943295 steering2 6.908323991250409e-05
after move, [x=188.99395 y=0.00330 orient=6.28308]
iteration 189
cross_track_err 0.00329543425138923 , steering_p -0.0006590868502778461 , steering_d 0.00032527088336146764 steering_i -0.17413222637928794 , steering -0.1744660423462043
steering_drift 0.17453292519943295 steering2 6.688285322864473e-05
after move, [x=189.99395 y=0.00319 orient=6.28308]
iteration 190
cross_track_err 0.003190464785583282 , steering_p -0.0006380929571166565 , steering_d 0.0003149083974178439 steering_i -0.17414498823843028 , steering -0.1744681727981291
steering_drift 0.17453292519943295 steering2 6.475240130385251e-05
after move, [x=190.99395 y=0.00309 orient=6.28309]
iteration 191
cross_track_err 0.0030888394624262743 , steering_p -0.0006177678924852549 , steering_d 0.00030487596947102345 steering_i -0.17415734359628 , steering -0.17447023551929422
steering_drift 0.17453292519943295 steering2 6.268968013872778e-05
after move, [x=191.99395 y=0.00299 orient=6.28309]
iteration 192
cross_track_err 0.002990451759322893 , steering_p -0.0005980903518645786 , steering_d 0.0002951631093101436 steering_i -0.1741693054033173 , steering -0.17447223264587172
steering_drift 0.17453292519943295 steering2 6.0692553561225715e-05
after move, [x=192.99395 y=0.00290 orient=6.28309]
iteration 193
cross_track_err 0.0028951985402158324 , steering_p -0.0005790397080431665 , steering_d 0.0002857596573211819 steering_i -0.17418088619747815 , steering -0.17447416624820014
steering_drift 0.17453292519943295 steering2 5.875895123280683e-05
after move, [x=193.99394 y=0.00280 orient=6.28310]
iteration 194
cross_track_err 0.0028029799487770254 , steering_p -0.0005605959897554051 , steering_d 0.0002766557743164211 steering_i -0.17419209811727326 , steering -0.17447603833271225
steering_drift 0.17453292519943295 steering2 5.68868667207012e-05
after move, [x=194.99394 y=0.00271 orient=6.28310]
iteration 195
cross_track_err 0.0027136993048913487 , steering_p -0.0005427398609782698 , steering_d 0.00026784193165703017 steering_i -0.1742029529144928 , steering -0.17447785084381404
steering_drift 0.17453292519943295 steering2 5.50743556189115e-05
after move, [x=195.99394 y=0.00263 orient=6.28310]
iteration 196
cross_track_err 0.0026272630043342182 , steering_p -0.0005254526008668436 , steering_d 0.0002593089016713914 steering_i -0.17421346196651016 , steering -0.1744796056657056
steering_drift 0.17453292519943295 steering2 5.331953372733622e-05
after move, [x=196.99394 y=0.00254 orient=6.28310]
iteration 197
cross_track_err 0.002543580421550702 , steering_p -0.0005087160843101405 , steering_d 0.00025104774835054853 steering_i -0.17422363628819637 , steering -0.17448130462415595
steering_drift 0.17453292519943295 steering2 5.162057527699493e-05
after move, [x=197.99394 y=0.00246 orient=6.28311]
iteration 198
cross_track_err 0.002462563815447332 , steering_p -0.0004925127630894665 , steering_d 0.00024304981831010973 steering_i -0.17423348654345816 , steering -0.1744829494882375
steering_drift 0.17453292519943295 steering2 4.997571119544353e-05
after move, [x=198.99394 y=0.00238 orient=6.28311]
iteration 199
cross_track_err 0.0023841282381015833 , steering_p -0.0004768256476203167 , steering_d 0.0002353067320372466 steering_i -0.17424302305641054 , steering -0.17448454197199362
steering_drift 0.17453292519943295 steering2 4.838322743933032e-05
after move, [x=199.99394 y=0.00231 orient=6.28311]
'''