# -----------
# User Instructions
#
# Implement a P controller by running 100 iterations
# of robot motion. The desired trajectory for the
# robot is the x-axis. The steering angle should be set
# by the parameter tau so that:
#
# steering = -tau * crosstrack_error
#
# You'll only need to modify the `run` function at the bottom.
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
        # NOTE: the angle of orientation
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

        # Execute motion
        # QA: why?
        # NOTE: seems turn is angle change of orientation
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

def run(robot, tau, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []

    # TODO: your code here
    print('start')
    print(robot)

    x_trajectory.append(robot.x)
    y_trajectory.append(robot.y)

    time_interval = 1
    dist = speed * time_interval
    for i in range(n):
        print('iteration', i)

        cross_track_err = robot.y - 0
        steering = -tau * cross_track_err
        print('cross_track_err', cross_track_err, ', steering', steering)

        robot.move(steering, dist)
        print('after move,', robot)

        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)

    return x_trajectory, y_trajectory

x_trajectory, y_trajectory = run(robot, 0.1)
# x_trajectory, y_trajectory = run(robot, 0.1, 1000)

# x_trajectory, y_trajectory = run(robot, 0.01, 1000)
# x_trajectory, y_trajectory = run(robot, 0.01, n=10000, speed=0.1)

n = len(x_trajectory)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.plot(x_trajectory, y_trajectory, 'g', label='P controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')

# TODO: your code here
ax1.legend()
plt.show()


'''
tau = 0.1

(env) Ruis-MacBook-Pro-15:4_implement_p_controller ruiwang$ python main.py
start
[x=0.00000 y=1.00000 orient=0.00000]
iteration  0
cross_track_err  1 , steering  -0.1
after move,  [x=1.00000 y=0.99749 orient=6.27817]
iteration  1
cross_track_err  0.997491638458655 , steering  -0.0997491638458655
after move,  [x=1.99997 y=0.98997 orient=6.27316]
iteration  2
cross_track_err  0.9899729506124686 , steering  -0.09899729506124687
after move,  [x=2.99989 y=0.97747 orient=6.26820]
iteration  3
cross_track_err  0.977469440459231 , steering  -0.0977469440459231
after move,  [x=3.99973 y=0.96003 orient=6.26330]
iteration  4
cross_track_err  0.960031957433074 , steering  -0.0960031957433074
after move,  [x=4.99948 y=0.93774 orient=6.25848]
iteration  5
cross_track_err  0.9377364753807171 , steering  -0.09377364753807171
after move,  [x=5.99912 y=0.91068 orient=6.25378]
iteration  6
cross_track_err  0.9106837322063086 , steering  -0.09106837322063087
after move,  [x=6.99861 y=0.87900 orient=6.24921]
iteration  7
cross_track_err  0.8789987335156866 , steering  -0.08789987335156867
after move,  [x=7.99796 y=0.84283 orient=6.24481]
iteration  8
cross_track_err  0.8428301247961656 , steering  -0.08428301247961656
after move,  [x=8.99714 y=0.80235 orient=6.24058]
iteration  9
cross_track_err  0.8023494377461873 , steering  -0.08023494377461873
after move,  [x=9.99614 y=0.75775 orient=6.23656]
iteration  10
cross_track_err  0.7577502172916581 , steering  -0.07577502172916582
after move,  [x=10.99497 y=0.70925 orient=6.23277]
iteration  11
cross_track_err  0.7092470365780059 , steering  -0.0709247036578006
after move,  [x=11.99360 y=0.65707 orient=6.22921]
iteration  12
cross_track_err  0.6570744077966992 , steering  -0.06570744077966993
after move,  [x=12.99206 y=0.60149 orient=6.22592]
iteration  13
cross_track_err  0.6014855970898907 , steering  -0.06014855970898907
after move,  [x=13.99033 y=0.54275 orient=6.22291]
iteration  14
cross_track_err  0.5427513519858849 , steering  -0.05427513519858849
after move,  [x=14.98843 y=0.48116 orient=6.22020]
iteration  15
cross_track_err  0.48115854985655915 , steering  -0.04811585498565592
after move,  [x=15.98637 y=0.41701 orient=6.21779]
iteration  16
cross_track_err  0.4170087757827332 , steering  -0.04170087757827332
after move,  [x=16.98416 y=0.35062 orient=6.21570]
iteration  17
cross_track_err  0.3506168379843757 , steering  -0.03506168379843757
after move,  [x=17.98183 y=0.28231 orient=6.21395]
iteration  18
cross_track_err  0.2823092286467954 , steering  -0.028230922864679542
after move,  [x=18.97938 y=0.21242 orient=6.21254]
iteration  19
cross_track_err  0.2124225375861215 , steering  -0.02124225375861215
after move,  [x=19.97685 y=0.14130 orient=6.21147]
iteration  20
cross_track_err  0.14130182577480355 , steering  -0.014130182577480355
after move,  [x=20.97428 y=0.06965 orient=6.21077]
iteration  21
cross_track_err  0.06965132942895697 , steering  -0.006965132942895697
after move,  [x=21.97166 y=-0.00270 orient=6.21042]
iteration  22
cross_track_err  -0.0027038891366995277 , steering  0.0002703889136699528
after move,  [x=22.96901 y=-0.07541 orient=6.21043]
iteration  23
cross_track_err  -0.07540645276524682 , steering  0.007540645276524683
after move,  [x=23.96637 y=-0.14810 orient=6.21081]
iteration  24
cross_track_err  -0.14809553271809206 , steering  0.014809553271809207
after move,  [x=24.96375 y=-0.22041 orient=6.21155]
iteration  25
cross_track_err  -0.22040856550422522 , steering  0.022040856550422525
after move,  [x=25.96122 y=-0.29143 orient=6.21265]
iteration  26
cross_track_err  -0.2914332738077974 , steering  0.029143327380779738
after move,  [x=26.95879 y=-0.36118 orient=6.21411]
iteration  27
cross_track_err  -0.3611812538382537 , steering  0.036118125383825375
after move,  [x=27.95647 y=-0.42930 orient=6.21592]
iteration  28
cross_track_err  -0.4293009779287331 , steering  0.042930097792873316
after move,  [x=28.95428 y=-0.49545 orient=6.21806]
iteration  29
cross_track_err  -0.49544790132125627 , steering  0.04954479013212563
after move,  [x=29.95224 y=-0.55929 orient=6.22054]
iteration  30
cross_track_err  -0.5592861665387545 , steering  0.055928616653875454
after move,  [x=30.95036 y=-0.62049 orient=6.22334]
iteration  31
cross_track_err  -0.6204902830029937 , steering  0.062049028300299375
after move,  [x=31.94866 y=-0.67875 orient=6.22645]
iteration  32
cross_track_err  -0.6787467746748348 , steering  0.06787467746748348
after move,  [x=32.94715 y=-0.73376 orient=6.22985]
iteration  33
cross_track_err  -0.7337557879528163 , steering  0.07337557879528163
after move,  [x=33.94582 y=-0.78523 orient=6.23352]
iteration  34
cross_track_err  -0.7852326515236996 , steering  0.07852326515236996
after move,  [x=34.94468 y=-0.83291 orient=6.23746]
iteration  35
cross_track_err  -0.8329093793539073 , steering  0.08329093793539073
after move,  [x=35.94373 y=-0.87654 orient=6.24163]
iteration  36
cross_track_err  -0.876536107577607 , steering  0.08765361075776071
after move,  [x=36.94296 y=-0.91588 orient=6.24603]
iteration  37
cross_track_err  -0.9158824557055993 , steering  0.09158824557055995
after move,  [x=37.94235 y=-0.95074 orient=6.25062]
iteration  38
cross_track_err  -0.9507388023884573 , steering  0.09507388023884573
after move,  [x=38.94189 y=-0.98092 orient=6.25539]
iteration  39
cross_track_err  -0.980917465937722 , steering  0.0980917465937722
after move,  [x=39.94157 y=-1.00625 orient=6.26031]
iteration  40
cross_track_err  -1.0062537799705922 , steering  0.10062537799705923
after move,  [x=40.94136 y=-1.02661 orient=6.26535]
iteration  41
cross_track_err  -1.0266070549061794 , steering  0.10266070549061795
after move,  [x=41.94124 y=-1.04186 orient=6.27051]
iteration  42
cross_track_err  -1.0418614166191276 , steering  0.10418614166191276
after move,  [x=42.94119 y=-1.05193 orient=6.27573]
iteration  43
cross_track_err  -1.0519265143439043 , steering  0.10519265143439044
after move,  [x=43.94118 y=-1.05674 orient=6.28101]
iteration  44
cross_track_err  -1.0567380909136546 , steering  0.10567380909136546
after move,  [x=44.94118 y=-1.05626 orient=0.00313]
iteration  45
cross_track_err  -1.0562584095915497 , steering  0.10562584095915498
after move,  [x=45.94116 y=-1.05048 orient=0.00843]
iteration  46
cross_track_err  -1.0504765330828434 , steering  0.10504765330828435
after move,  [x=46.94110 y=-1.03941 orient=0.01370]
iteration  47
cross_track_err  -1.0394084517667466 , steering  0.10394084517667466
after move,  [x=47.94096 y=-1.02310 orient=0.01892]
iteration  48
cross_track_err  -1.0230970597212377 , steering  0.10230970597212377
after move,  [x=48.94073 y=-1.00161 orient=0.02405]
iteration  49
cross_track_err  -1.0016119786812112 , steering  0.10016119786812112
after move,  [x=49.94038 y=-0.97505 orient=0.02908]
iteration  50
cross_track_err  -0.9750492316308907 , steering  0.09750492316308908
after move,  [x=50.93988 y=-0.94353 orient=0.03397]
iteration  51
cross_track_err  -0.9435307692322397 , steering  0.09435307692322398
after move,  [x=51.93922 y=-0.90720 orient=0.03870]
iteration  52
cross_track_err  -0.9072038536945684 , steering  0.09072038536945684
after move,  [x=52.93838 y=-0.86624 orient=0.04325]
iteration  53
cross_track_err  -0.8662403059544488 , steering  0.08662403059544488
after move,  [x=53.93735 y=-0.82084 orient=0.04759]
iteration  54
cross_track_err  -0.8208356231274081 , steering  0.08208356231274082
after move,  [x=54.93611 y=-0.77121 orient=0.05170]
iteration  55
cross_track_err  -0.7712079740920785 , steering  0.07712079740920785
after move,  [x=55.93467 y=-0.71760 orient=0.05557]
iteration  56
cross_track_err  -0.7175970817544339 , steering  0.0717597081754434
after move,  [x=56.93303 y=-0.66026 orient=0.05916]
iteration  57
cross_track_err  -0.660263001011856 , steering  0.0660263001011856
after move,  [x=57.93118 y=-0.59948 orient=0.06247]
iteration  58
cross_track_err  -0.5994848016961782 , steering  0.059948480169617825
after move,  [x=58.92913 y=-0.53556 orient=0.06547]
iteration  59
cross_track_err  -0.5355591658336039 , steering  0.05355591658336039
after move,  [x=59.92690 y=-0.46880 orient=0.06815]
iteration  60
cross_track_err  -0.4687989084376909 , steering  0.04687989084376909
after move,  [x=60.92450 y=-0.39953 orient=0.07050]
iteration  61
cross_track_err  -0.3995314307780973 , steering  0.03995314307780973
after move,  [x=61.92195 y=-0.32810 orient=0.07249]
iteration  62
cross_track_err  -0.3280971146715501 , steering  0.032809711467155014
after move,  [x=62.91926 y=-0.25485 orient=0.07414]
iteration  63
cross_track_err  -0.25484766585998386 , steering  0.025484766585998388
after move,  [x=63.91646 y=-0.18014 orient=0.07541]
iteration  64
cross_track_err  -0.18014441401010117 , steering  0.018014441401010117
after move,  [x=64.91362 y=-0.10481 orient=0.07631]
iteration  65
cross_track_err  -0.10480569580320534 , steering  0.010480569580320535
after move,  [x=65.91071 y=-0.02857 orient=0.07684]
iteration  66
cross_track_err  -0.028568748897535592 , steering  0.0028568748897535596
after move,  [x=66.90776 y=0.04819 orient=0.07698]
iteration  67
cross_track_err  0.04819071006193122 , steering  -0.004819071006193123
after move,  [x=67.90480 y=0.12509 orient=0.07674]
iteration  68
cross_track_err  0.1250925909293464 , steering  -0.012509259092934641
after move,  [x=68.90186 y=0.20175 orient=0.07611]
iteration  69
cross_track_err  0.20175422770061818 , steering  -0.02017542277006182
after move,  [x=69.89900 y=0.27729 orient=0.07510]
iteration  70
cross_track_err  0.27728918822322157 , steering  -0.02772891882232216
after move,  [x=70.89623 y=0.35163 orient=0.07372]
iteration  71
cross_track_err  0.35162965946028635 , steering  -0.035162965946028635
after move,  [x=71.89358 y=0.42440 orient=0.07196]
iteration  72
cross_track_err  0.4244015508074881 , steering  -0.04244015508074881
after move,  [x=72.89107 y=0.49524 orient=0.06983]
iteration  73
cross_track_err  0.49523737091936937 , steering  -0.04952373709193694
after move,  [x=73.88872 y=0.56378 orient=0.06736]
iteration  74
cross_track_err  0.563778032893083 , steering  -0.056377803289308304
after move,  [x=74.88654 y=0.62967 orient=0.06453]
iteration  75
cross_track_err  0.6296746381465255 , steering  -0.06296746381465256
after move,  [x=75.88456 y=0.69259 orient=0.06138]
iteration  76
cross_track_err  0.692590231799727 , steering  -0.0692590231799727
after move,  [x=76.88278 y=0.75220 orient=0.05791]
iteration  77
cross_track_err  0.7522015217123794 , steering  -0.07522015217123795
after move,  [x=77.88121 y=0.80820 orient=0.05414]
iteration  78
cross_track_err  0.808200552657695 , steering  -0.0808200552657695
after move,  [x=78.87986 y=0.86030 orient=0.05009]
iteration  79
cross_track_err  0.8602963264550283 , steering  -0.08602963264550284
after move,  [x=79.87871 y=0.90822 orient=0.04578]
iteration  80
cross_track_err  0.9082163582913836 , steering  -0.09082163582913837
after move,  [x=80.87776 y=0.95171 orient=0.04123]
iteration  81
cross_track_err  0.951708158966909 , steering  -0.09517081589669091
after move,  [x=81.87700 y=0.99054 orient=0.03646]
iteration  82
cross_track_err  0.9905406324471357 , steering  -0.09905406324471358
after move,  [x=82.87643 y=1.02451 orient=0.03149]
iteration  83
cross_track_err  1.0245053779282784 , steering  -0.10245053779282785
after move,  [x=83.87601 y=1.05342 orient=0.02635]
iteration  84
cross_track_err  1.0534178856532321 , steering  -0.10534178856532322
after move,  [x=84.87572 y=1.07712 orient=0.02106]
iteration  85
cross_track_err  1.0771186159778097 , steering  -0.10771186159778097
after move,  [x=85.87555 y=1.09547 orient=0.01565]
iteration  86
cross_track_err  1.0954739516933216 , steering  -0.10954739516933217
after move,  [x=86.87547 y=1.10838 orient=0.01015]
iteration  87
cross_track_err  1.1083770143696086 , steering  -0.11083770143696087
after move,  [x=87.87544 y=1.11575 orient=0.00459]
iteration  88
cross_track_err  1.1157483364819427 , steering  -0.11157483364819427
after move,  [x=88.87544 y=1.11754 orient=6.28217]
iteration  89
cross_track_err  1.1175363823136024 , steering  -0.11175363823136025
after move,  [x=89.87543 y=1.11372 orient=6.27656]
iteration  90
cross_track_err  1.1137179120502196 , steering  -0.11137179120502197
after move,  [x=90.87538 y=1.10430 orient=6.27097]
iteration  91
cross_track_err  1.1042981850700073 , steering  -0.11042981850700073
after move,  [x=91.87527 y=1.08931 orient=6.26543]
iteration  92
cross_track_err  1.0893110001354955 , steering  -0.10893110001354955
after move,  [x=92.87506 y=1.06882 orient=6.25996]
iteration  93
cross_track_err  1.0688185719608043 , steering  -0.10688185719608044
after move,  [x=93.87472 y=1.04291 orient=6.25459]
iteration  94
cross_track_err  1.0429112454048095 , steering  -0.10429112454048095
after move,  [x=94.87423 y=1.01171 orient=6.24936]
iteration  95
cross_track_err  1.011707050273344 , steering  -0.10117070502733441
after move,  [x=95.87357 y=0.97535 orient=6.24428]
iteration  96
cross_track_err  0.9753511013461207 , steering  -0.09753511013461208
after move,  [x=96.87272 y=0.93401 orient=6.23939]
iteration  97
cross_track_err  0.9340148497321934 , steering  -0.09340148497321935
after move,  [x=97.87165 y=0.88790 orient=6.23471]
iteration  98
cross_track_err  0.8878951929552557 , steering  -0.08878951929552557
after move,  [x=98.87037 y=0.83721 orient=6.23026]
iteration  99
cross_track_err  0.8372134522491024 , steering  -0.08372134522491025
after move,  [x=99.86885 y=0.78221 orient=6.22606]
'''
