# Polynomial Playground (making PTG work)

Before you begin the final project we'd like to give you a chance to play around with cost function design and weight tweaking. In the Python code you will be provided, you will have a working Polynomial Trajectory Generator. You can see it in action by doing the following:

## Getting Started
Download the project code by clicking TrajectoryExercise2 at the bottom of this page.
From the project's directory, run python evaluate_ptg.py. You should see a plot similar to the one below. This plot shows the s (x-axis) and d(y-axis) trajectories followed by a vehicle in traffic (red) and a self driving car (blue).
![img.png](img.png)


## Goal
In this situation, the self driving car was trying to get behind the target vehicle, but the cost functions it was using weren't weighted appropriately and so it didn't behave as expected.

## Fixing the Problem(s)
There are 5 files in the provided code. You'll probably want to start by modifying cost function weights in ptg.py but may also want to add cost functions of your own. As a bonus challenge try implementing this same code in C++.

## File Descriptions
ptg.py - The primary code for generating a polynomial trajectory for some constraints. This is also where weights are assigned to cost functions. Adjusting these weights (and possibly adding new cost functions), can have a big effect on vehicle behavior.

cost_functions.py - This file contains many cost functions which are used in ptg.py when selecting the best trajectory. Some cost functions aren't yet implemented...

evaluate_ptg.py - This file sets a start state, goal, and traffic conditions and runs the PTG code. Feel free to modify the goal, add traffic, etc... to test your vehicle's trajectory generation ability.

constants.py - constants like speed limit, vehicle size, etc...

helpers.py - helper functions used by other files.

## Supporting Materials

Trajectoryexercise2
https://video.udacity-data.com/topher/2017/October/59d5bfcb_trajectoryexercise2/trajectoryexercise2.zip

Trajectoryexercise2_python3
https://video.udacity-data.com/topher/2017/December/5a3ea459_trajectoryexercise2-python3/trajectoryexercise2-python3.zip