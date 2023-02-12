# Additional Resources on Path Planning

Nice work reaching the end of the path planning content! While you still have the project left to do here, we're also providing some additional resources and recent research on the topic that you can come back to if you have time later on.

Reading research papers is a great way to get exposure to the latest and greatest in the field, as well as expand your learning. However, just like the project ahead, it's often best to learn by doing - if you find a paper that really excites you, try to implement it (or even something better) yourself!

## Optional Reading
All of these are completely optional reading - you could spend hours reading through the entirety of these! We suggest moving onto the project first so you have what you’ve learned fresh on your mind, before coming back to check these out.

We've categorized these papers to hopefully help you narrow down which ones might be of interest, as well including their Abstract section, which summarizes the paper.

# Indoors
Intention-Net: Integrating Planning and Deep Learning for Goal-Directed Autonomous Navigation by S. W. Gao, et. al.
https://arxiv.org/abs/1710.05627

Abstract: How can a delivery robot navigate reliably to a destination in a new office building, with minimal prior information? To tackle this challenge, this paper introduces a two-level hierarchical approach, which integrates model-free deep learning and model-based path planning. At the low level, a neural-network motion controller, called the intention-net, is trained end-to-end to provide robust local navigation. The intention-net maps images from a single monocular camera and "intentions" directly to robot controls. At the high level, a path planner uses a crude map, e.g., a 2-D floor plan, to compute a path from the robot's current location to the goal. The planned path provides intentions to the intention-net. Preliminary experiments suggest that the learned motion controller is robust against perceptual uncertainty and by integrating with a path planner, it generalizes effectively to new environments and goals.

# City Navigation
Learning to Navigate in Cities Without a Map by P. Mirowski, et. al.
https://arxiv.org/abs/1804.00168

Abstract: Navigating through unstructured environments is a basic capability of intelligent creatures, and thus is of fundamental interest in the study and development of artificial intelligence. Long-range navigation is a complex cognitive task that relies on developing an internal representation of space, grounded by recognizable landmarks and robust visual processing, that can simultaneously support continuous self-localization ("I am here") and a representation of the goal ("I am going there"). Building upon recent research that applies deep reinforcement learning to maze navigation problems, we present an end-to-end deep reinforcement learning approach that can be applied on a city scale. [...] We present an interactive navigation environment that uses Google StreetView for its photographic content and worldwide coverage, and demonstrate that our learning method allows agents to learn to navigate multiple cities and to traverse to target destinations that may be kilometers away. [...]

# Intersections
A Look at Motion Planning for Autonomous Vehicles at an Intersection by S. Krishnan, et. al.
https://arxiv.org/abs/1806.07834

Abstract: Autonomous Vehicles are currently being tested in a variety of scenarios. As we move towards Autonomous Vehicles, how should intersections look? To answer that question, we break down an intersection management into the different conundrums and scenarios involved in the trajectory planning and current approaches to solve them. Then, a brief analysis of current works in autonomous intersection is conducted. With a critical eye, we try to delve into the discrepancies of existing solutions while presenting some critical and important factors that have been addressed. Furthermore, open issues that have to be addressed are also emphasized. We also try to answer the question of how to benchmark intersection management algorithms by providing some factors that impact autonomous navigation at intersection.

# Planning in Traffic with Deep Reinforcement Learning
DeepTraffic: Crowdsourced Hyperparameter Tuning of Deep Reinforcement Learning Systems for Multi-Agent Dense Traffic Navigation by L. Fridman, J. Terwilliger and B. Jenik
https://arxiv.org/abs/1801.02805

Abstract: We present a traffic simulation named DeepTraffic where the planning systems for a subset of the vehicles are handled by a neural network as part of a model-free, off-policy reinforcement learning process. The primary goal of DeepTraffic is to make the hands-on study of deep reinforcement learning accessible to thousands of students, educators, and researchers in order to inspire and fuel the exploration and evaluation of deep Q-learning network variants and hyperparameter configurations through large-scale, open competition. This paper investigates the crowd-sourced hyperparameter tuning of the policy network that resulted from the first iteration of the DeepTraffic competition where thousands of participants actively searched through the hyperparameter space.

