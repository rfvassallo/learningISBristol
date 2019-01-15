

In this lesson we implemented the path planning based on the Probablistic Road Map (PRM) Planner. We used a previous implementation done by Atsushi Sakai and adapted to our problem.

This program will be used to make the robot navigate inside the intelligent space using the built map.

###

* path_planning_PRM.py

You can pass as arguments:

- the name of the map file  
- the granularity for creating the roadmap from the points saved in the map file
- the start point (origin) for the path planning
- the target destination
- the robot radius size
- the number of edges (KNN) from one sampled point used for building the roadmap
- the maximum edge length [m] used for avoiding collisions
- if you want to see the animation for the path planning (but this slows down the running time)
  
###

* plot_map.py
  Plot the file given as an argument to the program
 
