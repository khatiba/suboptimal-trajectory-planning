# suboptimal-trajectory-planning
Sub-optimal trajectory planning algorithm to exploit key features of the optimal solutions but achieve higher performance and account for modeling errors.

## Requirements
* MATLAB 2011 or later

## Setup
* Execute the Main.m file to start the simulation.
* The bank profile is parameterized into P(downrange), P(crossrange), P(final altitude).
* Initial guesses for these parameters between 0-1 are in the comments.
* Min and max downranges are predetermined from GPOPS.
* State holds properties of the system, ie: bank angle, flight path angle, altitude etc.
* Newtons Method in optimization is used in favor of the steepest descent method because it factors in the curvature of the surface.
As a result, convergence is quickly achieved using this adaptive step. Steepest descent with a constant "large" step size leads to very slow and subpar convergence.
Smaller step sizes are not practical as the optimization approaches the minimum, convergence expoentially gets slower.
* The cost function has been proven previously to be convex and therefore has a global minumum.

** Refer to the optimal control simulations for details on the parameters and limitations of those solutions: (https://github.com/khatiba/gpops-optimal-trajectories)
