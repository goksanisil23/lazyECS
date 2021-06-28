# lazyECS
Simple Entity Component System implementation with minimal robotics application examples.

This project is mainly for learning and exploration purposes. For physics (and collision) system, lazyECS utilizes ReactPhysics3D (https://github.com/DanielChappuis/reactphysics3d).


## Minimal 3D
Minimal lazyECS application with both physics and render only actors that can be moved around kinematically or dynamically.
![](Applications/Minimal3D/minimal_3d.gif)

## Potential Field
Naive implementation of the potential field algorithm where Green capsule represent the goal and Red capsules represent the obstacles. Obstacles have repulsive and goal has attractive force field attached to them, which are calculated based on Normal Distribution, where mean of the distribution is the center of the goal/obstacle location and sigma defines the propagation of the field around the object. When/how this naive implementation suffer from being stuck in local minima is quite dependent on the layout of obstacles and the sigma of the distribution.

![](Applications/PotentialField/potential_field.gif)

