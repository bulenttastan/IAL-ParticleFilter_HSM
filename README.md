IAL-ParticleFilter_HSM
======================

Intelligent Agents Lab - Particle Filter implementation using Human Steering Model for human steering and obstacle behavior

The main files to run the code:
* particle.m (particle filter using HSM motion model)
* particle_ex.m (similar to particle.m but the motion model is slightly improved)
* particle_robot.m (works along with an iRobot simulator to return a path with detected obstacles from the robot's camera view)
* intel/compare.m (uses real world sensor data and compares voronoi motion against HSM motion)