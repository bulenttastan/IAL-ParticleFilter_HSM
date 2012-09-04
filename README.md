IAL-ParticleFilter_HSM
======================

Intelligent Agents Lab - Particle Filter implementation using Human Steering Model for human steering and obstacle behavior

The main files to run the code:
* particle.m (particle filter using HSM motion model)
* particle_ex.m (similar to particle.m but the motion model is slightly improved)

The following files are part of the journal paper [Leveraging Human Behavior Models to Predict Paths in Indoor Environments](http://www.sciencedirect.com/science/article/pii/S1574119211000289)
* particle_robot.m (works along with an iRobot simulator to return a path with detected obstacles from the robot's camera view)
* intel/compare.m (uses real world sensor data and compares voronoi motion against HSM motion)