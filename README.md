# Trog Navigation

Trog is UCLA's mobile, solar-powered robot, roaming campus to collect data for research related to:
* Reinforcement Learning
* Solar Energy
* ...

This is a ROS package for navigating a mobile robot based on a deep reinforcement learning agent.
The agent observes a LiDAR pointcloud of its environment and should optimally navigate to a final
destination.

## Packages
* drive: a high level state machine for navigation
* trog_agent: the RL agent
* controllers: motor controller interface for mobile robot

## NOTE
* This is a highly volatile repo, as the RL is still experimental and debugging robotic logic while core functionality is not working 100% is a bad idea!

## Contact
Zach Rash <zachrash@ucla.edu>
Brad Squicciarini <bsquicciarini@ucla.edu>
