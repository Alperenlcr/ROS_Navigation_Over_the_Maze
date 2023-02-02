# Robots Navigates Over the Maze
Threads are used to get information simultaneously from simulation and navigation algorithm developed by me. Explanations below. 👇
## Designed Algorithm
 The robot searches the maze until it finds the black spot. It records return points above a certain degree as graph nodes. Once it finds the black place graph creation ends. Then the shortest path between white and black squares is found by Dijkstra algorithm. And robot keeps going forward and backward in that path

## Sensors
 There are 2 lidars facing the front of the robot. One of them looks closer and checks for an edge, while it looks a little farther and centers itself on the road. If the robot comes to a turning point, it stops itself, turns 90 degrees to the left, turns to the right and looks for a new path, and if it finds it, it continues from there. There is also a lidar on the left of the robot. The purpose of this is to make the robot go there if there is a road on its left while it is going straight. If there is a road, 50% prefer the left road. There is also a downward-facing RGB camera on the back of the robot. With this camera, it checks whether it comes to the black ground.


## In Runtime
 If the front of the robot is empty and the left turn interrupt does not come, it will go straight until the road ends. While doing that it recentering itself on the road. When the road ends or when the turn left interrupt comes, it stops and turns 90 degrees left. Then it turns right and looks for a new path. When it finds it, it continues from there. Appends the odometry to the graph at each turning point. It checks if it is in the black place during the runtime. If it passes over it, it stops itself.

### 🙌 Final Automation
📽️ Refer this video for watching whole simulation on
<a href="https://youtu.be/ioGkle-YrU8" target="_blank">YouTube.</a>


## How to Run
- First install workspace and robot from https://github.com/Gastd/p3at_tutorial
- After source and catkin_make open gazebo.
```
$ roslaunch hm1 gazebo.launch
```
- Then run the algorithm.
```
$ rosrun hm1 main.py
```