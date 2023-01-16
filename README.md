# cs4501-project
This repository was created by Zach Wang and Max McCullough. 

We used ros to simulate a drone in a random maze. This drone uses its LIDAR scanners to take in information about its environment. 
Based on this data, it can determine its surroundings and populate an occupancy grid. It can also detect doors. It uses a custom algorithm to determine its 
movement, preferring to move towards doors and the goal. Once it has reached a door, the algorithm opens the door and continues navigating the maze. 
