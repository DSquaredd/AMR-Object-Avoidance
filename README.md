# AMR-Object-Avoidance

Simulation for a path following and object avoiding autonomous mobile robot using Coppelia Sim program. 

Part of a class project for Introduction to Autonomous Systems. 
 

![image](https://user-images.githubusercontent.com/115327300/194683655-1215fa45-1064-4ab1-a1c2-d488e0fb45bb.png)

OBJECTIVE
Have the autonomous robot navigate the path while detecting and avoiding obstacles in both clockwise and counterclockwise directions. Each time the robot avoids the obstacle, it should get back on track and continue traversing the track in the same direction. After a first attempt to finish the track, the autonomous robot has to rotate 180 degrees and complete the circuit in the opposite direction.

Furthermore,the obstacles will be moved each attempt, so the autonomous robot needs to comprehend and react to different situations. By all these characteristics, the autonomous robot will react to different environments and complete its goal: “Complete the track in both directions.”

Discussion:

  At the beginning of the project, the robot succeeded in detecting and avoiding obstacles until it approached the third obstacle, where it attempted to squeeze through between the obstacle and the end of the map. Once it breaches through, it loses sight of the path or falls off the map. The robot can navigate the path and avoid obstacles if the obstacle's initial position is altered. When altering the robot’s turn path, the robot is unable to complete its course and instead loops around the second and third obstacles. The robot fails to follow the path after avoiding obstacles.
  
	While analyzing and fixing these difficulties, the robot managed to complete the whole track in 3 minutes and 10 seconds; however, that was only one direction and it was not able to rotate by itself. Another problem at this moment was how the robot reacted to the obstacle while going in the opposite direction: the robot was not able to detect the obstacles fast enough to avoid them. 

Results:
	By using the try and error method, a couple of days later, the robot was able to complete the track (back and forth) in less than 3 minutes; this was possible by speeding up the reaction of the robot while detecting the obstacles, writing some code that would rotate the robot, and fixing some parts of the code. Therefore, when trying to change the position of the obstacles and looking at the results, the robot succeeds in completing its goal.
