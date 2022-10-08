function sysCall_init()
	-- variable that will store handles to each sonar sensor
	usensors={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}
    
	-- this variables stores info about whether the sonar sensors detected something.
	-- 0 means nothing was detected, 1 means something was detected
	res = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}--{0,0,0,0,0,0,0,0}
    
	-- this variable stores the distance of an object measured by the sonar sensors
	dist = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    
	-- associate each usensors element with one of the sonar sensors
	for i=1,16,1 do
    	usensors[i]=sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor"..i)
	end
    
	-- variables corresponding to each wheel's motor
	motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor")
	motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    
	-- the 3 vision sensors placed at the front of the robot
	leftSensor=sim.getObjectHandle("LeftSensor")
	middleSensor=sim.getObjectHandle("MiddleSensor")
	rightSensor=sim.getObjectHandle("RightSensor")
    
	maxDetectionDist= 0.39--0.35 --0.28--0.238
	v0=2.1
    
 	-- variables to keep track of laps and robot direction
	initialPosition = GetObjectPosition(sim.handle_self)
	turningAround = false
	turnedAround = false
	totalLaps = 0
    
	interWheelDistance = 7.0 -- this value was eye-balled.
	arcLength = interWheelDistance * 2.9
	timeToTurn90Degrees = arcLength / v0
   	 
	frontSideSensorDist = -1;
	rearSideSensorDist = -1;
	frontSideSensorRes = -1;
	rearSideSensorRes = -1;
    
	isAvoidingObstacle = false;
	isGettingBackOnTrack = false;    
    
	maxDistanceFromStartPosition = 0.75
end


function sysCall_cleanup()
 
end

function sysCall_actuation()
    
	currentPosition = GetObjectPosition(sim.handle_self)  
	x2 = currentPosition["position"]["x"]
	y2 = currentPosition["position"]["y"]
	startPositionDistance = math.sqrt((x2 - initialPosition["position"]["x"])^2 + (y2 - initialPosition["position"]["y"])^2)    

	-- These line read sensors from robot.
	sensorReading={false,false,false}
	sensorReading[1]=(sim.readVisionSensor(leftSensor))
	sensorReading[2]=(sim.readVisionSensor(middleSensor))
	sensorReading[3]=(sim.readVisionSensor(rightSensor))
    
	-- check if one lap has been completed
	if(totalLaps < 1 and not turnedAround and
    	turningAround == false and sim.getSimulationTime() > 60 and
    	startPositionDistance < maxDistanceFromStartPosition)
    	then
    	turningAround = true
    	previousTime = sim.getSimulationTime()
    	vLeft = 0
    	vRight = 0
	end
    
	-- rotate in place a few degrees without reacting to sensor readings
	if (totalLaps < 1 and turningAround == true
    	and (sim.getSimulationTime() - previousTime) < timeToTurn90Degrees)
    	then
    	-- rotate rightward
    	--print("turning around")
    	vLeft = v0/4
    	vRight = -v0/4
    	turnedAround = true
	-- continue rotating in place until one of the sensors detects the track
	elseif (totalLaps < 1 and turningAround == true
    	and ((sim.getSimulationTime() - previousTime) or
    	(sensorReading[1] == true and sensorReading[2] == true and
    	sensorReading[3] == true)))
    	then
    	vLeft = v0
    	vRight = 0
    	turnedAround = true
    	totalLaps = totalLaps + 1
	-- increment lap counter once the robot has turned around
	elseif(totalLaps < 1 and turningAround == false and turnedAround) then
    	turnedAround = false
    	totalLaps = totalLaps + 1
	-- follow the path
	else
    	follow_course()
	end

	-- Rolling!
	sim.setJointTargetVelocity(motorLeft, vLeft)
	sim.setJointTargetVelocity(motorRight, vRight)
end

function read_sensors()
	-- Read values from all 16 sonar sensors on the robot
	for i=1,16,1 do
    	-- To read, using sim.readProximitySensor for each usensors[i] from initilization.
    	-- "res[i]" stores detect status of a sensor [i]. 0 - undetect, 1 - detect
    	-- "dist[i]" stores the distance from a sensor [i] to detected object. See the example values from slide.
    	-- After the following line, we can get status and value from all sensors in the robot.
    	res[i],dist[i]=sim.readProximitySensor(usensors[i])
    	--print("total laps: " .. totalLaps)

    	-- This below, I wrote to change value from "nil" to 10000 if undetected for calculation.
    	-- If there is no this block, there is a risk if we calculate string and number together.
    	-- Note that, this is just the workarounf.
    	if(dist[i] == nil) then
        	dist[i] = 10000 -- Why 10000?, because the distant read by sensor is 0.xxxx approximatly
    	end
   	 
    	if(totalLaps < 1) then
        	frontSideSensorDist = dist[1];
        	rearSideSensorDist = dist[16];
        	frontSideSensorRes = res[1];
        	rearSideSensorRes = res[16];
    	else
        	frontSideSensorDist = dist[8];
        	rearSideSensorDist = dist[9];
        	frontSideSensorRes = res[8];
        	rearSideSensorRes = res[9];
    	end
	end
end

function follow_course()
	read_sensors()
    
	obstacleDetected = (dist[3] < maxDetectionDist or dist[4] < maxDetectionDist or
                    	dist[5] < maxDetectionDist or dist[6] < maxDetectionDist)
	isOffTrack = (sensorReading[1] == 1 and sensorReading[2] == 1 and sensorReading[3] == 1)
	isOnTrack = (sensorReading[1] == 0 or sensorReading[2] == 0 or sensorReading[3] == 0)
	frontSideDetectingObstacle = (frontSideSensorDist < 1) --(dist[8] < 10000)--(dist[1] < 10000)
	rearSideDetectingObstacle = (rearSideSensorDist < 1) --(dist[8] < 10000)--(dist[1] < 10000)
	allVisionSensorsDetectTrack = (sensorReading[1] == 0 and sensorReading[2] == 0 and sensorReading[3] == 0)

	if(isGettingBackOnTrack and isOffTrack) then
    	-- if the robot attempted getting back on track, got back on but went off again

    	if(totalLaps < 1) then
        	vRight = v0 * 0.05--0.1        	vLeft = v0 --* 0.8
    	else
        	vLeft = v0 * 0.05--0.1
        	vRight = v0 --* 0.8
    	end
	elseif(isGettingBackOnTrack and isOnTrack) then
    	-- robot is back on track
    	isGettingBackOnTrack = false
	elseif(not obstacleDetected and isOnTrack and allVisionSensorsDetectTrack and isAvoidingObstacle) then
        	-- if robot has detected the track for the first time after avoiding the obstacle
        	isAvoidingObstacle = false
        	isGettingBackOnTrack = true
        	if(totalLaps < 1) then
            	vLeft = v0
            	vRight = 0
        	else
            	vLeft = 0
            	vRight = v0
        	end
	elseif(not obstacleDetected and isOnTrack) then
    	-- follow the path
    	vLeft, vRight = w_rolling(sensorReading[1], sensorReading[2], sensorReading[3])
	elseif(obstacleDetected) then -------
    	-- start rotating the robot until the side sensors pick up the obstacle
    	if(totalLaps < 1) then
        	vLeft = v0*2--1
        	vRight = -v0* 0.2--0
    	else
        	vLeft = -v0*0.2--0
        	vRight = v0*2--1
    	end
    	isAvoidingObstacle = true
	elseif(not obstacleDetected and isAvoidingObstacle and isOffTrack and frontSideSensorDist==rearSideSensorDist ) then
    	-- go straight if the robot is off track and both side sensors sense the obstacle
    	-- this is sometimes necessary to make the robot start going around the obstacle
    	vLeft = v0
    	vRight = v0
	elseif(not obstacleDetected and isAvoidingObstacle and isOffTrack and rearSideDetectingObstacle) then
    	if(totalLaps < 1) then
        	vLeft = 0
        	vRight = v0--1.5
    	else
        	vLeft = v0--1.5
        	vRight = 0
    	end
    
	elseif(not obstacleDetected and isAvoidingObstacle and isOffTrack and frontSideDetectingObstacle) then--frontSideDetectingObstacle) then
    	-- speed up rotation of robot so the rear side sensor picks up the obstacle
    	if(totalLaps < 1) then
        	vLeft = 2*v0
    	else
        	vRight = v0*2
    	end
	elseif(sensorReading[1] == -1 and sensorReading[2] == -1 and sensorReading[3] == -1) then
    	-- initialize left and right motors when sim starts (sensors read -1)
    	vLeft = v0
    	vRight = v0
	else
    	isGettingBackOnTrack = true
	end
    
end

-- line following algorithm
function w_rolling(lef_ss, mid_ss, rig_ss)
	if(sensorReading[1] == 0 and sensorReading[2] == 0 and sensorReading[3] == 0) then
    	vLeft = v0 --vLeft
    	vRight = v0 --vRight
	elseif (sensorReading[3] == 0) then -- Turn right
    	vRight = v0 * 0.1
	elseif (sensorReading[1] == 0) then
    	vLeft = v0 * 0.1
	end
    
	return vLeft, vRight
end

function GetObjectPosition(objectName)
	objectHandle=sim.getObjectHandle(objectName)
	Pioneer = -1
	p=sim.getObjectPosition(objectHandle,Pioneer)
	o=sim.getObjectQuaternion(objectHandle,Pioneer)
	return {
  	position={x=p[1],y=p[2],z=p[3]}
	}
end
