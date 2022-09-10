clc;
clear all;
%rosinit
%sonarmsg = receive(sonarSub);


[velPub, velMsg] = rospublisher("/cmd_vel", "DataFormat", "struct");
%sonarscan = rosmessage('sensor_msgs/PointCloud');
pause(2)

vfh = controllerVFH;
vfh.UseLidarScan = true;
mcl.UseLidarScan = true;
vfh.DistanceLimits = [0.05 1];
vfh.RobotRadius = 0.6;
vfh.MinTurningRadius = 0.5;
vfh.SafetyDistance = 0.3;

rate = rateControl(2);
targetDir = 0; % Move forward

%Ruuning the loop for 25 seconds

while rate.TotalElapsedTime <25
    
sonarSub = rossubscriber("/scan","DataFormat","struct");
sonarScan = receive(sonarSub);

sonarscan = rosmessage('sensor_msgs/LaserScan');
sonardata = sonarSub.LatestMessage; % Extracting sonar data from the subscriber 

	% Get laser scan data

    y = [sonardata.points.y]; % Get the x co-ordinates from the sonar subcriber data
    x = [sonardata.points.x]; % Get the x co-ordinates from the sonar subcriber data
    %y = [sonarscan.points.y];
    %x = [sonarscan.points.x];
    angles= double(atan(y./x)); % calculation of angles
    ranges = double(sqrt(x.^2+y.^2));%calculation of ranges
    
    scan = lidarScan(ranges,angles); %lidarscan object created using angles and ranges
    plot(scan)
	% Call VFH object to computer steering direction
	steerDir = vfh(scan,targetDir);
    show(vfh)
	% Calculate velocities
	if ~isnan(steerDir) % If steering direction is valid
		desiredV = 0.2;
		w = exampleHelperComputeAngularVelocity(steerDir,0.2);
        
	else % Stop and search for valid direction
		desiredV = 0.0;
		w = 0.5;
	end

	% Assign and send velocity commands
	velMsg.linear.x = desiredV;
	velMsg.angular.z = w;
	velPub.send(velMsg);
end