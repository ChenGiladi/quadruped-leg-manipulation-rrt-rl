load("parkingMap.mat");
resolution = 3;
map = binaryOccupancyMap(map,resolution);

show(map)
title("Parking Lot Map")
startPose = [2 9 0];
goalPose = [27 18 pi/2];

show(map)
hold on
quiver(startPose(1),startPose(2),cos(startPose(3)),sin(startPose(3)),2,...
       color=[0 0.75 0.23],LineWidth=2,...
       Marker="o",MarkerFaceColor=[0 0.75 0.23],MarkerSize=5,...
       DisplayName="Start Pose",ShowArrowHead="off");
quiver(goalPose(1),goalPose(2),cos(goalPose(3)),sin(goalPose(3)),2, ...
       color=[1 0 0],LineWidth=2,...
       Marker='o',MarkerFaceColor=[1 0 0],MarkerSize=5,...
       DisplayName="Goal Pose",ShowArrowHead="off");
legend(Location="southeast");
title("Start and Goal Poses in Parking Lot Map")
hold off
%% Path Planning
% Create a |validatorOccupancyMap| state validator using the |stateSpaceSE2| 
% definition. Specify the map and the distance for interpolating and validating 
% path segments.

validator = validatorOccupancyMap(stateSpaceSE2,map=map);
validator.ValidationDistance = 0.1;
%% 
% Initialize the |plannerHybridAStar| object with the state validator object. 
% Specify the |MinTurningRadius| and |MotionPrimitiveLength| properties of the 
% planner.

planner = plannerHybridAStar(validator,MinTurningRadius=3,MotionPrimitiveLength=4);
%% 
% Set default random number for repeatability.

rng("default");
%% 
% Plan a path from the start pose to the goal pose.

refPath = plan(planner,startPose,goalPose);
path = refPath.States;
%% 
% Visualize the planned path.

show(planner,Tree="off");
legend(Location="southeast");
hold off
%% 
% Visualize the orientation of the vehicle.

plot(rad2deg(refPath.States(:,3)));
title("Orientation of Vehicle Along the Path in degrees")


options = optimizePathOptions;
options.MinTurningRadius = 3; % meters
options.MaxVelocity = 5; % m/s
options.MaxAcceleration = 1; % m/s/s
options.ReferenceDeltaTime = 0.1; % second

separationBetweenStates = 0.2; % meters
numStates = refPath.pathLength/separationBetweenStates;
options.MaxPathStates = round(numStates);
options.ObstacleSafetyMargin = 2; % meters
options.ObstacleInclusionDistance = 0.75; % meters
options.ObstacleCutOffDistance = 2.5; %i meters
options.NumIteration = 4; 
options.MaxSolverIteration = 15;
options.WeightTime = 10;
options.WeightSmoothness = 1000;
options.WeightMinTurningRadius = 10;
options.WeightVelocity = 10;
options.WeightObstacles = 50;
[optimizedPath,kineticInfo] = optimizePath(path,map,options);
drivingDir = sign(kineticInfo.Velocity);
%% 
% Visualize the optimized path.

show(planner,Tree="off");
hold on
forwardMotion = optimizedPath(drivingDir==1,:);
reverseMotion = optimizedPath(drivingDir==-1,:);
quiver(forwardMotion(:,1),forwardMotion(:,2),cos(forwardMotion(:,3)),sin(forwardMotion(:,3)),...
    0.1,Color=[0 0.45 0.74],LineWidth=1,DisplayName="Optimized Forward Path");
quiver(reverseMotion(:,1),reverseMotion(:,2),cos(reverseMotion(:,3)),sin(reverseMotion(:,3)),...
    0.1,Color=[0.47 0.68 0.19],LineWidth=1,DisplayName="Optimized Reverse Path");
legend(Location="southeast");
title("Planned Path and Optimized Path")
hold off
show(planner,Tree="off")
hold on
quiver(forwardMotion(:,1),forwardMotion(:,2),cos(forwardMotion(:,3)),sin(forwardMotion(:,3)),...
    0.1,Color=[0 0.45 0.74],LineWidth=1,DisplayName="Previous Optimized Forward Path");
quiver(reverseMotion(:,1),reverseMotion(:,2),cos(reverseMotion(:,3)),sin(reverseMotion(:,3)),...
    0.1,Color=[0.47 0.68 0.19],LineWidth=1,DisplayName="Previous Optimized Reverse Path");
[optimizedPath,kineticInfo] = optimizePath(path,map,options);
%% 
% Finally plot the new optimized path.

drivingDir = sign(kineticInfo.Velocity);

forwardMotion = optimizedPath(drivingDir==1,:);
reverseMotion = optimizedPath(drivingDir==-1,:);

quiver(forwardMotion(:,1),forwardMotion(:,2),cos(forwardMotion(:,3)),sin(forwardMotion(:,3)),...
    0.1,Color=[0.3 0.75 0.93],LineWidth=1,DisplayName="Optimized Forward Path");
quiver(reverseMotion(:,1),reverseMotion(:,2),cos(reverseMotion(:,3)),sin(reverseMotion(:,3)),...
    0.1,Color=[0.85 0.33 0.1],LineWidth=1,DisplayName="Optimized Reverse Path");

legend(Location="southeast");
title("Previous and Updated Optimized Path")

hold off
