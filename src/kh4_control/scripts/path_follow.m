clear
clc
close all

%% Parameters
robotRadius = 0.14;
LinearVelocity = 0.4; % m/s (desired)
AngularVelocity = 3; % rad/s (maximum)
LookAhead = LinearVelocity; % >= LinearVelocity!
goalError = 0.05; % m (tolerance)

%% Establish ROS connection
% https://uk.mathworks.com/help/robotics/examples/path-following-for-differential-drive-robot.html
rosshutdown;
rosinit('http://localhost:11311');
sub = rossubscriber('/kh4_diff_drive_controller/odom');
[pub,cmd_vel_msg] = rospublisher('/kh4_diff_drive_controller/cmd_vel','geometry_msgs/Twist');

%% Define Waypoints
odom_msg = receive(sub); % Get data from ROS odom
robot_pos = [odom_msg.Pose.Pose.Position.X, odom_msg.Pose.Pose.Position.Y];
robot_rotation = quat2eul([odom_msg.Pose.Pose.Orientation.X, odom_msg.Pose.Pose.Orientation.Y,...
    odom_msg.Pose.Pose.Orientation.Z, odom_msg.Pose.Pose.Orientation.W]);
robot_orientation = robot_rotation(3);

% Define a set of waypoints for the desired path for the robot
path = [robot_pos;
    0.5000  0.2500
    0.3125  0.4375
    1.3125  2.0625
    1.8125  2.1875
    2.9375  1.4375
    0.5000  0];

% Set the current location and the goal location of the robot as defined
% by the path.
robotInitialLocation = path(1,:);
robotGoal = path(end,:);

% Assume an initial robot orientation
% It is the angle between the robot heading and the positive X-axis,
% measured counterclockwise.
initialOrientation = robot_orientation;


% Define the current pose for the robot [x y theta]
robotCurrentPose = [robotInitialLocation initialOrientation]';

%% Create a Kinematic Robot Model
% Initialize the robot model and assign an initial pose
% The simulated robot has kinematic equations for the motion of a
% two-wheeled differential drive robot. The inputs to this simulated robot
% are linear and angular velocities.

robot = differentialDriveKinematics("TrackWidth", robotRadius, "VehicleInputs", "VehicleSpeedHeadingRate");

%% Define the Path Following Controller
% Based on the path defined above and a robot motion model,
% you need a path following controller to drive the robot along the path.
% Create the path following controller using the controllerPurePursuit object.
controller = controllerPurePursuit;

% Use the path defined above to set the desired waypoints for the controller
controller.Waypoints = path;

% Set the path following controller parameters.
controller.DesiredLinearVelocity = LinearVelocity;

% The maximum angular velocity acts as a saturation limit for
controller.MaxAngularVelocity = AngularVelocity;

% As a general rule, the lookahead distance should be larger than
% the desired linear velocity for a smooth path. The robot might cut
% corners when the lookahead distance is large. In contrast, a small
% lookahead distance can result in an unstable path following behavior.
% A value of 0.5 m was chosen for this example.
controller.LookaheadDistance = LookAhead;


%% Using the Path Following Controller, Drive the Robot over the Desired Waypoints
goalRadius = goalError;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v_x, omega] = controller(robotCurrentPose);
    
    if distanceToGoal <= 10*goalRadius
        controller.DesiredLinearVelocity = LinearVelocity * distanceToGoal/(10*goalRadius);
    end
    
    % Get the robot's velocity using controller inputs
    %vel = derivative(robot, robotCurrentPose, [v omega]);
    cmd_vel_msg.Linear.X = v_x;
    cmd_vel_msg.Angular.Z = omega;
    send(pub,cmd_vel_msg);
    
    % Update the current pose
    odom_msg = receive(sub); % Get data from ROS odom
    robot_pos = [odom_msg.Pose.Pose.Position.X, odom_msg.Pose.Pose.Position.Y];
    robot_rotation = quat2eul([odom_msg.Pose.Pose.Orientation.X, odom_msg.Pose.Pose.Orientation.Y,...
        odom_msg.Pose.Pose.Orientation.Z, odom_msg.Pose.Pose.Orientation.W]);
    robot_orientation = robot_rotation(3);
    
    robotCurrentPose = [robot_pos'; robot_orientation];
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 4])
    ylim([0 4])
    
    waitfor(vizRate);
end

rosshutdown