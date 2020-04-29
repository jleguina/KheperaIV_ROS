clear
clc
close all

path = [3 3];

namespace = '/robot1';
plot_flag = 0;

% rosshutdown;
% rosinit('http://localhost:11311');

%status = system('cd ~/catkin_ws && roslaunch kh4_gazebo kh4.launch','-echo');


% parpool(2)
parfor i = 1:2
    if i == 1
        rosinit('http://localhost:11311');
        followTrajectory(path, '/robot1', plot_flag)
        rosshutdown;
    else
        rosinit('http://localhost:11311');
        followTrajectory(path, '/robot2', plot_flag)
        rosshutdown;
    end
end

rosshutdown
