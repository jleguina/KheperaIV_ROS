clear
clc
close all

path = [3 3];

namespace = '/robot1';
plot_flag = 0;

% rosshutdown;
% rosinit('http://localhost:11311');

%status = system('cd ~/catkin_ws && roslaunch kh4_gazebo kh4.launch','-echo');


% parpool(3)
parfor i = 1:3
    if i == 1
        path = [3 3
            -1 0];
        rosinit('http://localhost:11311');
        followTrajectory(path, '/robot1', plot_flag)
        rosshutdown;
    elseif i == 2
        path = [4 4
            0 0];
        rosinit('http://localhost:11311');
        followTrajectory(path, '/robot2', plot_flag)
        rosshutdown;
    elseif i == 3
        path = [5 5
            1 0];
        rosinit('http://localhost:11311');
        followTrajectory(path, '/robot2', plot_flag)
        rosshutdown;
    end
end

rosshutdown
