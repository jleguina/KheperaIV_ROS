clear
clc
close all

t0 = 0;
tf = 25;
dt = 0.01;
numitem = 9;

t = linspace(0,20.95, 100)';
x = 0.3*sin(0.3*t);
y = 0.3*cos(0.3*t)-0.3;
plot(x,y)

% x = [ 0, 0.1, 0.2, 0.5, 1, 1.5, 2, 2.5, 3]';
% y = [0, 0.05, 0.2, 0.5, 1, 1, 1, 0.5, 0]';


time = linspace(t0, tf, length(t(1):dt:t(end))).';
T_trajectory = (t(1):dt:t(end)).';

%% X interp
X_curve_fit = fit(t, x,'cubicinterp');

X_trajectory = X_curve_fit(T_trajectory);

[X_dt,X_ddt] = differentiate(X_curve_fit,T_trajectory);

%% Y interp
Y_curve_fit = fit(t, y,'cubicinterp');

Y_trajectory = Y_curve_fit(T_trajectory);

[Y_dt,Y_ddt] = differentiate(Y_curve_fit,T_trajectory);

% plot(X_trajectory,Y_trajectory)
%% Theta & Omega
vel = sqrt(Y_dt.^2 + X_dt.^2);

theta = atan2(Y_dt, X_dt);
theta = theta - theta(1)

theta_curve_fit = fit(T_trajectory, theta,'cubicinterp');

[theta_dt,theta_ddt] = differentiate(theta_curve_fit,T_trajectory);


%% Processing
X_trajectory_ts = timeseries(X_trajectory, T_trajectory);
Y_trajectory_ts = timeseries(Y_trajectory, T_trajectory);

v = sqrt(Y_dt.^2 + X_dt.^2);
omega = (X_dt.*Y_ddt - Y_dt.*X_ddt)./v.^2

X_dt_ts = timeseries(X_dt, T_trajectory);
X_ddt_ts = timeseries(X_ddt, T_trajectory);

Y_dt_ts = timeseries(Y_dt, T_trajectory);
Y_ddt_ts = timeseries(Y_ddt, T_trajectory);

theta_ts = timeseries(theta, T_trajectory);
omega_ts = timeseries(omega, T_trajectory);


% omega_ts = timeseries(theta_dt - theta_dt(1), T_trajectory);

% plot(theta_ts)
%% Simulation

% sim('PIDkh4_controller_traj',tf)
% 
figure
hold on
plot(x_est,y_est,'r')
plot(x_ref,y_ref,'--k')
% 
% err = mean(sqrt((x_est-x_ref).^2 + (y_est-y_ref).^2));




