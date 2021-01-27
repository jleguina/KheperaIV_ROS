clear 
clc
close all

sim('PIDkh4_controller',25)

figure
hold on
plot(x_est,y_est,'r')
plot(x_ref,y_ref,'--k')

err = mean(sqrt((x_est-x_ref).^2 + (y_est-y_ref).^2));


