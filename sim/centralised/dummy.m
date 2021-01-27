clear
clc

kappa = 50;
xf = [20; 20];
p = [6; 4];
rho = 2;

syms x1(t) x2(t) z1(t) z2(t)

gS = 1/((z1(t)+14)^2+(z2(t)+16)^2-2^2);
gD = 1/((z1(t)+20)^2+(z2(t)+20)^2-1);

d_gS = [-2*(z1(t)+14)/((z1(t)+14)^2+(z2(t)+16)^2-2)^2
        -2*(z2(t)+16)/((z1(t)+14)^2+(z2(t)+16)^2-2)^2];

d_gD = [-2*(z1(t)+20)/((z1(t)+20)^2+(z2(t)+20)^2-1)^2
        -2*(z2(t)+20)/((z1(t)+20)^2+(z2(t)+20)^2-1)^2];


ode1 = diff(x1(t),t) == -x1(t)*(sqrt(1+gS+gD)+1)-(x1(t)-z1(t));
ode2 = diff(x2(t),t) == -x2(t)*(sqrt(1+gS+gD)+1)-(x2(t)-z2(t));
ode3 = diff(z1(t),t) == -kappa*x1(t)^2*x2(t)^2/(2*sqrt(1+gS+gD)) * (d_gS(1) + d_gD(1))-(x1(t)-z1(t));
ode4 = diff(z2(t),t) == -kappa*x1(t)^2*x2(t)^2/(2*sqrt(1+gS+gD)) * (d_gS(2) + d_gD(2))-(x2(t)-z2(t));


[V,S] = odeToVectorField(ode1,ode2,ode3,ode4);
F = matlabFunction(V,'vars',{'t','Y'});

tspan = [0, 2];
y0 = [0; 0; 20; -40];

[t, sol] = ode45(F, tspan, y0);

x_pos_sol = sol(:,2);
y_pos_sol = sol(:,1);


plot(x_pos_sol,y_pos_sol);
hold on
circle(p(1),p(2),rho)
%axis([0,20,0,20])
