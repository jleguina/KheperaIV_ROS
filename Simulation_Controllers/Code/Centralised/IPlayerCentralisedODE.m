clear
clc

% TODO - define x_tilda(t) & zeta(t) as VECTORS!!!

syms x_tilda(t) zeta(t) x_tilda1 zeta1 x_tilda2 zeta2

x_tilda(t) = [x_tilda1; x_tilda2];
zeta(t) = [zeta1; zeta2];

% u = sym('u');
% x_tilda = sym('x_tilda');
% zeta = sym('zeta');

%% Problem formulation


% Defined in paper
xf = [20; 20];
p = [6; 4];
rho = 2;
k = 1;
r = 1;
R = eye(2);


% NOT defined in paper
alpha = 1; % alpha>0
betaS = 1; % betaS>0
betaD = 1; % betaD>0
gamma = 1; % gamma>0
E = eye(2);
kappa = 5 ; % kappa > kappa_star

%% ODEs
gS = 1./(eNorm(zeta+xf-p, E).^2 - rho^2);
gD = 1/(eNorm((zeta+xf) - 0*(zeta+xf), E)^2 - r^2); % NOTE - 0 multiplying second part of norm since there is only 1 player now

diff_gS_zeta = jacobian(gS, [zeta1; zeta2]).';
diff_gD_zeta = jacobian(gD, [zeta1; zeta2]).';

syms x_tilda(t) zeta(t) x_tilda1(t) zeta1(t) x_tilda2(t) zeta2(t)
x_tilda(t) = [x_tilda1; x_tilda2];
zeta(t) = [zeta1; zeta2];

ode1 = diff(x_tilda(t),t) == -x_tilda*(sqrt(alpha + betaS*gS + betaD*gD) + gamma) - (x_tilda-zeta);
ode2 = diff(zeta(t),t) == -kappa*(x_tilda.'*x_tilda)/(2*sqrt(alpha + betaS*gS.' + betaD*gD.')) * ...
    (betaS*diff_gS_zeta + betaD*diff_gD_zeta) - R*(x_tilda-zeta);

[V,S] = odeToVectorField(ode1,ode2);
F = matlabFunction(V,'vars',{'t','Y'});

tspan = [0, 2];

y0 = [0; 0; 40; 40];

[t, sol] = ode45(F, tspan, y0);

x_pos_sol = sol(:,2);
y_pos_sol = sol(:,1);

plot(x_pos_sol,y_pos_sol);
circle(p(1),p(2),rho)
axis([0,20,0,20])

