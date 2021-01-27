clear
clc
close all

syms x_tilda1 x_tilda2 x_tilda3 x_tilda4...
    zeta1 zeta2 zeta3 zeta4...
    x1 x2 x3 x4

x_tilda = [x_tilda1; x_tilda2];
zeta = [zeta1; zeta2];
x = [x1; x2];

%% Problem formulation
N = 1;

% xf = [30; 0; -30; 0];
% x0 = [-30; 0; 30; 0];
% zeta0 = [60; -5; -5; -1];

x0 = [0; 0];
xf = [20; 8];

zeta0 = [-25; -45];

% zeta0 = [10; -40; 0; 0];

tspan = [0, 15];

x_tilda0 = x0 - xf;

p1 = [6; 4];
rho1 = 2;

p2 = [15; 2];
rho2 = 2;

k = 1;
r = 6;
R = eye(2);

alpha = 1; % alpha>0
betaS = 1; % betaS>0
gamma = 1; % gamma>0
E = eye(1);
RE = eye(2);
kappa = 1; % kappa > kappa_star


%% Barrier functions
gS = 1./(eNorm(zeta+xf-p1, E).^2 - (rho1+r)^2);

gS = 100000

diff_gS_zeta = jacobian(gS, zeta);
diff_sum = betaS*diff_gS_zeta;


% %% Construct P and V to check HJ inequality later
% P = sym([]);
% 
% P = sqrt(alpha+betaS*gS)*eye(2);
% 
% V = 0.5.* (x_tilda.'*P*x_tilda + eNorm(x_tilda-zeta,RE).^2);
% diff_v_x= jacobian(V, x_tilda);
% diff_v_zeta = jacobian(V, zeta);
% 
% % Check if zeta0 conditions are valid
% sym_vars = symvar(P);
% P_check = double(subs(P,sym_vars,zeta0.'));
% if P_check < abs(inf)
%     disp("zeta_0 valid");
% else
%     error("zeta_0 NOT valid");
% end
% 
% % Construct B matrices
% B = eye(2);
% 
% % Construct q function
% q = (alpha+betaS*gS) * (x_tilda.'*x_tilda);
% 

%% ODEs
syms x_tilda1(t) x_tilda2(t)...
    zeta1(t) zeta2(t)

x_tilda = [x_tilda1(t); x_tilda2(t)];
zeta = [zeta1(t); zeta2(t)];


summation = x_tilda-zeta;


root = sqrt(alpha + betaS*gS) + gamma;
ode1 = diff(x_tilda,t) == -x_tilda.*root - summation;

ode2sum = x_tilda.'*x_tilda/(2*root) * ...
    diff_sum.' - R*(x_tilda-zeta);


zeta_dot = -kappa*sum(ode2sum,2);
ode2 = diff(zeta,t) == zeta_dot;

symvar(ode1)
symvar(ode2)

[L,S] = odeToVectorField(ode1,ode2);
F = matlabFunction(L,'vars',{'t','Y'});

y0 = [x_tilda0; zeta0];
[t, sol] = ode45(F, tspan, y0);


% %% Construct HJ inequalities
% sum_HJ = diff_v_x*(B*B.')*diff_v_x.'
% 
% HJ = 0.5*diff_v_x*(B*B.')*diff_v_x.' + ...
%         0.5*q - sum_HJ + diff_v_zeta * zeta_dot;
% 
% HJ = HJ.';
% 
% x_tilda1 = sol(:,1);
% x_tilda2 = sol(:,2);
% zeta1 = sol(:,3);
% zeta2 = sol(:,4);
% 
% sym_vars = symvar(HJ);
% 
% for k = 1:length(sol)
%     HJ_subs(:,k) = double(subs(HJ,sym_vars,...
%         [0, x_tilda1(k), x_tilda2(k), zeta1(k), zeta2(k)]));
% 
%     progress = k*100 / length(sol);
%     if HJ_subs(:,k)>0
%         error("HJ inquality not satisfied. Progress: %f", progress);
%         break
%     else
%         fprintf("HJ inquality satisfied. Progress: %f", progress);
%         fprintf('\n')
%     end
% end


%% Plotting
LW = 2;

sol_x = sol(:,1:2) + xf.';

x_pos_sol1 = sol_x(:,1);
y_pos_sol1 = sol_x(:,2);

plot(x_pos_sol1,y_pos_sol1,'r',"LineWidth",LW);
hold on
circle(p1(1),p1(2),rho1,'k');
circle(p2(1),p2(2),rho2,'k');
% circle(8.474,1.349,1,'b');
% circle(2.655,6.915,1,'r');
grid on


title("Centralised Simulation",'Interpreter','latex')
xlabel("x-coordinate",'Interpreter','latex')
ylabel("y-coordinate",'Interpreter','latex')
% axis([-5,25,-5,25]);

% legend("Agent 1", "Agent 2",'Interpreter','latex')
% set(gca, "FontSize", 18)
% set(gca,'TickLabelInterpreter','latex')

% % Crop margin
% ax = gca;
% outerpos = ax.OuterPosition;
% ti = ax.TightInset;
% left = outerpos(1) + ti(1);
% bottom = outerpos(2) + ti(2);
% ax_width = outerpos(3) - ti(1) - ti(3);
% ax_height = outerpos(4) - ti(2) - ti(4);
% ax.Position = [left bottom ax_width ax_height];
%

