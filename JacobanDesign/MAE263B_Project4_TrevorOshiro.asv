%% MAE263B Homework 4 Code Trevor Oshiro

clear all;
clc;

%% Problem 1: Jacobian Ellipsoid

fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('DH parameters of 2R manipulator')
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

% Variable Definitions
syms th1 th2 real
syms L1 L2 TL real

% Configuration Definitions (L1, L2)
l_con = [L1, L2];
Con1 = [0.25, 0.75];
Con2 = [0.5, 0.5];
Con3 = [0.75, 0.25];

% Parameter Definitions
[L1, L2] = deal(Con2(1), Con2(2));
l_para = [th1, th2, TL];
l_in = [1, 1, 0];
para1 = [0, 0, 0];         % d, a, alpha
para2 = [0, L1, 0];         % d, a, alpha
para3 = [L2, 0, 0];         % a, alpha, theta
[L1, L2] = deal(Con1);


Arm1 = Link('revolute','d',para1(1),'a',para1(2),'alpha',para1(3),'modified');
Arm2 = Link('revolute','d',para2(1),'a',para2(2),'alpha',para2(3),'modified');
% TL is a constant for the end effector but treated as a variable for the
% purposes of this project
Tool = Link('prismatic','a',para3(1),'alpha',para3(2),'theta',para3(3),...
    'modified', 'qlim', [0,0.75]);
% Tool = [rotz(0), [0; 0; TL]; 
%     0, 0, 0, 1];

SCARA_HW4 = SerialLink([Arm1 Arm2 Tool], ...
    'name', 'DH parameters of arm')

% Transformation matrices
% Storing transformation matrices for functions
t_ar = [];
for ii = 1:(size(l_para, 2))
    t_ar(end+1) = ii;
    % Frame to frame transformation matrices
    T_bet.t{ii} = simplify(SCARA_HW4.A([ii], l_para));
    T_base.t{ii} = simplify(SCARA_HW4.A(t_ar, l_para));
end
% Adjusting for end effector frame:
T_bet.t{size(l_para, 2)}.t(3) = 0;
T_base.t{size(l_para, 2)}.t(3) = 0;

%% Execution of Jacobian Derivation methods
% Explicit method for derivation
fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Jacobian within base frame')
fprintf('\n')
J_expm = Jmethod_exp(T_base, l_in);
J_expm = J_expm(:,1:(size(l_para, 2) - 1));      % Only considering impact of revolutes
J_expm = J_expm(1:2, :)                          % Dependence on only revolute
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

% Velocity propagation method for derivation
fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Jacobian within EE frame')
fprintf('\n')
[J_vp, w_bet, v_bet, Rbet, Pbet] = Jmethod_vpEE(T_bet, l_in, [0;0;0], [0;0;0]);
J_vpm = J_vp(:,1:(size(l_para, 2) - 1));         % Only considering impact of revolutes
J_vpm = J_vpm(1:2, :)                            % Dependence on only revolute
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

%% Iterating over coordinates for configruatiions
clf;
hold on
grid on
daspect([1,1,1])
xlim([-1.5,1.5])
ylim([-1.5,1.5])
th = [-pi/3,-2*pi/3];
test = manJ(J_expm, l_con, [0.5, 0.5], l_para, th);
SCARA_HW4.vellipse([th,0], '2d')
manplot(Con2, th, test.vec, test.val, -1)
hold off


%%
clf;

[Bvec,Bval] = eig(manJ(J_expm, l_con, [0.5, 0.5], l_para, [pi/4,pi/4]))
Bvec1 = Bvec(:,1).*sqrt(Bval(1,1))/norm(Bvec(:,1))
Bvec2 = Bvec(:,2).*sqrt(Bval(2,2))/norm(Bvec(:,2))
% [EEvec,EEval] = eig(manJ(J_vpm, l_con, [0.5, 0.5], l_para, [pi/4,pi/4]));
% EEvec1 = EEvec(:,1).*EEval(1,1)/norm(EEvec(:,1))
% EEvec2 = EEvec(:,2).*EEval(2,2)/norm(EEvec(:,2))
hold on
grid on
daspect([1,1,1])
xlim([-1.5,1.5])
ylim([-1.5,1.5])
th = [pi/4,pi/4];
evec = [Bvec1, Bvec2];
SCARA_HW4.vellipse([th,0], '2d')
manplot(Con2, th, evec, -1)
hold off



%% Helper Functions

% Matrix calculations
% Calculate manipulability matrix
function manJJ = manJ(Jac, l_con, con, l_para, th)
    symJac = Jac*Jac';
    manJ = subs(symJac, [l_para(1:(size(l_para, 2)-1)), l_con], [th, con]);
    [Bvec,Bval] = eig(manJ);
    manJJ.m = manJ;
    manJJ.vec = Bvec;
    manJJ.val = Bval;
end
% Calculate the force torque mapping matrix
function ftJJ = ftJ(Jac, l_con, con, l_para, th)
    symJac = (Jac*Jac')';
    ftJJ = subs(symJac, [l_para(1:(size(l_para, 2)-1)), l_con], [th, con]);
end


% Plotting functions
% Plotting robot manipulator in x-y plane
function manplot(con, th, evec, eval, elbow)
    L1 = con(1);
    L2 = con(2);
    t1 = th(1);
    if elbow == 1
        t2 = th(2);
    else
        t2 = -th(2);
    end
    % Location of elbow
    xe = L1*cos(t1);
    ye = L1*sin(t1);
    % Location of EE
    xEE = xe + L2*cos(t2 - t1);
    yEE = ye - L2*sin(t2 - t1);

    % Calculations for eigen vectors
    vec1 = evec(:,1).*sqrt(eval(1,1))/norm(evec(:,1));
    vec2 = evec(:,2).*sqrt(eval(2,2))/norm(evec(:,2));

    % Coordinates for eigen vector 1
    xvec11 = xEE - vec1(1);
    yvec11 = yEE - vec1(2);
    xvec12 = xEE + vec1(1);
    yvec12 = yEE + vec1(2);
    % Coordinates for eigen vector 2
    xvec21 = xEE - vec2(1);
    yvec21 = yEE - vec2(2);
    xvec22 = xEE + vec2(1);
    yvec22 = yEE + vec2(2);
    
    % Plotting manipulator geometry
    plot([0, xe, xEE], [0, ye, yEE], 'LineWidth', 3, 'Color', [0,0,0])
    % Plotting eigen vector 1
    h1 = plot([xvec11, xvec12], [yvec11, yvec12], 'LineStyle', ':', 'LineWidth', 5,...
        'DisplayName', 'Axis: Eigenvector 1');
    % Plotting eigen vector 2
    h2 = plot([xvec21, xvec22], [yvec21, yvec22], 'LineStyle', ':', 'LineWidth', 5,...
        'DisplayName', 'Axis: Eigenvector 2');
    legend([h1, h2]);
end
