%% MAE263B Homework 3 Code Trevor Oshiro

clear all;
clc;

%% Problem 1: Creation of robot parameters

fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('DH parameters of the arm')
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

syms th1 th2 th3 d4 TL real
syms d1 a2 a3 real
l_para = [th1, th2, th3, d4, TL];
l_in = [1, 1, 1, 0, 0];
para1 = [d1, 0, 0];         % d, a, alpha
para2 = [0, a2, 0];         % d, a, alpha
para3 = [0, a3, 0];         % d, a, alpha
para4 = [0, 0, 0];          % a, alpha, theta
para5 = [0, 0, 0];          % a, alpha, theta

Arm1 = Link('revolute','d',para1(1),'a',para1(2),'alpha',para1(3),'modified');
Arm2 = Link('revolute','d',para2(1),'a',para2(2),'alpha',para2(3),'modified');
Arm3 = Link('revolute','d',para3(1),'a',para3(2),'alpha',para3(3),'modified');
Arm4 = Link('prismatic','a',para4(1),'alpha',para4(2),'theta',para4(3),...
    'modified', 'qlim', [0,0.15]);
% TL is a constant for the end effector but treated as a variable for the
% purposes of this project
Tool = Link('prismatic','a',para5(1),'alpha',para5(2),'theta',para5(3),...
    'modified', 'qlim', [0,0.15]);
% Tool = [rotz(0), [0; 0; TL]; 
%     0, 0, 0, 1];

SCARA_HW2 = SerialLink([Arm1 Arm2 Arm3 Arm4, Tool], ...
    'name', 'DH parameters of arm')

% Transformation matrices
% Storing transformation matrices for functions
t_ar = [];
for ii = 1:(size(l_para, 2))
    t_ar(end+1) = ii;
    % Frame to frame transformation matrices
    T_bet.t{ii} = simplify(SCARA_HW2.A([ii], l_para));
    T_base.t{ii} = simplify(SCARA_HW2.A(t_ar, l_para));
end
% Adding the end effector frame
% T_bet.t{size(l_para, 2) + 1} = Tool;                % Last frame to EE
% T_base.t{size(l_para, 2) + 1} = ...
%     simplify((SCARA_HW2.A(t_ar, l_para))*Tool);     % Base to EE

%% Execution of Jacobian Derivation methods
% Explicit method for derivation
fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Explicit method of derivation: Jmethod_exp.m file')
fprintf('\n')
J_expm = Jmethod_exp(T_base, l_in);
J_expm(:,1:(size(l_para, 2) - 1))
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

% Velocity propagation method for derivation
fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Velocity Propagation method of derivation: Jmethod_vp.m file')
fprintf('\n')
[J_vp, w_bet, v_bet, Rbet, Pbet] = Jmethod_vp(T_bet, l_in, [0;0;0], [0;0;0]);
J_vp(:,1:(size(l_para, 2) - 1))
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

% Force Torque propagation method for derivation
fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Force Torque Propagation method of derivation: Jmethod_ft.m file')
fprintf('\n')
[J_ft, t_bet, f_bet, Rbet, Pbet] = Jmethod_ft(T_bet, l_in);
J_ft(:,1:(size(l_para, 2) - 1))
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

%% Demonstration of Singularities in jacobian matrix
% Reduced jacobian matrix for the system
fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Reduced jacobian matrix:')
fprintf('\n')
Red_Jac = J_expm(:,1:(size(l_para, 2) - 1));
Red_Jac([4,5],:) = [];
Red_Jac
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

% Calculation of the determinent of the jacobian
red_det = det(Red_Jac)

% Examples of singularity orientations:
fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Stretched along x axis:')
fprintf('\n')
subs(Red_Jac, [a2, a3, th1, th2], [0.325, 0.225, 0, 0])
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Stretched along y axis:')
fprintf('\n')
subs(Red_Jac, [a2, a3, th1, th2], [0.325, 0.225, sym(pi)/2, 0])
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Stretched when 60 degrees and force along arm axis:')
fprintf('\n')
s30 = subs(Red_Jac, [a2, a3, th1, th2], [0.325, 0.225, sym(pi)/3, 0])
fprintf('\n')
fprintf('Row reduction of the euqations for force')
fprintf('\n')
rref([[(sqrt(3)/2); 0.5], s30(1:2,1:2)])
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

