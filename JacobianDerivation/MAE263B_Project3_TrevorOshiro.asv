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

syms th1 th2 th3 d4 real
syms d1 a2 a3 real
l_para = [th1, th2, th3, d4];
l_in = [1,1,1,0];
para1 = [d1, 0, 0];      % d, a, alpha
para2 = [0, a2, 0];     % d, a, alpha
para3 = [0, a3, 0];      % d, a, alpha
para4 = [0, 0, 0];          % a, alpha, theta

Arm1 = Link('revolute','d',para1(1),'a',para1(2),'alpha',para1(3),'modified');
Arm2 = Link('revolute','d',para2(1),'a',para2(2),'alpha',para2(3),'modified');
Arm3 = Link('revolute','d',para3(1),'a',para3(2),'alpha',para3(3),'modified');
Arm4 = Link('prismatic','a',para4(1),'alpha',para4(2),'theta',para4(3),...
    'modified', 'qlim', [0,0.15]);
Arm_tool = transl(0,0,0.17);

SCARA_HW2 = SerialLink([Arm1 Arm2 Arm3 Arm4], 'tool', Arm_tool,...
    'name', 'DH parameters of arm')

% Transformation matrices
% Storing transformation matrices for functions
t_ar = [];
for ii = 1:size(l_para, 2)
    t_ar(end+1) = ii;
    % Frame to frame transformation matrices
    T_bet.t{ii} = simplify(SCARA_HW2.A([ii], l_para));
    T_base.t{ii} = simplify(SCARA_HW2.A(t_ar, l_para));
end

%% Execution of Jacobian Derivation methods
% Explicit method for derivation
fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Explicit method of derivation: Jmethod_exp.m file')
fprintf('\n')
J_expm = Jmethod_exp(T_base, l_in)
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

% Velocity propagation method for derivation
fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Velocity Propagation method of derivation: Jmethod_vp.m file')
fprintf('\n')
[J_vp, w_bet, v_bet, Rbet, Pbet] = Jmethod_vp(T_bet, l_in, [0;0;0], [0;0;0]);
J_vp
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

