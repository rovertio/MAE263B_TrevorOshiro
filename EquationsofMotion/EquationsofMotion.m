%% MAE263B HW5: Equations of Motion, Trevor Oshiro

clear all
clc;
clf;

%% Establishing variables

syms L1 L2 m1 m2 real
syms ri ro real
syms t1 t2 real
syms dt1 dt2 real
syms d2t1 d2t2 real

Lmass = 0.5*(2710*((pi*0.05^2) - (pi*0.045^2)));

Ixx1 = (m1/2)*(ri^2 + ro^2);
Iyy1 = (m1/2)*(3*(ri^2 + ro^2) + L1^2);
Izz1 = (m1/2)*(3*(ri^2 + ro^2) + L1^2);

Ixx2 = (m2/2)*(ri^2 + ro^2);
Iyy2 = (m2/2)*(3*(ri^2 + ro^2) + L2^2);
Izz2 = (m2/2)*(3*(ri^2 + ro^2) + L2^2);

mvec = [m1, m2];
thvec = [t1, t2];
dthvec = [dt1, dt2];
d2thvec = [d2t1, d2t2];

syms g real

% Transformation matrices
T01 = [cos(t1), -sin(t1), 0, 0;
    sin(t1), cos(t1), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];
T12 = [cos(t2), -sin(t2), 0, L1;
    sin(t2), cos(t2), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];
T23 = [1, 0, 0, L2;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];
T02 = T01*T12;
T03 = T01*T12*T23;

% Rotational matrices
R01 = T01(1:3,1:3);
R12 = T12(1:3,1:3);
R23 = T23(1:3,1:3);
R02 = T02(1:3,1:3);
R03 = T03(1:3,1:3);

% Position vectors
P01 = T01(1:3,4);
P12 = T12(1:3,4);
P23 = T23(1:3,4);
P02 = T02(1:3,4);
P03 = T03(1:3,4);

%% Jacobian Derivation

syms TL real
l_para = [t1, t2, TL];
l_in = [1, 1, 0];
para1 = [0, 0, 0];         % d, a, alpha
para2 = [0, L1, 0];         % d, a, alpha
para5 = [L2, 0, 0];          % a, alpha, theta

Arm1 = Link('revolute','d',para1(1),'a',para1(2),'alpha',para1(3),'modified');
Arm2 = Link('revolute','d',para2(1),'a',para2(2),'alpha',para2(3),'modified');
Tool = Link('prismatic','a',para5(1),'alpha',para5(2),'theta',para5(3),...
    'modified', 'qlim', [0,0.15]);
% Tool = [rotz(0), [0; 0; TL]; 
%     0, 0, 0, 1];

SCARA_HW5 = SerialLink([Arm1 Arm2 Tool], ...
    'name', 'DH parameters of arm');

% Transformation matrices
% Storing transformation matrices for functions
t_ar = [];
for ii = 1:(size(l_para, 2))
    t_ar(end+1) = ii;
    % Frame to frame transformation matrices
    T_bet.t{ii} = simplify(SCARA_HW5.A([ii], l_para));
    T_base.t{ii} = simplify(SCARA_HW5.A(t_ar, l_para));
end

J_expm = Jmethod_exp(T_base, l_in);
Red_Jac = J_expm(:,1:(size(l_para, 2) - 1));


%% Newton-Euler Method
n=2;

% Angular velocity, linear velocity, force, and moment
wvec.v{1} = [0;0;0];
dwvec.v{1} =[0;0;0];
dvvec.v{1} = [0;g;0];
%dvvec.v{1} = [0;0;g];
dvcvec.v{1} = [];
Fvec.v{1} = [];
Nvec.v{1} = [];

% Rotation and psoition vectors from transformations
Rotvec.i{1} = R01;
Rotvec.i{2} = R12;
Rotvec.i{3} = R23;
Posvec.i{1} = P01;
Posvec.i{2} = P12;
Posvec.i{3} = P23;

% Positon of link center of mass
PosCvec.i{1} = [0.5*L1;0;0];
PosCvec.i{2} = [0.5*L2;0;0];

% Inertia of links
CImat.i{1} = [Ixx1, 0, 0;
    0, Iyy1, 0;
    0, 0, Izz1];
CImat.i{2} = [Ixx2, 0, 0;
    0, Iyy2, 0;
    0, 0, Izz2];

% Outward Iteration
for ll = 1:n
    wvec.v{ll+1} = simplify((Rotvec.i{ll})'*wvec.v{ll} + dthvec(ll)*[0;0;1]);
    dwvec.v{ll+1} = simplify((Rotvec.i{ll})'*dwvec.v{ll} + ...
        cross((Rotvec.i{ll})'*wvec.v{ll}, dthvec(ll)*[0;0;1]) + ...
        d2thvec(ll)*[0;0;1]);
    dvvec.v{ll+1} = simplify((Rotvec.i{ll})'*...
        (cross(dwvec.v{ll}, Posvec.i{ll}) + ...
        cross(wvec.v{ll}, cross(wvec.v{ll}, Posvec.i{ll})) + dvvec.v{ll}));
    dvcvec.v{ll+1} = simplify(cross(dwvec.v{ll+1}, PosCvec.i{ll}) + ...
        cross(wvec.v{ll+1}, cross(wvec.v{ll+1},PosCvec.i{ll})) + ...
        dvvec.v{ll+1});

    Fvec.v{ll} = simplify(mvec(ll)*dvcvec.v{ll+1});
    Nvec.v{ll} = simplify((CImat.i{ll}*dwvec.v{ll+1}) + ...
        cross(wvec.v{ll+1}, (CImat.i{ll}*wvec.v{ll+1})));
end


% Forces and moments at end effector
syms f3x f3y f3z real
syms n3x n3y n3z real
f_in.v{n+1} = [f3x; f3y; f3z];
n_in.v{n+1} = [n3x; n3y; n3z];
% f_in.v{n+1} = [0;0;0];
% n_in.v{n+1} = [0;0;0];
tor_in.v{n} = [];

% Inward Iteration
for qq = n:-1:1
    f_in.v{qq} = (Rotvec.i{qq+1}*f_in.v{qq+1}) + Fvec.v{qq};
    n_in.v{qq} = Nvec.v{qq} + (Rotvec.i{qq+1}*n_in.v{qq+1}) + ...
        cross(PosCvec.i{qq}, Fvec.v{qq}) + ...
        cross(Posvec.i{qq+1}, (Rotvec.i{qq+1}*f_in.v{qq+1}));
    tor_in.v{qq} = simplify((n_in.v{qq}')*[0;0;1]);
end

%% Obtaining equation of motion
sNEuEq = [tor_in.v{1}; tor_in.v{2}];

cTor1 = subs(tor_in.v{1}, [L1 L2 m1 m2 ro ri f3x f3y f3z n3x n3y n3z], ...
    [0.5, 0.5, Lmass, Lmass, 0.05, 0.045, -10, 0, 0, 0, 0, 10]);
cTor2 = subs(tor_in.v{2}, [L1 L2 m1 m2 ro ri f3x f3y f3z n3x n3y n3z], ...
    [0.5, 0.5, Lmass, Lmass, 0.05, 0.045, -10, 0, 0, 0, 0, 10]);
cNEuEq = [cTor1; cTor2];


%% Lagrange method for equations
% Vectors to center of mass
P1c1 = [0.5*L1; 0; 0];
P0c1 = (T01)*[P1c1;1];
P0c1 = simplify(P0c1(1:3));
P2c2 = [0.5*L2; 0; 0];
P0c2 = (T02)*[P2c2;1];
P0c2 = simplify(P0c2(1:3));

% Inertial matrices
% I1c1 = (1/12).*m1.*(L1^2)*[1, 0, 0;
%     0, 1, 0;
%     0, 0, 1];
% I2c2 = (1/12).*m2.*(L2^2)*[1, 0, 0;
%     0, 1, 0;
%     0, 0, 1];
I1c1 = [Ixx1, 0, 0;
    0, Iyy1, 0;
    0, 0, Izz1];
I2c2 = [Ixx2, 0, 0;
    0, Iyy2, 0;
    0, 0, Izz2];
I01 = R01*I1c1*(R01');
I02 = R02*I2c2*(R02');

% Formation of Jacobian Matrices
J0v1 = [(cross(R01(1:3,3), (P0c1 - P01))), [0;0;0]];
J0v1 = simplify(J0v1);
J0w1 = [R01(1:3,3), [0;0;0]];
J0v2 = [(cross(R01(1:3,3), (P0c2 - P01))), (cross(R02(1:3,3), (P0c2 - P02)))];
J0v2 = simplify(J0v2);
J0w2 = [R01(1:3,3), R02(1:3,3)];

% Manipulator matrix
ManMat = simplify((J0v1')*m1*(J0v1) + (J0w1')*I01*(J0w1) + ...
    (J0v2')*m2*(J0v2) + (J0w2')*I02*(J0w2));


% Velocity coupling vector
vecsum = [0];
for ii = 1:2
    for jj = 1:2
        for kk = 1:2
            vect = ((diff(ManMat(ii,jj),thvec(kk)) - ...
                0.5*diff(ManMat(jj,kk),thvec(ii))))*(dthvec(kk)*dthvec(jj));
            vecsum = vect + vecsum;
        end
    end
    cvec.i{ii} = simplify(vecsum);
    vecsum = [0];
end
coupvec = [collect(cvec.i{1}, [dt1, dt2]);
    collect(cvec.i{2}, [dt1, dt2])];

% Gravity Vector
glan = [0; -g; 0];
G1 = simplify((-m1*glan'*J0v1(:,1)) + (-m2*glan'*J0v2(:,1)));
G2 = simplify((-m1*glan'*J0v1(:,2)) + (-m2*glan'*J0v2(:,2)));
Gvec = [G1; G2];

% Terms from external forces
Fhat = [f3x; f3y; f3z; n3x; n3y; n3z];
ext_f = -(Red_Jac' * Fhat);

%% Obtaining Equations of Motion
% Symbolic version of euqation
sLanEq = simplify(ManMat*[d2t1; d2t2] + coupvec + Gvec + ext_f);

% Substitution of values for problem
pcManMat = subs(ManMat, [L1 L2 m1 m2 ro ri], [0.5, 0.5, Lmass, Lmass, 0.05, 0.045])*[d2t1; d2t2];
pcCoupVec = subs(coupvec, [L1 L2 m1 m2 ro ri], [0.5, 0.5, Lmass, Lmass, 0.05, 0.045]);
pcGVec = subs(Gvec, [L1 L2 m1 m2 ro ri], [0.5, 0.5, Lmass, Lmass, 0.05, 0.045]);
pcExtVec = subs(ext_f, [L1 L2 m1 m2 ro ri f3x f3y f3z n3x n3y n3z], ...
    [0.5, 0.5, Lmass, Lmass, 0.05, 0.045, -10, 0, 0, 0, 0, 10]);

% Plugging into final equation of motion for Langrange Method
cLanEq = simplify(pcManMat + pcCoupVec + pcGVec + pcExtVec);


%% Problem 1c

% Developing Trajectory for Arm
% Max acceleration assumed to be 50 
acc = 50;
points2 = [Ikine2RR([0.1, 0], [0.5, 0.5], -1);
    Ikine2RR([0.45, 0], [0.5, 0.5], -1)
    Ikine2RR([0.9, 0], [0.5, 0.5], -1)];
tpts2 = [0, 2, 4];
hold on
subplot(1,2,1)
[Lin_Coeff2a, Para_Coeff2a, time_seg2a] = Joint_blend(points2(:,1)', tpts2, acc);
[fig_plot2a] = Joint_blendplot(Lin_Coeff2a, Para_Coeff2a, time_seg2a, tpts2, 1);
title('Trajectory for theta 1')
xlabel('Time (s)')
ylabel('Position (rad)')

subplot(1,2,2)
[Lin_Coeff2b, Para_Coeff2b, time_seg2b] = Joint_blend(points2(:,2)', tpts2, acc);
[fig_plot2b] = Joint_blendplot(Lin_Coeff2b, Para_Coeff2b, time_seg2b, tpts2, 1);
title('Trajectory for theta 2')
xlabel('Time (s)')
ylabel('Position (rad)')

sgtitle('Plot trajectory for both joints')
hold off

%%

% Obntaining values for joint angle, velocity, and acceleration
nstep=100;
PCo = [2*Para_Coeff2a(:,1), Para_Coeff2a(:,2)];
LCo = [Lin_Coeff2a(:,1)];
[A1jtstep, A1jval] = kinePlot(2*Para_Coeff2a(:,1), zeros(size(Lin_Coeff2a,1)), time_seg2a, tpts2, nstep);
[V1jtstep, V1jval] = kinePlot(PCo, LCo, time_seg2a, tpts2, nstep);
[P1jtstep, P1jval] = kinePlot(Para_Coeff2a, Lin_Coeff2a, time_seg2a, tpts2, nstep);

PCo2 = [2*Para_Coeff2b(:,1), Para_Coeff2b(:,2)];
LCo2 = [Lin_Coeff2b(:,1)];
[A2jtstep, A2jval] = kinePlot(2*Para_Coeff2b(:,1), zeros(size(Lin_Coeff2b,1)), time_seg2b, tpts2, nstep);
[V2jtstep, V2jval] = kinePlot(PCo2, LCo2, time_seg2b, tpts2, nstep);
[P2jtstep, P2jval] = kinePlot(Para_Coeff2b, Lin_Coeff2b, time_seg2b, tpts2, nstep);

JointAcc = [A1jval; A2jval];
JointVel = [V1jval; V2jval];
JointPos = [P1jval; P2jval];

[cor] = FKine(JointPos, [0.5,0.5]);
taskVel = Jcalc(J_expm(1:2,1:2), JointPos, JointVel);
taskAcc = Jcalc(J_expm(1:2,1:2)', JointPos, JointAcc);

%% Plotting joint space values 

figure(2);
hold on
subplot(3,2,1)
plot(P1jtstep, P1jval)
title('Joint one position')
xlabel('Time (s)')
ylabel('Position (rad)')
subplot(3,2,3)
plot(V1jtstep, V1jval)
title('Joint one velocity')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
subplot(3,2,5)
plot(A1jtstep, A1jval)
title('Joint one acceleration')
xlabel('Time (s)')
ylabel('Acceleration (rad/s^2)')

subplot(3,2,2)
plot(P2jtstep, P2jval)
title('Joint two position')
xlabel('Time (s)')
ylabel('Position (rad)')
subplot(3,2,4)
plot(V2jtstep, V2jval)
title('Joint two velocity')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
subplot(3,2,6)
plot(A2jtstep, A2jval)
title('Joint two acceleration')
xlabel('Time (s)')
ylabel('Acceleration (rad/s^2)')
sgtitle('Plot of joint kinematics')
hold off

%% Plotting positiion, velocity, and acceleration in task space

figure(3);
clf;
hold on
subplot(3,1,1)
plot(P2jtstep, cor(1,:), 'LineWidth', 3, 'Color', 'blue')
title('X coordinate vs time')
xlabel('Time (s)')
ylabel('X coordinate (m)')

subplot(3,1,2)
plot(V2jtstep, taskVel(1,:), 'LineWidth', 3, 'Color', 'blue')
title('Velocity vs time')
xlabel('Time (s)')
ylabel('Velocity (m/s)')

subplot(3,1,3)
plot(A2jtstep, taskAcc(1,:), 'LineWidth', 3, 'Color', 'blue')
title('Acceleration vs time')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')

sgtitle('Kinematics in task space')
hold off


%% Plotting torques as a function of time
% Obtaining values for torques from values of positon, velocity and
% acceeration using the derived equations of motion
[Tval1, Tval2] = Tcalc(JointPos, JointVel, JointAcc);

figure(4);
clf;
hold on
subplot(5,1,1)
plot(P2jtstep, Tval1(1,:), 'LineWidth', 3, 'Color', 'blue')
title('Inertial torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,2)
plot(P2jtstep, Tval1(2,:), 'LineWidth', 3, 'Color', 'blue')
title('Centrifugal torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,3)
plot(P2jtstep, Tval1(3,:), 'LineWidth', 3, 'Color', 'blue')
title('Coriolis torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,4)
plot(P2jtstep, Tval1(4,:), 'LineWidth', 3, 'Color', 'blue')
title('Gravitational torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,5)
plot(P2jtstep, Tval1(5,:), 'LineWidth', 3, 'Color', 'blue')
title('Total torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
sgtitle('Plot of torques at joint 1')
hold off


figure(5);
clf;
hold on
subplot(5,1,1)
plot(P2jtstep, Tval2(1,:), 'LineWidth', 3, 'Color', 'blue')
title('Inertial torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,2)
plot(P2jtstep, Tval2(2,:), 'LineWidth', 3, 'Color', 'blue')
title('Centrifugal torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,3)
plot(P2jtstep, Tval2(3,:), 'LineWidth', 3, 'Color', 'blue')
title('Coriolis torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,4)
plot(P2jtstep, Tval2(4,:), 'LineWidth', 3, 'Color', 'blue')
title('Gravitational torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,5)
plot(P2jtstep, Tval2(5,:), 'LineWidth', 3, 'Color', 'blue')
title('Total torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
sgtitle('Plot of torques at joint 2')
hold off

%% Plotting torques as function of time (remove gravity)
% Obtaining values for torques from values of positon, velocity and
% acceeration using the derived equations of motion
[Tgval1, Tgval2] = Tgcalc(JointPos, JointVel, JointAcc);

figure(6);
clf;
hold on
subplot(5,1,1)
plot(P2jtstep, Tgval1(1,:), 'LineWidth', 3, 'Color', 'blue')
title('Inertial torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,2)
plot(P2jtstep, Tgval1(2,:), 'LineWidth', 3, 'Color', 'blue')
title('Centrifugal torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,3)
plot(P2jtstep, Tgval1(3,:), 'LineWidth', 3, 'Color', 'blue')
title('Coriolis torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,4)
plot(P2jtstep, Tgval1(4,:), 'LineWidth', 3, 'Color', 'blue')
title('Gravitational torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,5)
plot(P2jtstep, Tgval1(5,:), 'LineWidth', 3, 'Color', 'blue')
title('Total torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
sgtitle('Plot of torques at joint 1 (gravity on z)')
hold off


figure(7);
clf;
hold on
subplot(5,1,1)
plot(P2jtstep, Tgval2(1,:), 'LineWidth', 3, 'Color', 'blue')
title('Inertial torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,2)
plot(P2jtstep, Tgval2(2,:), 'LineWidth', 3, 'Color', 'blue')
title('Centrifugal torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,3)
plot(P2jtstep, Tgval2(3,:), 'LineWidth', 3, 'Color', 'blue')
title('Coriolis torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,4)
plot(P2jtstep, Tgval2(4,:), 'LineWidth', 3, 'Color', 'blue')
title('Gravitational torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(5,1,5)
plot(P2jtstep, Tgval2(5,:), 'LineWidth', 3, 'Color', 'blue')
title('Total torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
sgtitle('Plot of torques at joint 2 (gravity on z)')
hold off

%% Gravity Ratio plots

figure(7);
clf;
hold on
subplot(2,1,1)
plot(P2jtstep, Tgval2(5,:)./Tval2(5,:), 'LineWidth', 3, 'Color', 'blue')
title('Joint 1 torque ratio')
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(2,1,2)
plot(P2jtstep, Tgval1(5,:)./Tval1(5,:), 'LineWidth', 3, 'Color', 'blue')
title('Joint 2 torque ratio')
xlabel('Time (s)')
ylabel('Torque (Nm)')
sgtitle('Plot of ratio of total torque without gravity vs with gravity')
hold off


%% Helper functions

% Calculations for torques
function [val1, val2] = Tgcalc(th, vel, acc)
    % Joint 1 torque terms
    ine_t1 = @(t2, d2t1, d2t2) (1.2912 + 0.5055*cos(t2))*d2t1 + (0.3929 + 0.2528*cos(t2))*d2t2;
    cen_t1 = @(t2, dt2) -0.2528*sin(t2)*dt2 ^2;
    cor_t1 = @(t2, dt1, dt2) (-0.5055)*sin(t2)*dt1*dt2;
    gra_t1 = @(t1, t2) 10 + (-5*sin(t2));

    % Joint 2 torque terms
    ine_t2 = @(t2, d2t1, d2t2) (0.3929 + 0.2528*cos(t2))*d2t1 + (0.3929)*d2t2;
    cen_t2 = @(t2, dt1) 0.2528*sin(t2)*dt1^2;
    gra_t2 = @(t1, t2) (10);

    val1 = zeros(5,size(th,2));
    val2 = zeros(5,size(th,2));
    for kk = 1:size(th,2)
        val1(1,kk) = ine_t1(th(2,kk), acc(1,kk), acc(2,kk));
        val1(2,kk) = cen_t1(th(2,kk), vel(2,kk));
        val1(3,kk) = cor_t1(th(2,kk), vel(1,kk), vel(2,kk));
        val1(4,kk) = gra_t1(th(1,kk), th(2,kk));
        val1(5,kk) = val1(1,kk) + val1(2,kk) + val1(3,kk) + val1(4,kk);

        val2(1,kk) = ine_t2(th(2,kk), acc(1,kk), acc(2,kk));
        val2(2,kk) = cen_t2(th(2,kk), vel(1,kk));
        val2(3,kk) = 0;
        val2(4,kk) = gra_t2(th(1,kk), th(2,kk));
        val2(5,kk) = val2(1,kk) + val2(2,kk) + val2(3,kk) + val2(4,kk);
    end
end

% Calculations for torques
function [val1, val2] = Tcalc(th, vel, acc)
    % Joint 1 torque terms
    ine_t1 = @(t2, d2t1, d2t2) (1.2912 + 0.5055*cos(t2))*d2t1 + (0.3929 + 0.2528*cos(t2))*d2t2;
    cen_t1 = @(t2, dt2) -0.2528*sin(t2)*dt2^2;
    cor_t1 = @(t2, dt1, dt2) (-0.5055)*sin(t2)*dt1*dt2;
    gra_t1 = @(t1, t2) (1.5165*9.81*cos(t1) + 10) + (-5*sin(t2)) + (0.5055*9.81*cos(t1 + t2));

    % Joint 2 torque terms
    ine_t2 = @(t2, d2t1, d2t2) (0.3929 + 0.2528*cos(t2))*d2t1 + (0.3929)*d2t2;
    cen_t2 = @(t2, dt1) 0.2528*sin(t2)*dt1^2;
    gra_t2 = @(t1, t2) (10) + (0.5055*9.81*cos(t1 + t2));

    val1 = zeros(5,size(th,2));
    val2 = zeros(5,size(th,2));
    for kk = 1:size(th,2)
        val1(1,kk) = ine_t1(th(2,kk), acc(1,kk), acc(2,kk));
        val1(2,kk) = cen_t1(th(2,kk), vel(2,kk));
        val1(3,kk) = cor_t1(th(2,kk), vel(1,kk), vel(2,kk));
        val1(4,kk) = gra_t1(th(1,kk), th(2,kk));
        val1(5,kk) = val1(1,kk) + val1(2,kk) + val1(3,kk) + val1(4,kk);

        val2(1,kk) = ine_t2(th(2,kk), acc(1,kk), acc(2,kk));
        val2(2,kk) = cen_t2(th(2,kk), vel(1,kk));
        val2(3,kk) = 0;
        val2(4,kk) = gra_t2(th(1,kk), th(2,kk));
        val2(5,kk) = val2(1,kk) + val2(2,kk) + val2(3,kk) + val2(4,kk);
    end
end

% Calculations using Jacobian
function [val] = Jcalc(Jac, th, vel)
    syms L1 L2 t1 t2
    val = zeros(2,size(th,2));
    for kk = 1:size(th,2)
        JacVal = subs(Jac, [L1, L2, t1, t2], [0.5, 0.5, th(1,kk), th(2,kk)]);
        val(:,kk) = JacVal*vel(:,kk);
    end
end


% Forward Kinematics function
function [cor] = FKine(th, conf)
    % Takes in a horizontal vector of angles
    cor = zeros(2,size(th,2));
    L1 = conf(1);
    L2 = conf(2);
    for ii = 1:size(th,2)
        cor(1,ii) = L1*cos(th(1,ii)) + L2*cos((th(1,ii) + th(2,ii)));
        cor(2,ii) = L1*sin(th(1,ii)) + L2*sin((th(1,ii) + th(2,ii)));
    end
end


% Function for getting plotting values from the parabolic blend trajectory
function [jtstep, jval] = kinePlot(PCo, LCo, time_seg, tpts, nstep)
    tseg = size(PCo, 1) + size(LCo, 1);
    j1tstep = zeros(tseg,nstep);
    j1vel = zeros(tseg,nstep);
    
    % Plotting first parabola
    j1tstep(1,:) = linspace(0, time_seg(2,1), nstep);
    j1vel(1,:) = polyval(PCo(1,:), j1tstep(1,:));
    
    % Plotting the middle parabolas
    for ii = 2:size(time_seg, 2)-1
        t_para = linspace(0, (time_seg(2,ii)), nstep);
        j1vel(2*(ii-2)+3,:) = polyval(PCo(ii,:),t_para);
        j1tstep(2*(ii-2)+3,:) = linspace(tpts(ii) - 0.5*time_seg(2,ii), ...
            tpts(ii) + 0.5*time_seg(2,ii), nstep);
    end
    % Plotting end parabola
    t_last = linspace(0, time_seg(2,end), nstep);    
    j1vel(tseg,:) = polyval(PCo(end,:), t_last);
    j1tstep(tseg,:) = linspace(tpts(end) - time_seg(2,end), tpts(end), nstep);
       
    % Plotting the linear sections
    tl_off = time_seg(2,1);
    for jj = 1:size(LCo, 1)
        t = linspace(tl_off, (tl_off+time_seg(1,jj)), nstep);
        j1tstep(2*jj,:) = t;
        j1vel(2*jj,:) = polyval(LCo(jj,:),t);
    
        tl_off = tl_off + time_seg(2,jj+1) + time_seg(1,jj);
    end
    
    jval = reshape(j1vel', [1,nstep*tseg]);
    jtstep = reshape(j1tstep', [1,nstep*tseg]);

end

% Function for plotting the overall trajectories generated
function [fig_plot] = Joint_blendplot(Lin_Coeff, Para_Coeff, time_seg, tpts, n)
    %N = 15;
    hold on
    fig_plot = figure(n);
    % Plotting the reference times for blending sections
    c_time = time_seg(2,1);
    xline(c_time, 'LineStyle',':', 'LineWidth', 2, 'DisplayName', 't = ' + string(c_time))
    %f_val = zeros(1,N*(size(Lin_Coeff,1) + size(Para_Coeff,1)));
    for tt = 1:(size(time_seg, 2)-1)
        lin_end = c_time + time_seg(1,tt);
        xline(lin_end, 'LineStyle',':', 'LineWidth', 2, 'DisplayName', 't = ' + string(lin_end))
        %f_val(N*(tt-1):N*tt) = polyval(Para_Coeff(tt,:),linspace(c_time, lin_end, N));
        para_end = c_time + time_seg(1,tt) + time_seg(2,tt+1);
        xline(para_end, 'LineStyle',':', 'LineWidth', 2, 'DisplayName', 't = ' + string(para_end))
        c_time = c_time + time_seg(1,tt) + time_seg(2,tt+1);
    end

    % Plotting the parabolic sections
    % Plotting first parabola
    t = linspace(0, time_seg(2,1));
    plot(t, polyval(Para_Coeff(1,:),t), 'LineWidth', 3, 'Color', ...
            'blue', 'DisplayName', 'Parabola section ' + string(1))
    % Plotting the middle parabolas
    for ii = 2:size(time_seg, 2)-1
        %t = linspace(tp_off, (tp_off+time_seg(2,ii)));
        %t_para = -0.5*time_seg(2,ii-1) + time_seg(1,ii-1) + linspace(0, (time_seg(2,ii)));
        t_para = linspace(0, (time_seg(2,ii)));
        %t_para = 0.5*time_seg(2,ii-1) + time_seg(1,ii-1) + linspace(0, (time_seg(2,ii)));
        para_val = polyval(Para_Coeff(ii,:),t_para);
        t_all = linspace(tpts(ii) - 0.5*time_seg(2,ii), ...
            tpts(ii) + 0.5*time_seg(2,ii));
        plot(t_all, para_val, 'LineWidth', 3, 'Color', ...
            'blue', 'DisplayName', 'Parabola section ' + string(ii))
        %tp_off = tpts(ii+1);
    end
    % Plotting end parabola
    t_last = linspace(0, time_seg(2,end));    
    para_last = polyval(Para_Coeff(end,:), t_last);
    t_all = linspace(tpts(end) - time_seg(2,end), tpts(end));
    plot(t_all, para_last, 'LineWidth', 3, 'Color', ...
            'blue', 'DisplayName', 'Parabola section ' + string(4))


    % Plotting the linear sections
    tl_off = time_seg(2,1);
    for jj = 1:(size(time_seg, 2) - 1)
        t = linspace(tl_off, (tl_off+time_seg(1,jj)));
        plot(t, polyval(Lin_Coeff(jj,:),t), 'LineWidth', 3, 'Color', ...
            'red', 'DisplayName', 'Linear section ' + string(jj))
        tl_off = tl_off + time_seg(2,jj+1) + time_seg(1,jj);
    end
    legend()
    %legend show
    hold off

end