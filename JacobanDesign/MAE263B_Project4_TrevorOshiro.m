%% MAE263B Homework 4 Code Trevor Oshiro

clear all;
clc;
clf;

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
Con = [Con1; 
    Con2;
    Con3];

% Parameter Definitions
%[L1, L2] = deal(Con2(1), Con2(2));
l_para = [th1, th2, TL];
l_in = [1, 1, 0];
para1 = [0, 0, 0];         % d, a, alpha
para2 = [0, L1, 0];         % d, a, alpha
para3 = [L2, 0, 0];         % a, alpha, theta

Arm1 = Link('revolute','d',para1(1),'a',para1(2),'alpha',para1(3),'modified');
Arm2 = Link('revolute','d',para2(1),'a',para2(2),'alpha',para2(3),'modified');
Tool = Link('prismatic','a',para3(1),'alpha',para3(2),'theta',para3(3),...
    'modified', 'qlim', [0,0.75]);

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

s_pt = 0;
e_pt = 1;
N = 11;
xpts = linspace(s_pt, e_pt, N);
ypts = 0;
Jac_mat = J_expm;

em_area = zeros(3,N);
ef_area = zeros(3,N);

% Iterate over configurations
for ii = 1:3
    % Set current configuration
    Con_use = Con(ii,:);
    % Create serial manipulator object
    [robot, ~, ~] = serialCreate(Con_use);

    % Iterate over the points
    for jj = 1:N
        xpts(jj)
        % Calculate the IK
        [th, err] = Ikine2RR([xpts(jj),ypts], Con_use, -1);
        th
        if err == 1
        else
            figure(1);
            % Plotting for the manipulation space
            clf;
            hold on
            grid on
            daspect([1,1,1])
            xlim([-1.5,1.5])
            ylim([-1.5,1.5])
            testm = manJ(Jac_mat, l_con, Con_use, l_para, th);
            robot.vellipse([th,0], '2d')
            m_area = manplot(Con_use, th, testm.vec, testm.val, -1);
            m_name = strcat('Con', string(ii), '_ManEllipse_', 'x', string(xpts(jj)), '.png');
            em_area(ii,jj) = m_area;
            % text(-1,-1,strcat('Area: ', em_area))
            title(strcat('Con', string(ii), 'ManEllipse: ', 'x', string(xpts(jj))))
            saveas(figure(1), strcat(pwd, '/ManEll/', m_name))
            hold off
            
            if det(testm.m) == 0
            else
                % Plotting for force-torque space
                clf;
                hold on
                grid on
                daspect([1,1,1])
                xlim([-7,7])
                ylim([-7,7])
                testft = ftJ(Jac_mat, l_con, Con_use, l_para, th);
                robot.fellipse([th,0], '2d')
                f_area = manplot(Con_use, th, testft.vec, testft.val, -1);
                f_name = strcat('Con', string(ii), '_FTEllipse_', 'x', string(xpts(jj)), '.png');
                % text(-6,-6,strcat('Area: ', ef_area))
                ef_area(ii,jj) = f_area;
                title(strcat('Con', string(ii), 'FTEllipse: ', 'x', string(xpts(jj))))
                saveas(figure(1), strcat(pwd, '/FTEll/', f_name))
                hold off
            end

        end
    end
end

fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Configuration 1: Best Performance')
fprintf('\n')
[m1,i1] = max(em_area(1,:));
con1_xbest = (i1-1)*0.1
con1_area = m1
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Configuration 2: Best Performance')
fprintf('\n')
[m2,i2] = max(em_area(2,:));
con2_xbest = (i2-1)*0.1
con2_area = m2
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Configuration 3: Best Performance')
fprintf('\n')
[m3,i3] = max(em_area(3,:));
con3_xbest = (i3-1)*0.1
con3_area = m3
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

%% Design - Arm Optimization

row = 1;
col = 1;
err1 = 0;
err2 = 0;
goal = zeros(50);
% cm1 = cell(50,50,15,11);
cm1 = zeros(50,50,15,11);
N = 10;
l = linspace(10,100,N);
alpha = linspace(0,pi/2,N);

for l1 = 10:10:500
    for l2 = 10:10:500
        % We want the selection of l1 and l2 to be within
        % The workspace
        [~, err1] = Ikine2RR([200, 50], [l1, l2], -1);
        [~, err2] = Ikine2RR([340, 50], [l1, l2], -1);
        [~, err3] = Ikine2RR([200, 0], [l1, l2], -1);
        if (err1 + err2 + err3) == 0                    % Fill this out
            % Calculate the goal at this configurtaion using
            % calGoal function. 
            [K,C] = calGoal(J_expm, l1,l2);
            
            % Store that into a matrix
            % Also store the K 
            cm1(row, col, :, :) = K;
            goal(row, col) = C;
            fprintf('\n')
            fprintf(strcat("---include ", "L1: ", string(l1), ...
                "and L2: ", string(l2)));            
        end
        col = col + 1;
        err1 = 0;
        err2 = 0;
        fprintf('\n')
        fprintf(strcat("discard ", "L1: ", string(l1), ...
            "and L2: ", string(l2)));
    end
    row = row + 1;
end

%%

[K, C] = calGoal(J_expm, 500, 500);



%% Helper Functions

function [K,C] = calGoal(Jac, l1,l2)
    syms L1 L2 th1 th2 TL real
    l_con = [L1, L2];
    l_para = [th1, th2, TL];
    % This function will calculate the goal function C
    % Define and initialize parameters
    L3 = l1^3 + l2^3;
    ki_sum = 0;
    % K = zeros(1,(11*15));
    % ii = 0;
    % jj = 1;
    K = [];
    % Initialize kmin to be very large such that we 
    % Can keep updating this values as long as there
    % Is a smaller value pops out
    ki_min = 20000; 
    % We want to cover all workspace in x and y direction
    for Px = linspace(200,340,15)
        for Py = linspace(-50,50,11)
            % Call our IK solution function to solve inverse kinematic
            [th, ~] = Ikine2RR([Px, Py], [l1, l2], -1); 
            % Calculate Jacobian with our IK solution angles
            manJJ = manJ(Jac, l_con, [l1, l2], l_para, th);
            % Extract all eigen valeus from the matrix
            eigvals = diag(manJJ.val);
            % Get max and min value
            lambda_min = [double(min(eigvals))];
            lambda_max = [double(max(eigvals))];
            % Calculate ki
            ki = sqrt(lambda_min) / sqrt(lambda_max);
            % Calculate sum of ki
            ki_sum = ki_sum + ki;
            % Compare wether this ki is the smallest among all 
            % Workspace
            % if ki < ki_min
            %     ki_min = ki;
            % end
            ki_min = min(ki_min, ki);
            % Record the corresponding location of Ki within
            % The workspace
            %K(11*ii + jj) = ki;
            K = [K, ki];
            % jj = jj + 1;
        end
        % ii = ii + 1;
        % jj = 1;
    end
    % Calculate goal function C
    C = max([l1,l2])*(ki_sum)*(ki_min)/L3;
    % Reshape K to match the size of our workspace
    K = reshape(K,11,15)';
end


% Serial Link Robot Creator
function [robot, T_base, T_bet] = serialCreate(Con)
    % Parameter Definitions
    syms th1 th2 TL real
    [L1, L2] = deal(Con(1), Con(2));
    l_para = [th1, th2, TL];
    para1 = [0, 0, 0];         % d, a, alpha
    para2 = [0, L1, 0];         % d, a, alpha
    para3 = [L2, 0, 0];         % a, alpha, theta
    
    Arm1 = Link('revolute','d',para1(1),'a',para1(2),'alpha',para1(3),'modified');
    Arm2 = Link('revolute','d',para2(1),'a',para2(2),'alpha',para2(3),'modified');
    % TL is a constant for the end effector but treated as a variable for the
    % purposes of this project
    Tool = Link('prismatic','a',para3(1),'alpha',para3(2),'theta',para3(3),...
        'modified', 'qlim', [0,0.75]);
    % Tool = [rotz(0), [0; 0; TL]; 
    %     0, 0, 0, 1];
    
    robot = SerialLink([Arm1 Arm2 Tool], ...
        'name', 'DH parameters of arm');
    
    % Transformation matrices
    % Storing transformation matrices for functions
    t_ar = [];
    for ii = 1:(size(l_para, 2))
        t_ar(end+1) = ii;
        % Frame to frame transformation matrices
        T_bet.t{ii} = simplify(robot.A([ii], l_para));
        T_base.t{ii} = simplify(robot.A(t_ar, l_para));
    end
    % Adjusting for end effector frame:
    T_bet.t{size(l_para, 2)}.t(3) = 0;
    T_base.t{size(l_para, 2)}.t(3) = 0;
end

% Matrix calculations
% Calculate manipulability matrix
function manJJ = manJ(Jac, l_con, con, l_para, th)   
    submanJ = subs(Jac, [l_para(1:(size(l_para, 2)-1)), l_con], [th, con]);
    manJ = submanJ*submanJ';
    [Bvec,Bval] = eig(manJ);
    manJJ.m = manJ;
    manJJ.vec = Bvec;
    manJJ.val = Bval;
end
% Calculate the force torque mapping matrix
function ftJJ = ftJ(Jac, l_con, con, l_para, th)
    symJac = (inv((Jac*Jac')))';
    ftJ = subs(symJac, [l_para(1:(size(l_para, 2)-1)), l_con], [th, con]);
    [Bvec,Bval] = eig(ftJ);
    ftJJ.m = ftJ;
    ftJJ.vec = Bvec;
    ftJJ.val = Bval;
end


% Plotting functions
% Plotting robot manipulator in x-y plane
function e_area = manplot(con, th, evec, eval, elbow)
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

    % Calculation of ellipse area
    a_axis = sqrt((yvec12 - yvec11)^2 + (xvec12 - xvec11)^2);
    b_axis = sqrt((yvec22 - yvec21)^2 + (xvec22 - xvec21)^2);
    e_area = double(3.14*a_axis*b_axis);
    
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
