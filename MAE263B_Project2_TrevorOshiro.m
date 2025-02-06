%% Trevor Oshiro MAE263B HW2 Code
% Code is made to be run through each section for implementation of the
% project

clear all;
clc;

fprintf('Arm parameters taken with from SCARA Mitsubishi Arm - Model RH-3FRH5515')
fprintf('\n')
fprintf('Gripper parameters taken from Yamaha YRG-4220W')
fprintf('\n')

% Maximum possible acceleration of actuators
acc_max = 50;       % ms^{-1}

%% Definition of the robot
fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Question 1.1: Define the DH parameters of the arm')
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

syms larm1 larm2
para1 = [0.416, 0, 0];      % d, a, alpha
para2 = [0, 0.325, 0];     % d, a, alpha
para3 = [0, 0.225, 0];      % d, a, alpha
para4 = [0, 0, 0];          % a, alpha, theta

Arm1 = Link('revolute','d',para1(1),'a',para1(2),'alpha',para1(3),'modified');
Arm2 = Link('revolute','d',para2(1),'a',para2(2),'alpha',para2(3),'modified');
Arm3 = Link('revolute','d',para3(1),'a',para3(2),'alpha',para3(3),'modified');
Arm4 = Link('prismatic','a',para4(1),'alpha',para4(2),'theta',para4(3),'modified');
Arm_tool = transl(0,0,-0.17);

SCARA_modDH = SerialLink([Arm1 Arm2 Arm3 Arm4], 'tool', Arm_tool,...
    'name', '(Problem 1.1) DH parameters of arm')

fprintf(['Orientation of robot in schematic obtained with editing rest...' ...
    'values for revolute joint'])
fprintf('\n')

fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Question 1.2: Define the Forward kinematics of the arm')
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

syms th1 th2 th3 di3
SCARA_FK = simplify(SCARA_modDH.fkine([th1, th2, th3, di3]))

fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Question 1.2: Define the Inverse kinematics of the arm')
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

fprintf('Derivations of the inverse kinematics can be seen with the report')
fprintf('\n')
fprintf('Simple calculations with IK are shown with the Ikine function execution below')
fprintf('\n')
fprintf('Fully extended along y axis:')
fprintf('\n')
[th1, th2, th3, di3] = Ikine(0,0.255+0.16,0,90,1)
fprintf('\n')
fprintf('Fully extended along x axis:')
fprintf('\n')
[th1, th2, th3, di3] = Ikine(0.255+0.16,0,0,90,1)
fprintf('\n')

fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Question 1.3 Define initial positions of the feeder and target locations')
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

fprintf('(1) Feeder Location')
fprintf('\n')
T_feeder = [rotz(0), [0.155; sqrt(0.2^2 - 0.155^2); 0];
    0,0,0,1]
fprintf('\n')
fprintf('(2) Top Left Corner Location')
fprintf('\n')
T_2 = [rotz(90), [0.2+(0.01/2); 0.09; 0];
    0,0,0,1]
fprintf('\n')
fprintf('(3) Top Right Corner Location')
fprintf('\n')
T_3 = [rotz(0), [0.2+0.01+(0.01/2); 0.09; 0];
    0,0,0,1]
fprintf('\n')
fprintf('(4) Bottom Right Corner Location')
fprintf('\n')
T_4 = [rotz(-90), [0.2+0.01+(0.01/2); 0; 0];
    0,0,0,1]
fprintf('\n')
fprintf('(4) Bottom Left Corner Location')
fprintf('\n')
T_5 = [rotz(180), [0.2+(0.01/2); 0; 0];
    0,0,0,1]

fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Question 1.4 Define Define via points to avoid collisions')
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')

fprintf('\n')
fprintf('Center of board')
fprintf('\n')
T_center = [rotz(0), [0.2+(0.1/2); 0.045; 0];
    0,0,0,1]
fprintf('\n')
fprintf('Middle of two top chips')
fprintf('\n')
T_tmid = [rotz(0), [0.2+(0.1/2); 0.09; 0];
    0,0,0,1]
fprintf('\n')
fprintf('Middle of two right chips')
fprintf('\n')
T_rmid = [rotz(0), [0.2+(0.1-(0.01/2)); 0.045; 0];
    0,0,0,1]
fprintf('\n')
fprintf('Middle of two left chips')
fprintf('\n')
T_lmid = [rotz(0), [0.2+(0.01/2); 0.045; 0];
    0,0,0,1]
fprintf('\n')
fprintf('Middle of two bottom chips')
fprintf('\n')
T_bmid = [rotz(0), [0.2+(0.1/2); 0; 0];
    0,0,0,1]

fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')
fprintf('Question 1.5 Develop a model of the arm from the robotics toolbox')
fprintf('\n')
fprintf('-----------------------------------------------------------------')
fprintf('\n')





%% Section to test functions

clf;
points = [10, 35, 25, 10];
tpts = [0, 2, 3, 6];
acc = 50;
[Lin_Coeff, Para_Coeff, time_seg] = Joint_blend(points, tpts, acc)
[fig_plot] = Joint_blendplot(Lin_Coeff, Para_Coeff, time_seg, tpts)
[th1, th2, th3, di3] = Ikine(0,0.255+0.16,0,90,1)


%% Helper Functions

% Function for generating inverse kinematics
% Angles defined with degrees
% Orientation of claw defined with orient being 0 when facing forward
function [th1, th2, th3, di3] = Ikine(x, y, z, orient, elbow)
    % Solving for theta 2 (elbow)
    c2 = (x^2 + y^2 - 0.255^2 - 0.325^2)/(2*0.255*0.325);
    % Account for perferred orientation of elbow
    if elbow == 1
        s2 = sqrt(1 - c2^2);
    elseif elbow == -1
        s2 = -sqrt(1 - c2^2);
    end
    if isreal(s2)
        th2 = rad2deg(atan2(s2,c2));
        if abs(th2) > 145
            th2 = 0;
            fprintf('Out of elbow range')
        end
    else
        th2 = 0;
    end

    % Solving for z extension of prismatic joint
    di3 = 0.2460 - z;

    % Solving for theta 1 (shoulder)
    deag = (y*(0.255*cos(th2) + 0.325)) - (x*0.255*sin(th2));
    cgdf = (x*(0.255*cos(th2) + 0.325)) - (y*(-0.255*sin(th2)));
    th1 = rad2deg(atan2(deag, cgdf));
    if abs(th1) > 170
        th1 = 0;
        fprintf('Out of shoulder range')
    end

    % Solving for gripper orientation
    th3 = orient + th1 - th2;
end


% Function for generating plots of trajectory: linear + parabolic blend
function [fig_plot] = Joint_blendplot(Lin_Coeff, Para_Coeff, time_seg, tpts)
    hold on
    fig_plot = figure(1);
    % Plotting the reference times for blending sections
    c_time = time_seg(2,1);
    xline(c_time, 'LineStyle',':', 'LineWidth', 2, 'DisplayName', 't = ' + string(c_time))
    for tt = 1:(size(time_seg, 2)-1)
        lin_end = c_time + time_seg(1,tt);
        xline(lin_end, 'LineStyle',':', 'LineWidth', 2, 'DisplayName', 't = ' + string(lin_end))
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
    legend show
    hold off

end

% Joint space linear parabolic blend
function [Lin_Coeff, Para_Coeff, time_seg] = Joint_blend(points, tpts, acc)
    % points contains angle values
    % tpts is the time steps for each point
    % acc is the acceleration maximum of the joint applied

    % ------------------------------------------------------------------
    % CALCULATIONS OF TIME SEGMENTS
    % store times calculated from middle sections 
    %       row 1 for times of linear part before parabola
    %       row 2 for times of parabola section
    time_seg = zeros([2,size(points, 2)]);
    % store velocity values for function making
    vel_seg = zeros([1,size(points, 2) - 1]);
    % store acceleration values for function making
    acc_seg = zeros([1,size(points, 2)]);

    % Starting section
    % Getting acceleration direction
    acc_1 = sign(points(2) - points(1))*acc;
    td12 = tpts(2) - tpts(1);
    % Get times for parabola section
    time_seg(2,1) = td12 - sqrt((td12)^2 + ((2*(points(1) - points(2))/acc_1)));
    % Getting velocity from 1-2
    vel_seg(1) = (points(2) - points(1))/(td12 - 0.5*time_seg(2,1));
    % Getting acceleration 
    acc_seg(1) = acc_1;

    % Ending section calculation
    % Getting acceleration direction
    acc_la = sign(points(end-1) - points(end))*acc;
    % Getting times for last parabola
    tdn1n = (tpts(end) - tpts(end-1));
    time_seg(2,end) = tdn1n - sqrt((tdn1n)^2 - ((2*(points(end-1) - points(end))/acc_la)));
    % Getting velocty for the last segment
    vel_seg(end) = (points(end) - points(end-1))/(tdn1n - 0.5*time_seg(2,end));
    % Getting accelration for the last parabola
    acc_seg(end) = acc_la;

    % Middle points:
    % Looping over parabolas
    for ii = 1:(size(points, 2) - 2)
        % Calculate the velocity
        v_a = vel_seg(ii);
        if ii == size(points, 2) - 2
            v_b = vel_seg(end);
        else
            v_b = (points(ii+2) - points(ii+1))/(tpts(ii+2) - tpts(ii+1));
            vel_seg(ii+1) = v_b;
        end
        % Get acceleration direction
        acc_t = acc*(sign(v_b - v_a));
        acc_seg(ii+1) = acc_t;
        % Get times for parabola section
        time_seg(2,ii+1) = (v_b - v_a)/acc_t;
    end
    % Looping over linear segments
    for jj = 2:(size(points, 2) - 2)
        % Get times for linear section
        time_seg(1,jj) = (tpts(jj+1)-tpts(jj)) - (0.5*time_seg(2,jj)) - (0.5*time_seg(2,jj+1));
    end

    % Calculations for first linear segment
    time_seg(1,1) = td12 - time_seg(2,1) - (0.5*time_seg(2,2));
    % Celcualtions for last linear segment
    time_seg(1,end-1) = tdn1n - time_seg(2,end) - (0.5*time_seg(2,end-1));

    % ------------------------------------------------------------------
    % GENERATION OF LINE EQUATIONS
    % Store linear segment coefficients
    %       First column has offsets
    %       Second column has velocity slopes
    %       Upper rows are earlier in the trajectory
    Lin_Coeff = zeros((size(points, 2) - 1), 2);
    Lin_Coeff(1,1) = vel_seg(1);
    Lin_Coeff(1,2) = points(1) - (vel_seg(1)*0.5*time_seg(2,1));
    xlin_off = 0;
    for kk = 2:(size(points, 2) - 1)
        xlin_off = xlin_off + time_seg(2,kk-1) + time_seg(1,kk-1);
        Lin_Coeff(kk,2) = points(kk) - (vel_seg(kk)*(xlin_off+0.5*time_seg(2,kk)));
        Lin_Coeff(kk,1) = vel_seg(kk);
    end

    % Store parabolic segment coefficients
    %       First row according to coefficients, others according to format
    %       First column has offsets
    %       Second column has velocity slopes
    %       Upper rows are earlier in the trajectory
    Para_Coeff = zeros(size(points, 2), 3);
    Para_Coeff(1,1) = 0.5*(vel_seg(1)/time_seg(2,1));
    Para_Coeff(1,2) = 0;
    Para_Coeff(1,3) = points(1);
    % End Section
    z_e = 0.5*time_seg(2,end-1) + time_seg(1,end-1);
    f_e = (vel_seg(end))/(2*(tpts(end) - tpts(end-1)));
    t_einb = points(end-1) + ...
        (vel_seg(end)*((tpts(end) - tpts(end-1)) - time_seg(2,end)));
    %Para_Coeff(end,1) = -0.5*(vel_seg(end)/(tpts(end) - tpts(end-1)));
    Para_Coeff(end,1) = -0.5*(vel_seg(end)/(time_seg(2,end)));
    Para_Coeff(end,2) = vel_seg(end);
    Para_Coeff(end,3) = t_einb;
    for jj = 2:(size(points, 2) - 1)        
        z_t = 0.5*time_seg(2,jj-1) + time_seg(1,jj-1);
        acc_dir = acc_seg(jj);
        Para_Coeff(jj,3) = points(jj-1) + vel_seg(jj-1)*(z_t);
        Para_Coeff(jj,2) = vel_seg(jj-1);
        Para_Coeff(jj,1) = 0.5*(acc_dir);
    end

    vel_seg
    acc_seg

end




