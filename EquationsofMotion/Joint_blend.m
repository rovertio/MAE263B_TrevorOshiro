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
    
    % time_seg
    % vel_seg
    % acc_seg

end