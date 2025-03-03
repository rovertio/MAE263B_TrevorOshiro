function [th] = Ikine2RR(coor, con, elbow)
    % Calculates inverse kinematics of a 2R planar motion
    % con: configuration lengths of links
    % coor: input coordinates for the end effector
    % elbow: input orientation for elbow joint
    % Variable definition
    x = coor(1);
    y = coor(2);
    L1 = con(1);
    L2 = con(2);

    % Calculation of theta2
    c2 = ((x^2 + y^2) - (L1^2 + L2^2))/(2*L1*L2);
    if elbow == 1
        s2 = sqrt(1 - c2^2);
    else
        s2 = -sqrt(1 - c2^2);
    end
    t2 = atan2(s2, c2);

    % Calculation of theta1
    a = L1 + L2*c2;
    c = L2*s2;
    d = x;
    e = -L2*s2;
    f = L1 + L2*c2;
    g = y;

    if (a*f - c*e) == 0
        fprintf('No solution')
    elseif (a*f - c*e) > 0
        t1 = atan2((a*g - d*e), (d*f - c*g));
    elseif (a*f - c*e) < 0
        t1 = atan2((d*e - a*g), (c*g - d*f));
    end

    %th = rad2deg([t1, t2]);
    th = [t1, t2];

end