%% MAE263B Homework 3 Code Trevor Oshiro: Explicit method function
function [J_ex] = Jmethod_exp(T_base, j_type)
    % Calculations for each column (joint)
    f_num = size(j_type, 2);
    J_ex = [];
    for ii = 1:f_num
        if j_type(ii) == 1      % Revolute Joint
            % Calculating Jv (top part)
            Z = T_base.t{ii}.a;
            Jv_top = cross(Z, (T_base.t{f_num}.t - T_base.t{ii}.t));
            % Multiplier for Jw (bottom part)
            wm = 1;
        elseif j_type(ii) == 0  % Prismatic joint
            Jv_top = T_base.t{ii}.a;
            wm = 0;
        end
        Jw_bot = wm.*T_base.t{ii}.a;

        % Adding to Jacobian
        J_ex = [J_ex, [Jv_top; Jw_bot]];
    end
end
    