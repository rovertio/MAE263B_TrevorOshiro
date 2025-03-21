%% MAE263B Homework 3 Code Trevor Oshiro: Velocity Propagation function
function [J_vp, w_bet, v_bet, Rbet, Pbet] = Jmethod_vpEE(T_bet, j_type, w_in, v_in)
    % Propagation from the base to the end effector
    f_num = size(j_type, 2);
    % Generate symbolic variables for rotational joints
    w_var = sym("vt", [1,f_num]).*j_type;
    d_var = sym("vd", [1,f_num]).*(~j_type);
    vel_var = w_var + d_var;
    % fprintf('\n')
    % fprintf('Variables used for Jacobian')
    % fprintf('\n')
    % vel_var
    % fprintf('\n')
    % Extract rotational matrices for calculations
    for ii = 1:f_num
        Rbet.m{ii} = [T_bet.t{ii}.n, T_bet.t{ii}.o, T_bet.t{ii}.a]';
    end
    % Extract positional vectors for calculations
    for jj = 1:f_num
        Pbet.m{jj} = [T_bet.t{jj}.t];
    end
    % Matrix to tranform jacobian to base frame reference
    % Getting tranfromation to end frame
    R_end = (Rbet.m{1})';
    for ff = 2:f_num
        R_end = R_end*(Rbet.m{ff})';
    end
    R_jac = [simplify(R_end), zeros(3,3);
        zeros(3,3), simplify(R_end)];

    % Derivation of rotational velocities
    % Array of rotational velocities begins with velocity at base
    w_bet.w{1} = w_in;
    for kk = 2:f_num+1
        w_bet.w{kk} = Rbet.m{kk-1}*w_bet.w{(kk-1)}...
            + j_type(kk-1).*[0;0;vel_var(kk-1)];
    end

    % Derivation of linear velocities
    % Array of linear velocities begins with velocity at base
    v_bet.v{1} = v_in;
    for mm = 2:f_num+1
        vel = cross(w_bet.w{mm-1}, Pbet.m{mm-1}) + v_bet.v{mm-1};
        v_bet.v{mm} = Rbet.m{mm-1}*vel...
            + (~j_type(mm-1)).*[0;0;vel_var(mm-1)];
    end

    % Jacobian Creation
    end_vel = [v_bet.v{f_num+1}; w_bet.w{f_num+1}];
    J_vp = equationsToMatrix(end_vel,vel_var);
    % J_vp = simplify(R_jac*J_vp);

end