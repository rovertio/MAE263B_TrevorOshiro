%% MAE263B Homework 3 Code Trevor Oshiro: Force/Torque Propagation function
function [J_ft] = Jmethod_ft(T_bet, j_type, f_in, t_in)
    f_num = size(j_type, 2);
    % Generate symbolic variables for rotational joints
    T_var = sym("T", [1,f_num]).*j_type;
    F_var = sym("F", [1,f_num]).*(~j_type);
    ft_var = T_var + F_var;
    fprintf('\n')
    fprintf('Variables used for Jacobian')
    fprintf('\n')
    ft_var
    fprintf('\n')
    % Extract rotational matrices for calculations
    for ii = 1:f_num
        Rbet.m{ii} = [T_bet.t{ii}.n, T_bet.t{ii}.o, T_bet.t{ii}.a];
    end
    % Extract positional vectors for calculations
    for jj = 1:f_num
        Pbet.m{jj} = [T_bet.t{jj}.t];
    end
    % Matrix to tranform jacobian to base frame reference
    % Getting tranfromation to end frame
    R_end = (Rbet.m{1});
    for ff = 2:f_num
        R_end = R_end*(Rbet.m{ff});
    end
    R_jac = [R_end, zeros(3,3);
        zeros(3,3), R_end];

    % Derivation of forces
    

    % Jacobian Creation
    end_FT = [v_bet.v{f_num+1}; w_bet.w{f_num+1}];
    J_ft = equationsToMatrix(end_FT, ft_var);
    J_ft = simplify(R_jac*J_ft);

end