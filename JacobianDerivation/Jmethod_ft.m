%% MAE263B Homework 3 Code Trevor Oshiro: Force/Torque Propagation function
function [J_ft, t_bet, f_bet, Rbet, Pbet] = Jmethod_ft(T_bet, j_type, f_in, t_in)
    f_num = size(j_type, 2);
    % Generate symbolic variables for rotational joints
    if ~exist('f_in', 'var') || isempty(f_in)
        syms fx fy fz real
        F_var = [fx, fy, fz];
    else
        F_var = f_in;
    end
    if ~exist('t_in', 'var') || isempty(t_in)
        syms nx ny nz real
        T_var = [nx, ny, nz];
    else
        T_var = t_in;
    end    
    fprintf('\n')
    fprintf('End Effector force values used for Jacobian')
    fprintf('\n')
    F_var
    fprintf('\n')
    fprintf('End Effector moment values used for Jacobian')
    fprintf('\n')
    T_var
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
    R_jac = [simplify(R_end), zeros(3,3);
        zeros(3,3), simplify(R_end)];

    % Derivation of forces
    f_bet.f{f_num} = F_var';
    for jj = f_num:-1:2
        f_bet.f{jj-1} = simplify(Rbet.m{jj} * f_bet.f{jj});
    end
    % Derivation of moments
    t_bet.v{f_num} = T_var';
    eq_FT = [T_var(end)];
    for nn = f_num:-1:2
        t_bet.v{nn-1} = simplify((Rbet.m{nn} * t_bet.v{nn}) ...
            + (cross(Pbet.m{nn}, f_bet.f{nn-1})));
        eq_FT = [eq_FT, (t_bet.v{nn-1}')*[0;0;1]];
    end
    eq_FT = flip(eq_FT)';

    % Jacobian Creation
    J_ft = (equationsToMatrix(eq_FT, [F_var, T_var]))';
    FT = J_ft';
    J_ft = simplify(R_jac*J_ft);

end