%% MAE263B HW5: Equations of Motion, Trevor Oshiro

clear all
clc;
clf;

%% Establishing variables

syms L1 L2 m1 m2 real
syms t1 t2 real
syms dt1 dt2 real
syms d2t1 d2t2 real

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

%% Newton-Euler Method
n=2;

% Angular velocity, linear velocity, force, and moment
wvec.v{1} = [0;0;0];
dwvec.v{1} =[0;0;0];
dvvec.v{1} = [0;g;0];
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
PosCvec.i{1} = [L1;0;0];
PosCvec.i{2} = [L2;0;0];

% Inertia of links
CImat.i{1} = zeros(3,3);
CImat.i{2} = zeros(3,3);

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
% f_in.v{n+1} = [f3x; f3y; f3z];
% n_in.v{n+1} = [n3x; n3y; n3z];
f_in.v{n+1} = [0;0;0];
n_in.v{n+1} = [0;0;0];
tor_in.v{n} = [];

% Inward Iteration
for qq = n:-1:1
    f_in.v{qq} = (Rotvec.i{qq+1}*f_in.v{qq+1}) + Fvec.v{qq};
    n_in.v{qq} = Nvec.v{qq} + (Rotvec.i{qq+1}*n_in.v{qq+1}) + ...
        cross(PosCvec.i{qq}, Fvec.v{qq}) + ...
        cross(Posvec.i{qq+1}, (Rotvec.i{qq+1}*f_in.v{qq+1}));
    tor_in.v{qq} = simplify((n_in.v{qq}')*[0;0;1]);
end



%% Lagrange method for equations
% Vectors to center of mass
P1c1 = [0.5*L1; 0; 0];
P0c1 = (T01)*[P1c1;1];
P0c1 = simplify(P0c1(1:3));
P2c2 = [0.5*L2; 0; 0];
P0c2 = (T02)*[P2c2;1];
P0c2 = simplify(P0c2(1:3));

% Inertial matrices
I1c1 = (1/12).*m1.*(L1^2)*[1, 0, 0;
    0, 1, 0;
    0, 0, 1];
I2c2 = (1/12).*m2.*(L2^2)*[1, 0, 0;
    0, 1, 0;
    0, 0, 1];
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

% Gravity Vector
glan = [0; -g; 0];
G1 = simplify((-m1*glan'*J0v1(:,1)) + (-m2*glan'*J0v2(:,1)));
G2 = simplify((-m1*glan'*J0v1(:,2)) + (-m2*glan'*J0v2(:,2)));



