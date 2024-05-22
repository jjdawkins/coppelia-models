% calculate equations needed for rover controller simulation
clc; clearvars;

% make folder for functions
folderName = 'genFunctions';
if exist(folderName, 'dir') ~= 7
    % Folder doesn't exist, create it
    mkdir(folderName);
    disp(['Folder "', folderName, '" created.']);
else
    % Folder exists
    disp(['Folder "', folderName, '" already exists.']);
end

% Build the System with MATLAB syms
syms x_dot y_dot psi_dot real;
syms x_dot_d y_dot_d psi_dot_d real;
syms x_ddot y_ddot psi_ddot real;
syms x_ddot_d y_ddot_d psi_ddot_d real;
syms m j_z k c_rr c_af c_s c_d real;
syms m_hat j_z_hat k_hat c_rr_hat c_af_hat c_s_hat c_d_hat real; 
syms i_in delta_in real;
syms a b c d e real;
syms k1 k2 real;
syms l real;

l=0.16;

% Make a check cell
check = {};

% parameter vectors
theta =[m j_z k c_rr c_af c_s c_d]';
n = length(theta);
theta_hat = [m_hat j_z_hat k_hat c_rr_hat c_af_hat c_s_hat c_d_hat]';

% velocities
z_dot = [x_dot psi_dot y_dot]';
z_ddot = [x_ddot psi_ddot, y_ddot]';
z_ddot_d = [x_ddot_d  psi_ddot_d y_ddot_d]';
z_dot_d = [x_dot_d psi_dot_d y_dot_d]';

% inputs
u = [i_in, delta_in]';

% Define system dynamics D
D = [
     k/m * i_in - c_rr/m * x_dot + y_dot*psi_dot;
     -(c_d  * l)/(j_z * x_dot) * y_dot - (c_s * l^2)/(j_z*x_dot) * psi_dot + (c_af* l)/(j_z) * delta_in;
       -(c_s * y_dot)/(m*x_dot) - (l*c_d)/(m*x_dot) *(psi_dot) + c_af/m * delta_in - x_dot *psi_dot;
];


% split up to F and G st m*z_ddot = F(s,theta) + G(u,theta) 
M = diag([m j_z m]);
% we know what G should be (control portion)
G = [k * i_in; c_af * l * delta_in; c_af * delta_in]
F = simplify(M*D-G) 

% Now break up G into B and A:  G = B A(theta) * u
A = diag([k c_af]);
B = [1 0; 0 l; 0 1];

% check to make sure G = BAu
G_check = simplify(G-B*A*u);

% Define the regressors as labeled by llw
W_m = simplify(jacobian(M*z_ddot, theta))
W_f = simplify(jacobian(F, theta))
W_g = simplify(jacobian(G, theta));

% W_m * Theta = (W_f + W_g) Theta
% 0 = (W_m - W_f - W_g) Theta;
W_n = (W_m - W_f - W_g);
W_n = W_n(1:2, :)


% Check that regressors work
W_n_check = simplify(-W_m+W_f+W_g);
check = simplify(D-M\((W_f + W_g)*theta));

% Define the controller params
delta_x_dot = x_dot - x_dot_d;
delta_psi_dot = psi_dot - psi_dot_d;


% Create the ideal controller
K_gain = diag([k1, k2]);
delta_z_bar = [delta_x_dot; delta_psi_dot];
B_bar = B(1:2,:);
M_bar = M(1:2,1:2);
z_ddot_d_bar = z_ddot_d(1:2,:);
F_bar = F(1:2,:);

% controller in form of [i_in; delta_in]
C0 = inv(A) * inv(B_bar) *  (M_bar * z_ddot_d_bar - F_bar) - K_gain * delta_z_bar;
C0 = simplify(C0)

% print out the controlled plant
D_controlled = subs(D,[i_in, delta_in],C0');
D_controlled = simplify(D_controlled);

m_delta_z_ddot = simplify(M * D_controlled -[m*x_ddot_d; j_z*psi_ddot_d; 0] );

m_delta_z_ddot = simplify(m_delta_z_ddot);
ideal_controlled_system = m_delta_z_ddot(1:2,:)

% create controller regressors
W_m_bar_d = subs(W_m(1:2,:), z_ddot, z_ddot_d);
W_f_bar = W_f(1:2,:);
W_c = simplify(W_m_bar_d - W_f_bar)


% get W_delta
syms delta_theta [n 1];
new_theta_hat = theta + delta_theta;

delta_A = subs(A, theta, delta_theta);
A_hat = subs(A, theta, new_theta_hat);

term_1 = simplify(delta_A * inv(A_hat) * W_c * new_theta_hat);
term_1 = subs(term_1, theta, (theta_hat-delta_theta));

W_delta = jacobian( term_1, delta_theta);
W_delta = simplify(W_delta)

% get W_z
W_z = simplify(W_c - W_delta)

k_vector = [k1 k2]';

% Export the Dynamics Function
matlabFunction(D,"File", folderName + "/D_function","Vars",...
    {theta, z_dot, u},...
    "Comments", '{theta, z_dot, u}');

% save controller function
matlabFunction(C0,"File", folderName + "/C_function","Vars",... 
    {z_ddot_d, z_dot_d, z_dot, theta, k_vector},...
    "Comments", '{z_ddot_d, z_dot_d, z_dot, theta, k_vector}');

% save W_z function
matlabFunction(W_z,"File", folderName + "/W_z_function","Vars",... 
    {z_ddot_d,  z_dot, theta_hat},...
    "Comments", '{z_ddot_d,  z_dot, theta_hat}');


% save W_n function
matlabFunction(W_n,"File", folderName + "/W_n_function","Vars",... 
    {z_ddot,  z_dot, u},...
    "Comments", '{z_ddot,  z_dot, u}');


% create reference trajectory function
syms t real;
z_dot_d_function = [2 + 1*sin(0.31 * t), 0.20 * sin(0.23 * t) + 0.05 * sin(0.31 *t), 0];
z_ddot_d_function = diff(z_dot_d_function, t);

matlabFunction(z_dot_d_function,"File","genFunctions/z_dot_d_function","Vars",{t},...
    "Comments", '{t}');

matlabFunction(z_ddot_d_function,"File","genFunctions/z_ddot_d_function","Vars",{t},...
    "Comments", '{t}');







