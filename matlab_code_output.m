>> ccode(W_z)
ans =
    '  W_z[0][0] = x_ddot_d-psi_dot*y_dot;
       W_z[0][2] = -(c_rr_hat*x_dot)/k_hat-(m_hat*(x_ddot_d-psi_dot*y_dot))/k_hat;
       W_z[0][3] = x_dot;
       W_z[1][1] = psi_ddot_d;
       W_z[1][4] = -(c_d_hat*l*y_dot+j_z_hat*psi_ddot_d*x_dot+c_s_hat*(l*l)*psi_dot)/(c_af_hat*x_dot);
       W_z[1][5] = ((l*l)*psi_dot)/x_dot;
       W_z[1][6] = (l*y_dot)/x_dot;'

>> ccode(C0)
ans =
    '  C0[0][0] = -k1*(x_dot-x_dot_d)+(c_rr*x_dot+m*x_ddot_d-m*psi_dot*y_dot)/k;
       C0[1][0] = -k2*(psi_dot-psi_dot_d)+(j_z*psi_ddot_d+(l*(c_d*y_dot+c_s*l*psi_dot))/x_dot)/(c_af*l);'



Matlab controller;


function X_dot = full_sys(t,X)
    
    % load params 
    params;

    % initialize x_dot
    X_dot = zeros(size(X));

    % pull out state values
    z_dot = X(z_dot_index);
    theta_h = X(theta_h_index);

    % get reference signals
    z_dot_d = z_dot_d_function(t)';
    z_ddot_d = z_ddot_d_function(t)';
 
    % get controller signal 
    u = C_function(z_ddot_d, z_dot_d, z_dot, theta_h, k_vector);
        
    % apply controller to system dynamics & update
    X_dot(z_dot_index) = D_function(theta, z_dot, u);

    % update state and estimated parameters
    delta_z_dot = z_dot(1:2,:) - z_dot_d(1:2,:);

    %Wz({z_ddot_d,  z_dot, theta_hat})
    W_z = W_z_function(z_ddot_d, z_dot, theta_h);

    X_dot(theta_h_index) = -gamma_gain' * W_z' * delta_z_dot;

    % other parameters will have 0 change

end
