function u_k = Prediction(x_k, E, H, N, p)
    % Prediction function for MPC control action.
    % x_k: Current state
    % E, H: Matrices from MPC setup
    % N: Control horizon
    % p: Number of inputs

    % Initialize the control vector
    U_k = zeros(N*p, 1);

    % Solve the quadratic programming problem
    U_k = quadprog(2*H,2*E'*x_k);

    % Extract the control action for the first step
    u_k = U_k(1:p,1); % 取第一个结果
end