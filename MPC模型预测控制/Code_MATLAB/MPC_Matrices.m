function [E, H] = MPC_Matrices(A, B, Q, R, F, N)
    % This function generates the E and H matrices used in MPC.
    % A is the system matrix, B is the input matrix, Q and R are the state
    % and input weighting matrices, F is the terminal state weighting matrix, 
    % and N is the horizon length.

    n = size(A, 1); % A is an n x n matrix, get n
    p = size(B, 2); % B is an n x p matrix, get p

    % Initialize M matrix. M matrix is (N+1)n x n,
    % It starts with an n x n identity matrix on top.
    M = [eye(n); zeros(N*n, n)];

    % Initialize C matrix, start with (N+1)n x NP zeros
    C = zeros((N+1)*n, N*p);

    % Define M and C
    tmp = eye(n); % Define an n x n identity matrix

    % Update M and C
    for i = 1:N % Loop from 1 to N
        rows = i*n + (1:n); % Define current rows, starting at i*n, total n rows
        C(rows, :) = [tmp*B, C(rows-n, 1:end-p)]; % Fill the C matrix
        tmp = A * tmp; % Multiply tmp by A each time
        M(rows, :) = tmp; % Fill the M matrix
    end

    % Define Q_bar and R_bar
    Q_bar = kron(eye(N), Q);
    Q_bar = blkdiag(Q_bar, F);
    R_bar = kron(eye(N), R); 

    % Calculate G, E, H
    G = M' * Q_bar * M; % G: n x n
    E = M' * Q_bar * C; % E: NP x n
    H = C' * Q_bar * C + R_bar; % H: NP x NP

    % 强制H为对称矩阵
    %H = (H + H') / 2;

end