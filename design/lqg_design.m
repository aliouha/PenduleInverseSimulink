% =========================================================
%  lqg_design.m — Conception LQG (LQR + Kalman)
% =========================================================
run('E:\Mes-CD\Matlab-Projects\PenduleInverseSimulink\params\init_params.m')

%% ─── LQR ─────────────────────────────────────────────────
Q_lqr = diag([5000, 1, 100, 100]);
R_lqr = 0.01;
K_lqr = lqr(A, B, Q_lqr, R_lqr);
fprintf('Gain LQR K =\n'); disp(K_lqr)
poles_BF = eig(A - B*K_lqr);
fprintf('Pôles BF (LQR) :\n'); disp(poles_BF)
% --- Calcul de N_bar pour le suivi de consigne sur x ---
C_track = [1 0 0 0]; % On veut que x suive la consigne
N_bar = -1 / (C_track * inv(A - B * K_lqr) * B);

fprintf('Gain de pré-compensation N_bar = %.4f\n', N_bar);

%% ─── KALMAN — mesure x et θ ──────────────────────────────
C_mes = [1 0 0 0 ;    % mesure x
         0 0 1 0 ];   % mesure θ

% Vérification observabilité
Ob_mes = obsv(A, C_mes);
fprintf('Rang observabilité : %d / %d\n', rank(Ob_mes), size(A,1))

% Placement pôles observateur
poles_obs = [-10+2j ; -10-2j ; -20 ; -30];
L_kal = place(A', C_mes', poles_obs)';

fprintf('Gain L =\n'); disp(L_kal)
fprintf('Size L_kal : %dx%d\n', size(L_kal,1), size(L_kal,2))

%% ─── Matrices observateur ────────────────────────────────
A_kal = A - L_kal * C_mes;   % 4x4
B_kal = [B, L_kal];          % [4x1, 4x2] = 4x3 ✔
C_kal = eye(4);
D_kal = zeros(4, 3);         % 4x3 ✔

fprintf('Size B_kal : %dx%d\n', size(B_kal,1), size(B_kal,2))
fprintf('Pôles observateur :\n'); disp(eig(A_kal))