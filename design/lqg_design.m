% =========================================================================
%  lqg_design_kalman_robust.m - Conception LQG avec vérifications
% =========================================================================
clear; close all; clc;

%% Chargement des paramètres
fprintf('=== Chargement des paramètres ===\n');
run('E:\Mes-CD\Matlab-Projects\PenduleInverseSimulink\params\init_params.m')

% Vérification des variables essentielles
if ~exist('A','var') || ~exist('B','var')
    error('Les matrices A et B doivent être définies dans init_params.m');
end

fprintf('Dimensions A : %dx%d\n', size(A,1), size(A,2));
fprintf('Dimensions B : %dx%d\n', size(B,1), size(B,2));

% Si C et D existent, on les ignore car on utilise C_mes
if exist('C','var')
    fprintf('C existante (ignorée) : %dx%d\n', size(C,1), size(C,2));
end
if exist('D','var')
    fprintf('D existante (ignorée) : %dx%d\n', size(D,1), size(D,2));
end

%% ─── 1. LQR (régulation) ─────────────────────────────────────────────
fprintf('\n=== Calcul du LQR ===\n');
Q_lqr = diag([5000, 1, 100, 100]);
R_lqr = 0.01;
K_lqr = lqr(A, B, Q_lqr, R_lqr);
fprintf('Gain LQR K : %dx%d\n', size(K_lqr,1), size(K_lqr,2));
fprintf('Pôles BF LQR :\n'); disp(eig(A - B*K_lqr));

%% ─── 2. Filtre de Kalman ─────────────────────────────────────────────
fprintf('\n=== Calcul du filtre de Kalman ===\n');

% Matrice de mesure (x et θ)
C_mes = [1 0 0 0;   % mesure de x
         0 0 1 0];  % mesure de θ
fprintf('Dimensions C_mes : %dx%d\n', size(C_mes,1), size(C_mes,2));

% Vérification observabilité
Ob = obsv(A, C_mes);
fprintf('Rang observabilité : %d / %d\n', rank(Ob), size(A,1));
if rank(Ob) < size(A,1)
    error('Le système n''est pas observable avec ces mesures.');
end

% --- Paramètres du bruit de mesure ---
% Vous utilisez un bloc Random Number avec Variance = ? (ex: 0.01)
% Pour forcer le filtrage, on prend Rn très grand
fprintf('\nRÉGLAGE DES COVARIANCES (pour un filtrage maximal)\n');
Rn = diag([100, 100]);      % TRÈS GRAND : le filtre ignore presque les mesures
fprintf('Rn :\n'); disp(Rn);

% Bruit d'état (incertitude modèle) - très petit
Qn = 1e-6 * eye(4);        % PRESQUE NUL : modèle parfait
fprintf('Qn :\n'); disp(Qn);

% --- Calcul du gain de Kalman par dualité ---
% Système dual : A_dual = A', B_dual = C_mes', Q_dual = Qn, R_dual = Rn
% La commande lqr retourne K_dual tel que u_dual = -K_dual * x_dual
% K_dual a pour dimensions (nombre d'entrées du dual) x (nombre d'états du dual)
% Ici, entrées du dual = sorties de C_mes = 2, états du dual = 4 → K_dual = 2x4
[K_dual, ~, ~] = lqr(A', C_mes', Qn, Rn);
fprintf('Dimensions K_dual : %dx%d\n', size(K_dual,1), size(K_dual,2));

% Le gain du filtre est la transposée : L = K_dual'  (4x2)
L_kal = K_dual';
fprintf('Dimensions L_kal : %dx%d\n', size(L_kal,1), size(L_kal,2));

% Vérification du produit L_kal * C_mes
temp = L_kal * C_mes;
fprintf('Dimensions L_kal * C_mes : %dx%d (doit être 4x4)\n', size(temp,1), size(temp,2));

% Matrice d'état de l'observateur
A_obs = A - L_kal * C_mes;
fprintf('Dimensions A_obs : %dx%d\n', size(A_obs,1), size(A_obs,2));
fprintf('Pôles de l''observateur (très lents normalement) :\n');
disp(eig(A_obs));

%% ─── 3. Matrices pour Simulink ────────────────────────────────────────
fprintf('\n=== Construction des matrices pour Simulink ===\n');
A_kal = A_obs;                              % 4x4
B_kal = [B, L_kal];                         % 4x3 (u, mesure_x, mesure_theta)
C_kal = eye(4);                              % on estime tous les états
D_kal = zeros(4, 3);                         % 4x3

fprintf('Dimensions A_kal : %dx%d\n', size(A_kal,1), size(A_kal,2));
fprintf('Dimensions B_kal : %dx%d\n', size(B_kal,1), size(B_kal,2));
fprintf('Dimensions C_kal : %dx%d\n', size(C_kal,1), size(C_kal,2));
fprintf('Dimensions D_kal : %dx%d\n', size(D_kal,1), size(D_kal,2));

% Sauvegarde
save('lqg_params.mat', 'K_lqr', 'A_kal', 'B_kal', 'C_kal', 'D_kal', 'L_kal');
fprintf('\nMatrices sauvegardées dans lqg_params.mat\n');