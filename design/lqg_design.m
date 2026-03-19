% =========================================================================
%  LQG_Segway_Design.m — Conception Robuste Kalman + LQR (Inspiré Moteur CC)
% =========================================================================
clear; close all; clc;

fprintf('=== 1. Chargement des Paramètres Physiques ===\n');
% Charge A, B, C, D du Segway
run('E:\Mes-CD\Matlab-Projects\PenduleInverseSimulink\params\init_params.m')

%% --- PARTIE 1 : CONCEPTION DU FILTRE DE KALMAN (Observateur) ---
fprintf('\n=== 2. Synthèse du Filtre de Kalman ===\n');

% On mesure la position (x) et l'angle du pendule (theta)
C_mes = [1 0 0 0;   % Mesure de x
         0 0 1 0];  % Mesure de theta

% --- RÉGLAGE DES COVARIANCES (Le "Cerveau" du filtre) ---
% Q_kal : Incertitude sur le modèle (Accélération, inclinaison imprevue)
% On fait confiance au modèle pour les vitesses, moins pour les positions
Q_kal = diag([0.01, 1, 0.01, 1]); 

% R_kal : Incertitude sur les capteurs (Bruit des encodeurs et IMU)
% Si R est grand, le filtre lisse énormément (comme sur ton scope moteur)
R_kal = diag([0.1, 0.5]); % Confiance moyenne sur x, plus faible sur theta

% Calcul du gain de Kalman par l'équation de Riccati (Méthode CARE)
[~, ~, L_trans] = care(A', C_mes', Q_kal, R_kal);
L_kal = L_trans';

%% --- PARTIE 2 : CONCEPTION DU RÉGULATEUR LQR (Commande) ---
fprintf('\n=== 3. Synthèse du Régulateur LQR ===\n');

% Pondération des états : [x, x_dot, theta, theta_dot]
% On veut surtout stabiliser l'angle (theta) pour ne pas tomber
Q_lqr = diag([5000, 10, 1000, 10]); 
R_lqr = 0.01; % Coût de l'effort moteur (U)

K_lqr = lqr(A, B, Q_lqr, R_lqr);

%% --- PARTIE 3 : CALCUL DU GAIN DE PRÉ-COMPENSATION (Nbar) ---
% On veut que la position x (état 1) suive exactement la consigne
C_position = [1 0 0 0]; 
Nbar = -1 / (C_position * inv(A - B*K_lqr) * B);

%% --- PARTIE 4 : VÉRIFICATION ET EXPORT ---
fprintf('\n==========================================\n');
fprintf('       RÉSULTATS DE LA SYNTHÈSE SEGWAY      \n');
fprintf('==========================================\n');

% Vérification de la stabilité
poles_lqr = eig(A - B*K_lqr);
poles_obs = eig(A - L_kal*C_mes);

fprintf('1. STABILITÉ :\n');
fprintf('   Pôles LQR (BF) : %s\n', mat2str(round(poles_lqr, 1)));
fprintf('   Pôles Obs (Kal) : %s\n', mat2str(round(poles_obs, 1)));

if all(real(poles_lqr) < 0) && all(real(poles_obs) < 0)
    fprintf('\n✔ SYSTÈME GLOBALEMENT STABLE ET PRÊT\n');
else
    error('⚠ ATTENTION : Instabilité détectée dans le calcul');
end

% Matrices pour le bloc State-Space de l'observateur dans Simulink
A_sim = A - L_kal * C_mes;
B_sim = [B, L_kal]; % Entrées : [u, y1, y2]
C_sim = eye(4);      % On sort les 4 états estimés
D_sim = zeros(4, 3);

fprintf('\n2. GAIN DE COMPENSATION (Nbar) : %.4f\n', Nbar);
fprintf('==========================================\n');

% Sauvegarde automatique pour Simulink
save('lqg_params.mat', 'K_lqr', 'L_kal', 'Nbar', 'A_sim', 'B_sim', 'C_sim', 'D_sim');