% =========================================================
%  compare_results.m — Comparaison 3 modèles Pendule Inversé
%  Boucle ouverte | LQR | LQG
%  Layout : 2 lignes x 3 colonnes
%  Ligne 1 : x(t)     | Ligne 2 : θ(t)
% =========================================================

cd('E:\Mes-CD\Matlab-Projects\PenduleInverseSimulink')
addpath(genpath(pwd))

%% ─── Chargement paramètres et design ────────────────────
run('params/init_params.m')
run('design/lqg_design.m')

%% ─── Simulation 1 : Boucle ouverte ──────────────────────
sim('models/PI_boucle_ouverte');
s1      = sim('models/PI_boucle_ouverte');
t1      = s1.x_out.Time;
x1      = s1.x_out.Data;
theta1  = s1.theta_out.Data;

%% ─── Simulation 2 : Retour d'état LQR ───────────────────
sim('models/PI_Retour_Etat_LQR');
s2      = sim('models/PI_Retour_Etat_LQR');
t2      = s2.x_out.Time;
x2      = s2.x_out.Data;
theta2  = s2.theta_out.Data;

%% ─── Simulation 3 : LQG (Kalman + LQR) ──────────────────
sim('models/PI_Retour_Etat_LQG');
s3      = sim('models/PI_Retour_Etat_LQG');
t3      = s3.x_out.Time;
x3      = s3.x_out.Data;
theta3  = s3.theta_out.Data;

%% ─── Figure comparaison ──────────────────────────────────
figure('Name','Comparaison Pendule Inversé — 3 Modèles', ...
       'NumberTitle','off', ...
       'Position',[100 100 1400 700]);

%% --- Ligne 1 : Position x(t) ----------------------------

% Colonne 1 — Boucle ouverte
subplot(2,3,1)
plot(t1, x1, 'r', 'LineWidth', 1.5)
xlabel('Temps (s)')
ylabel('x (m)')
title('Boucle Ouverte — x(t)')
legend('x réel')
grid on
xlim([0 t1(end)])

% Colonne 2 — LQR
subplot(2,3,2)
plot(t2, x2, 'b', 'LineWidth', 1.5)
xlabel('Temps (s)')
ylabel('x (m)')
title('Retour d''état LQR — x(t)')
legend('x réel')
grid on
xlim([0 t2(end)])

% Colonne 3 — LQG
subplot(2,3,3)
plot(t3, x3, 'g', 'LineWidth', 1.5)
xlabel('Temps (s)')
ylabel('x (m)')
title('LQG (Kalman+LQR) — x(t)')
legend('x réel')
grid on
xlim([0 t3(end)])

%% --- Ligne 2 : Angle θ(t) -------------------------------

% Colonne 1 — Boucle ouverte
subplot(2,3,4)
plot(t1, theta1, 'r', 'LineWidth', 1.5)
xlabel('Temps (s)')
ylabel('\theta (rad)')
title('Boucle Ouverte — \theta(t)')
legend('\theta réel')
grid on
xlim([0 t1(end)])

% Colonne 2 — LQR
subplot(2,3,5)
plot(t2, theta2, 'b', 'LineWidth', 1.5)
xlabel('Temps (s)')
ylabel('\theta (rad)')
title('Retour d''état LQR — \theta(t)')
legend('\theta réel')
grid on
xlim([0 t2(end)])

% Colonne 3 — LQG
subplot(2,3,6)
plot(t3, theta3, 'g', 'LineWidth', 1.5)
xlabel('Temps (s)')
ylabel('\theta (rad)')
title('LQG (Kalman+LQR) — \theta(t)')
legend('\theta réel')
grid on
xlim([0 t3(end)])

%% ─── Mise en page ────────────────────────────────────────
sgtitle('Pendule Inversé — Comparaison des 3 Stratégies de Commande', ...
        'FontSize', 14, 'FontWeight', 'bold')

%% ─── Sauvegarde ──────────────────────────────────────────
saveas(gcf, 'results/comparaison_pendule.png')
fprintf('✔ Comparaison terminée — figure sauvegardée !\n')