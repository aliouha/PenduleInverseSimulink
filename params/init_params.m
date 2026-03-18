% =========================================================
%  init_params.m — Paramètres Pendule Inversé Linéaire
%  Projet : PenduleInverse (Boucle ouverte, LQG)
%  MATLAB 2024a
% =========================================================

%% ─── 1. PARAMÈTRES PHYSIQUES ─────────────────────────────
M = 0.5;        % Masse chariot                [kg]
m = 0.2;        % Masse pendule                [kg]
L = 0.3;        % Longueur pendule             [m]
b = 0.1;        % Frottement chariot           [N.s/m]
g = 9.81;       % Gravité                      [m/s²]
I = m*L^2/3;    % Inertie pendule              [kg.m²]

%% ─── 2. TERMES INTERMÉDIAIRES ────────────────────────────
p   = I + m*L^2;          % = 0.024
q   = M + m;              % = 0.7
det = p*q - m^2*L^2;      % = 0.01320

fprintf('p   = %.4f\n', p);
fprintf('q   = %.4f\n', q);
fprintf('det = %.6f\n', det);

%% ─── 3. MATRICES D'ÉTAT ──────────────────────────────────
%
%  ẋ = A*x + B*u
%  y  = C*x
%
%  x = [x ; ẋ ; θ ; θ̇]
%  u = F  (force sur chariot)

A = [0          1              0                0  ;
     0         -b*p/det        m^2*g*L^2/det    0  ;
     0          0              0                1  ;
     0          m*L*b/det      (M+m)*m*g*L/det  0  ];
%                              ↑ signe + ici !
B = [0        ;
     p/det    ;
     0        ;
    -m*L/det  ];

C = [1  0  0  0 ;    % mesure position x
     0  0  1  0 ];   % mesure angle θ

D = zeros(2,1);

%% ─── 4. SIMULATION ───────────────────────────────────────
Ts     = 1e-4;     % Pas de simulation         [s]
Tfinal = 5;        % Durée totale              [s]
x0     = [0; 0; 0.1; 0];  % CI : θ=0.1 rad (légère perturbation)

%% ─── 5. VÉRIFICATION SYSTÈME ─────────────────────────────
fprintf('\n══════════════════════════════════════\n');
fprintf('   PARAMÈTRES PENDULE INVERSÉ\n');
fprintf('══════════════════════════════════════\n');
fprintf('M=%.1f kg | m=%.1f kg | L=%.1f m\n', M, m, L);
fprintf('b=%.2f | g=%.2f | I=%.4f kg.m²\n', b, g, I);

fprintf('\nMatrice A :\n'); disp(A)
fprintf('Matrice B :\n'); disp(B)
fprintf('Matrice C :\n'); disp(C)

% Valeurs propres
poles = eig(A);
fprintf('Pôles naturels (boucle ouverte) :\n');
for i = 1:length(poles)
    fprintf('  p%d = %.4f\n', i, poles(i));
end

% Vérification stabilité
if any(real(poles) > 0)
    fprintf('⚠ Système INSTABLE en boucle ouverte !\n');
else
    fprintf('✔ Système stable\n');
end

% Commandabilité & Observabilité
Co = ctrb(A, B);
Ob = obsv(A, C);
fprintf('\nRang Commandabilité : %d / %d', rank(Co), size(A,1));
fprintf('\nRang Observabilité  : %d / %d\n', rank(Ob), size(A,1));

if rank(Co) == size(A,1)
    fprintf('✔ Système COMMANDABLE\n');
end
if rank(Ob) == size(A,1)
    fprintf('✔ Système OBSERVABLE\n\n');
end