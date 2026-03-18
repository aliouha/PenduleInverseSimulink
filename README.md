# Inverted Pendulum Control — Commande Pendule Inversé Linéaire

> **Simulation & Control Design** | MATLAB/Simulink

---

## Project Overview — Vue d'ensemble

Ce projet implémente différentes stratégies de commande pour un pendule inversé
linéarisé autour de son point d'équilibre instable (position verticale haute).

---

## System Parameters — Paramètres du Système

| Paramètre | Symbole | Valeur | Unité |
|-----------|---------|--------|-------|
| Masse chariot | M | 0.5 | kg |
| Masse pendule | m | 0.2 | kg |
| Longueur pendule | L | 0.3 | m |
| Frottement | b | 0.1 | N.s/m |
| Gravité | g | 9.81 | m/s² |
| Inertie pendule | I | mL²/3 = 0.006 | kg.m² |

---

## Mathematical Model — Modèle Mathématique

### Nonlinear Equations — Équations non linéaires (Lagrange)

```
(M+m)ẍ + bẋ + mLθ̈cos(θ) - mLθ̇²sin(θ) = F    (chariot)
(I+mL²)θ̈ + mgLsin(θ) = -mLẍcos(θ)             (pendule)
```

### Linearization — Linéarisation (θ≈0)

```
sin(θ) ≈ θ
cos(θ) ≈ 1
θ̇²    ≈ 0
```

### State-Space — Représentation d'état

```
ẋ = A*x + B*u
y  = C*x

x = [x ; ẋ ; θ ; θ̇]     (vecteur d'état)
u = F                     (force sur chariot)

A = [0      1        0          0    ]
    [0    -0.1818   2.6755       0    ]
    [0      0        0           1    ]
    [0     0.4545  31.2136       0    ]

B = [0 ; 1.8182 ; 0 ; -4.5455]

C = [1  0  0  0]    mesure x
    [0  0  1  0]    mesure θ
```

---

## Open-Loop Poles — Pôles en Boucle Ouverte

```
p1 =  0.0000   ← intégrateur (chariot libre)
p2 = -0.2208   ← stable lent
p3 = +5.6057   ← INSTABLE ! pendule tombe
p4 = -5.5667   ← stable rapide
```

---

## Control Strategies — Stratégies de Commande

### 1. Open Loop — Boucle Ouverte

```
[F] ──► [Système] ──► [x, ẋ, θ, θ̇]
```
- Système instable — x et θ divergent exponentiellement ✔

### 2. State Feedback LQR — Retour d'État Optimal

```
u = -K_lqr * x

Critère LQR : J = ∫(x'Qx + u'Ru) dt

Q = diag([1000, 1, 100, 100])   ← pondération états
R = 0.01                         ← pondération commande

K_lqr = [-316.23  -196.36  -927.78  -180.33]

Pôles BF : -454.97, -3.29, -2.29±3.05j
```
- θ converge à 0 ✔ | x converge à 0 ✔
- Perturbation F=5N rejetée ✔

### 3. LQG — Kalman Filter + LQR

```
Observateur Kalman :
x̂˙ = A*x̂ + B*u + L*(y - C*x̂)

Loi de commande :
u = -K_lqr * x̂

Mesures capteurs : x  et  θ  uniquement

Pôles observateur : -30, -20, -10±2j

L_kal = [35.08    8.36 ]
        [269.37  127.65]
        [2.51    34.74 ]
        [3.32   255.52 ]
```
- États estimés depuis x et θ seulement ✔
- Robuste au bruit de mesure ✔

---

## Results — Résultats

| Strategy | x converge | θ converge | Capteurs |
|----------|-----------|------------|----------|
| Boucle ouverte | ❌ diverge | ❌ diverge | — |
| LQR | ✔ | ✔ | x, ẋ, θ, θ̇ |
| LQG | ✔ | ✔ | x, θ seulement |

---

## Project Structure — Structure du Projet

```
PenduleInverseSimulink/
├── params/
│   └── init_params.m          # Paramètres + matrices A,B,C
├── models/
│   ├── PI_boucle_ouverte.slx  # Modèle boucle ouverte
│   ├── PI_Retour_Etat_LQR.slx # Modèle LQR
│   └── PI_Retour_Etat_LQG.slx # Modèle LQG
├── design/
│   └── lqg_design.m           # Calcul K_lqr + L_kal
├── scripts/
│   └── compare_results.m      # Comparaison 3 modèles
├── results/
│   └── comparaison_pendule.png
└── README.md
```

---

## How to Run — Lancer le Projet

```matlab
% 1. Initialisation
cd('E:\Mes-CD\Matlab-Projects\PenduleInverseSimulink')
run('params/init_params.m')

% 2. Design LQG
run('design/lqg_design.m')

% 3. Comparaison
run('scripts/compare_results.m')
```

---

*Generated with MATLAB R2024a*
