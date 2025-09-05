%% --------------- SIMULATION PARAMETERS --------------- 
T_end      = 60;          % [s] tempo finale
dt         = 1.0;         % [s] passo di integrazione (loop: 60 iterazioni)
t_grid     = 0:dt:T_end;  % griglia temporale
nSteps     = numel(t_grid)-1;

nDim       = 2;           % lavoro in piano (x,y)
workspace  = [-10 10;     % [xmin xmax;
               -10 10];   %  ymin ymax] (placeholder, non vincolante)
rng_seed   = 42;          % seed per riproducibilità (uso solo per posizioni/ID random)
use_random_init = false;  % se true, posizioni iniziali random; altrimenti layout deterministico

%% --------------- OMRS / TEAM SCHEDULING --------------- 
N_max      = 5;                 % numero totale di agenti disponibili (ID globali)
agentIDs   = 1:N_max;           % etichette globali
N0         = N_max;                 % agenti attivi a t=0
activeIDs0 = 1:N0;              % scelta iniziale degli ID attivi (puoi cambiarla)

event_time = [20, 30, 40]; % tempi in cui succede qualcosa
event_type = [-1, 1, 1 ]; % cosa succede ai nodi
event_agent = { [1,3], [1], [3] };  % quali agenti sono agigunti e rimossi

%% ---------------  INITIAL STATE --------------- 
init_spacing    = 0.5;      % distanza target tra vicini (≈ dBeta_des)
init_speed_max  = 1.0;      % [m/s] saturazione di velocità (per sicurezza)
init_vel_zero   = true;     % velocità iniziale nulla

%% ---------------  ROBOT DYNAMICS / PASSIVITY (PLACEHOLDER) --------------- 
M_i   = diag([1, 1]);       % massa (2D)
B_i   = diag([0.8, 0.8]);   % smorzamento viscoso
F_ext = [0; 0];             % forze esterne (none per ora)

% Energy tanks (se vorrai abilitarle dopo)
use_energy_tanks = false;
xt0              = 1.0;     % energia iniziale nel serbatoio (placeholder)
eps_sync_lambda  = 1e-3;    % soglia per mismatch di sincronizzazione
Et   = ones(N_max,1) * 1.0;   % energia iniziale per ciascun agente
Emin = 0.1;                   % minimo consentito
Emax = 0.5;                   % massimo consentito


%% ---------------  CONNECTIVITY WEIGHTS (gamma, alpha, beta, fov) --------------- 
% Raggio di comunicazione (eq. (8))
Dgamma   = 2.3;    % MAX range (oltre è 0)
dgamma   = 1;    % inizio tapering (entro dgamma vale 1)

% Collision avoidance (eq. (10)–(11))
dAlpha_min = 0.2; % lower bound (distanza di sicurezza "hard")
Dalpha     = 0.5;  % upper bound della finestra alpha

% Formazione "soft" (eq. (9))
dBeta_des  = 0.9;      % distanza desiderata
sigmaBeta  = (0.8)^2;  % larghezza del "bump" gaussiano

% Field-of-View (eq. (12)–(13))
fov_r = 2.3; % distanza della circonferenza entro il quale conosce il vicino

%% ---------------  BICONNECTIVITY CONTROLLER --------------- 
% Potenziale e guadagno (eq. (15), (18)–(19))
kappa        = 1.0;    % guadagno del termine F_lambda
lambda_bar   = 0.1;    % target inferiore per biconnettività
eps_lambda   = 0.02;   % epsilon piccolo per evitare singolarità

% Perturbazione e soglie (Sec. III-B, eq. (5)–(7))
delta                 = 1e-2;    % ampiezza perturbazione
sigma_lambda_local    = 1e-6;    % soglia "local biconnectivity"
rho_min               = 0.0;     % bound inferiore per variabile locale rho_i
rho_max               = 1.0 - delta; % bound superiore coerente con il modello

% Saturazioni/limiti di sicurezza
lambda_star_min = 0.0;                       % lower bound per lambda*
lambda_star_max = lambda_bar + eps_lambda;   % upper bound per lambda* dinamico
force_sat_max   = 5.0;                       % saturazione modulo forza di controllo (placeholder)

%% --------------- TARGET ADAPTATION  --------------- 
delta        = 0.02;
k1           = 1.0;
k2           = (1 - delta) * k1;
sigma_lambda = 1e-3;
dt           = 1.0;
rho0         = 0.0;   % inizializzazione per gli attivi

%% --------------- NUMERICAL / TOLERANCES / LOGGING --------------- 
eps_dist         = 1e-9;   % tolleranza per distanze ~0
eps_graph        = 1e-9;   % tolleranza weight ~0
log_every_step   = true;   % salva log ad ogni step
save_A_matrices  = true;   % salva A e A_tilde
save_min_dist    = true;   % salva min d_ij
save_lambda2     = true;   % salva lambda_2 perturbata e/o stimata


%% --------------- initialization ---------------
% X: matrice N x 2 con le posizioni correnti degli agenti attivi (definiscila prima).
% Esempio placeholder: X = zeros(N0,2);  % <-- solo per test rapido

l = 1;
robotPosition = [-l, -l; l, -l; l, l; -l, l; 0, 0];
A = buildAdjacency(robotPosition, Dgamma, dgamma, dBeta_des, sigmaBeta, dAlpha_min, Dalpha, fov_r );

%% --------------- main ---------------
initRobotPosition = robotPosition;
history_robot_position = nan(numel(t_grid), 5, 2);
history_lambda2 = nan(numel(t_grid), 1);
history_lambda2_tilde = nan(numel(t_grid), 1);
rho_prev = ones(N_max, 1) * lambda_bar;
lambda_bar_vec = ones(N_max, 1) * lambda_bar;
lambda2_local = ones(N_max, 1);
k = 1;
eps = 1e-3;

event_select = 1;
for k = 1:numel(t_grid)
    t = t_grid(k);

    if t > 0
        [robotPosition, initRobotPosition] = update_dynamics(robotPosition, initRobotPosition, t, dt,N_max, Et, Emin, Emax, lambda2_local, lambda_bar_vec, k, eps);
    end

    % --- evento a questo istante?
    if event_select <= numel(event_time) && t == event_time(event_select)
        ids = event_agent{event_select};
        if event_type(event_select) > 0
            % ADD: riattiva questi ID alle loro posizioni iniziali
            for id = ids
                robotPosition(id, :) = initRobotPosition(id, :);
            end
        else
            % REMOVE: disattiva questi ID -> NaN
            for id = ids
                robotPosition(id, :) = [NaN, NaN];
            end
        end
        event_select = event_select + 1;
    end

    % --- costruisci A sugli attivi (righe non-NaN)
    idxActive = find(all(~isnan(robotPosition), 2));
    A = buildAdjacency(robotPosition(idxActive, :), Dgamma, dgamma, dBeta_des, sigmaBeta, dAlpha_min, Dalpha, fov_r);

     % --- Laplaciana e lambda2
    L = buildLaplacian(A);
    lambda2 = computeSecondSmallestEigenvalue(L);
    history_lambda2(k) = lambda2;

    % rho, eps, E e A_tilde
    %[rho, eps, E] = computeRho(robotPosition, delta, idxActive);
    [rho_act, eps_act, E, lambda2_local_act] = computeRho( robotPosition, rho_prev(idxActive), A, k1, k2, sigma_lambda, delta, dt, idxActive);
    rho_prev(idxActive) = rho_act;
    lambda2_local = lambda_bar * ones(N_max, 1);   % inattivi = lambda_bar
    lambda2_local(idxActive) = lambda2_local_act;  % attivi = valori calcolati

    Atilde = buildPerturbedAdjacency(A, E);
    Ltilde  = buildLaplacian(Atilde);
    lam2til = computeSecondSmallestEigenvalue(Ltilde);
    history_lambda2_tilde(k) = lam2til;

    % --- log posizioni
    history_robot_position(k, :, :) = robotPosition;

    % --- plot live (figure separate per chiarezza)
    figure(1);
    plot_formation_live(robotPosition, t);
    %plot_lambda2_live(t_grid(1:k), history_lambda2(1:k));
    plot_lambda2_live(t_grid(1:k), history_lambda2_tilde(1:k))

    pause(0.1);
end

%% --------------- plot finale ---------------
plot_formation(t_grid, history_robot_position, event_time);
plot_lambda2_history(t_grid, history_lambda2);
plot_lambda2_history(t_grid, history_lambda2_tilde);

%% --------------- Auxiliar function ---------------
% Costruisce la matrice di adiacenza A (N x N) a partire dalle posizioni X (N x 2)
% usando i pesi: a_ij = gamma * beta * alpha * f.
function A = buildAdjacency(X, Dgamma, dgamma, dBeta_des, sigmaBeta, dAlpha_min, Dalpha, fov_r)
    N = size(X,1);
    A = zeros(N);

    for i = 1:N-1
        for j = i+1:N
            dij = norm(X(i,:) - X(j,:));

            g  = gammaWeight(dij, Dgamma, dgamma);
            b  = betaWeight(dij, dBeta_des, sigmaBeta);
            av = alphaWeight(dij, dAlpha_min, Dalpha);
            f  = fovWeight(dij, fov_r);

            aij = g * b * av * f;   % peso finale
            A(i,j) = aij;
            A(j,i) = aij;
        end
    end
end

% --- gamma(d): raggio di comunicazione con tapering coseno tra dgamma e Dgamma
function g = gammaWeight(d, Dgamma, dgamma)
    if d >= Dgamma
        g = 0.0;
    elseif d <= dgamma
        g = 1.0;
    else
        tau = (d - dgamma) / (Dgamma - dgamma);   % in [0,1]
        g   = 0.5 * (1 + cos(pi * tau));          % 1 -> 0 in modo smooth
    end
end

% --- beta(d): “soft formation” con bump gaussiano centrato in dBeta_des
function b = betaWeight(d, dBeta_des, sigmaBeta)
    b = exp(- ((d - dBeta_des)^2) / sigmaBeta);
end

% --- alpha(d): collision avoidance: 0 fino a dAlpha_min, poi smooth 0->1 fino a Dalpha
function a = alphaWeight(d, dAlpha_min, Dalpha)
    if d <= dAlpha_min
        a = 0.0;
    elseif d >= Dalpha
        a = 1.0;
    else
        tau = (d - dAlpha_min) / (Dalpha - dAlpha_min);  % in [0,1]
        a   = 0.5 * (1 - cos(pi * tau));                 % smooth ramp 0->1
    end
end

% --- f(d): FOV semplificato basato su raggio (visibile solo entro fov_r)
function f = fovWeight(d, fov_r)
    if d <= fov_r
        f = 1.0;
    else
        f = 0.0;
    end
end

function plot_formation(t_grid, history_robot_position, event_time)
    % Snapshot ai tempi chiave: inizio, eventi, fine
    snap_times = unique([t_grid(1), event_time(:).', t_grid(end)]);
    nSnaps = numel(snap_times);

    % Limiti assi globali (escludendo NaN)
    Xall = history_robot_position(:,:,1); Yall = history_robot_position(:,:,2);
    xmin = min(Xall(~isnan(Xall))); xmax = max(Xall(~isnan(Xall)));
    ymin = min(Yall(~isnan(Yall))); ymax = max(Yall(~isnan(Yall)));
    pad = 0.2 * max([xmax - xmin, ymax - ymin, 1]);  % un po' di margine
    xl = [xmin - pad, xmax + pad]; yl = [ymin - pad, ymax + pad];

    % Griglia compatta
    nrows = ceil(sqrt(nSnaps)); ncols = ceil(nSnaps / nrows);
    tiledlayout(nrows, ncols, "Padding","compact","TileSpacing","compact");

    for s = 1:nSnaps
        t = snap_times(s);
        % indice esatto nella griglia temporale (dt=1, tempi interi)
        k = find(t_grid == t, 1, 'first');
        nexttile; hold on; axis equal; grid on;
        title(sprintf('t = %g', t));
        xlim(xl); ylim(yl);
        % plotta SOLO agenti attivi (non-NaN) a questo istante
        XY = squeeze(history_robot_position(k, :, :));  % 5 x 2
        active = all(~isnan(XY), 2);
        plot(XY(active,1), XY(active,2), 'o', 'MarkerFaceColor','auto', 'MarkerSize',7);
        % etichette ID
        ids = find(active);
        for ii = 1:numel(ids)
            text(XY(ids(ii),1), XY(ids(ii),2), sprintf('  %d', ids(ii)), ...
                'VerticalAlignment','middle','FontSize',9);
        end
    end
end

function plot_formation_live(robotPosition, t)
    clf; hold on; axis equal; grid on;
    title(sprintf('t = %d', t));
    xlabel('x'); ylabel('y');

    % plotta solo gli attivi
    active = all(~isnan(robotPosition), 2);
    XY = robotPosition(active, :);

    plot(XY(:,1), XY(:,2), 'o', 'MarkerFaceColor','auto', 'MarkerSize',7);

    % etichette ID
    ids = find(active);
    for ii = 1:numel(ids)
        text(XY(ii,1), XY(ii,2), sprintf('  %d', ids(ii)), ...
            'VerticalAlignment','middle','FontSize',9);
    end

    xlim([-3 20]); ylim([-3 3]); % range fisso per non far saltare la scala
    drawnow;
end

% --- Laplaciana dal grafo pesato A
function L = buildLaplacian(A)
    d = sum(A, 2);
    L = diag(d) - A;
end

% --- Seconda autovalore più piccola (algebraic connectivity)
function lambda2 = computeSecondSmallestEigenvalue(L)
    n = size(L,1);
    if n < 2
        lambda2 = NaN;  % non definita con <2 nodi
        return;
    end
    % simmetrizza numericamente e prendi parte reale
    S = (L + L')/2;
    ev = sort(real(eig(S)), 'ascend');
    lambda2 = ev(2);    % se grafo disconnesso -> tipicamente 0
end

% --- Plot live di lambda2 (finestra separata)
function plot_lambda2_live(t_vec, lambda2_vec)
    figure(2); clf; hold on; grid on;
    plot(t_vec, lambda2_vec, '-', 'LineWidth', 1.5, 'Marker','o');
    xlabel('t'); ylabel('\lambda_2 (L)');
    title('Algebraic connectivity (second smallest eigenvalue)');

    % limiti automatici ma robusti
    if numel(t_vec) > 1
        xlim([t_vec(1), t_vec(end)]);
    else
        xlim([t_vec(1)-1, t_vec(1)+1]);
    end
    ylim auto;
    drawnow;
end

function plot_lambda2_history(t_grid, history_lambda2)
    figure; hold on; grid on;
    plot(t_grid, history_lambda2, 'LineWidth', 1.5, 'Marker','o');
    xlabel('t'); ylabel('\lambda_2 (L)');
    title('Andamento della seconda autovalore più piccola della Laplaciana');
    ylim padded; % aggiunge margine automatico in verticale
end

function [rho, eps, E] = computeRhoOld(robotPosition, delta, idxActive)
    % Numero di agenti attivi
    Nactive = numel(idxActive);

    % rho solo sugli attivi
    rho = zeros(Nactive,1);          % attivi -> rho = 0

    % eps solo sugli attivi
    eps = 1 - rho;                   % qui eps = 1 per tutti

    % Matrice E solo sugli attivi (Nactive x Nactive)
    E = min(eps, eps.');
end

function [rho, eps, E, lambda2_local] = computeRho(robotPosition, rho_prev, A_active, k1, k2, sigma_lambda, delta, dt, idxActive)

    Nact = size(A_active,1);

    lambda2_local = zeros(Nact,1);
    for i = 1:Nact
        Ni = find(A_active(i,:) > 0);
        Ni(Ni == i) = [];                  % esclude i (di norma già escluso)
        if numel(Ni) >= 2
            A_loc = A_active(Ni, Ni);
            L_loc = buildLaplacian(A_loc);
            lambda2_local(i) = computeSecondSmallestEigenvalue(L_loc);
        else
            lambda2_local(i) = 0;          % non biconnesso localmente
        end
    end

    % --- ingresso dinamica: 0.5*(1 + sign(sigma - λ²_loc))
    u = 0.5 * (1 + sign(sigma_lambda - lambda2_local));
    % (equivalente a 1{λ²_loc < sigma} ma gestisce bene il caso =sigma)

    % --- integrazione esplicita di rho con clamp in [0, 1 - delta]
    drho = -k1 .* rho_prev + k2 .* u;
    rho  = rho_prev + dt .* drho;
    rho  = min(max(rho, 0), 1 - delta);

    % --- eps ed E sugli attivi
    eps = 1 - rho;
    E   = min(eps, eps.');
end

function Atilde = buildPerturbedAdjacency(A, E)
    Atilde = E .* A;           % simmetrica se A ed E sono simmetriche
end

function [v_safe, Et_new] = tank_update_vel(v_cmd, Et, dt, Emin, Emax)
% Scala la velocità comandata v_cmd per non far scendere il serbatoio Et sotto Emin.
% v_cmd: scalare o vettore riga/colonna (es. [vx vy])
% Et:    energia corrente del tank (scalare)
% dt:    passo
% Emin, Emax: limiti del tank
%
% Output:
% v_safe: velocità (stessa forma di v_cmd) dopo scaling
% Et_new: energia aggiornata

    % Assicura vettore colonna per il calcolo, poi ripristina la forma
    origSize = size(v_cmd);
    v = v_cmd(:);

    % "potenza" ~ ||v||^2
    p = sum(v.^2);
    Et_new = Et - p*dt;

    if Et_new < Emin && p > 0
        % Fattore minimo per non scendere sotto Emin
        s = max(0, min(1, (Et - Emin) / (p*dt)));
        v = s * v;
        % Ricalcola drenaggio con v scalata
        p2 = sum(v.^2);
        Et_new = Et - p2*dt;
    end

    % Clip superiore
    if Et_new > Emax
        Et_new = Emax;
    end

    % Ripristina forma originale
    v_safe = reshape(v, origSize);
end

function [robotPosition, initRobotPosition] = update_dynamics(robotPosition, initRobotPosition, t, dt, N_max, Et, Emin, Emax, lambda_cur, lambda_bar, k, eps)
    % Aggiorna le posizioni: x_i <- x_i + v_i(t)*dt
    % v_i(t) = vmax * (cos(t + phi_i) + 1)
    % Solo agenti attivi (righe non-NaN). Si muove lungo x, y invariata.

    active = all(~isnan(robotPosition), 2);
    ids = find(active);

    for i = 1:N_max
        current_phase_offset = i*(2/5);
        v = cos(t + current_phase_offset) + 1;   % velocità scalare
        v = min(max(v, 0), 0.1);
        lambda_cur
        lambda_bar
        Vp = storageFunction(lambda_cur(i) + eps, lambda_bar(i));
        Vm = storageFunction(lambda_cur(i) - eps, lambda_bar(i));
        dVdl = (Vp - Vm) / (2*eps);
    
        % legge di velocità comune (puoi personalizzare per-agente se serve)
        v = -k * dVdl;
        [v_safe, Et(i)] = tank_update_vel(v, Et(i), dt, Emin, Emax);
        robotPosition(i,1) = robotPosition(i,1) + v_safe * dt;  % muovi lungo x
        initRobotPosition(i, 1) = initRobotPosition(i, 1) + v_safe * dt; 
    end
end

function dV = storageFunction(lambda_cur, lambda_bar)
    dV = coth(lambda_cur - lambda_bar) + 1;
end