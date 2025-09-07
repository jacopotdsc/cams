% This code is inspired by the paper [METTI LINK] to simulate the biconnectivity of a graph.
% Assumptions:
% - FOV is not a frontal camera with limited aperture, but agents sense all within a given radius.
% - When computing biconnectivity, lambda is not defined for N = 1 agent in the graph.
% - For subgraphs that are not feasible, lambda_i_local is set to 0.


%% --------------- SIMULATION PARAMETERS --------------- 
T_end      = 60;          % Simulation duration
dt         = 1.0;         % Simulation step
t_grid     = 0:dt:T_end;  % griglia temporale
rng_seed   = 42;          % seed for reproducibility

%% --------------- OMRS / TEAM SCHEDULING --------------- 
use_fixed = false;
formation_type = 1; % 1, 2, 3
l = 1.5; % 1, 1.2, 1.5 
formations = {[-l, -l; l, -l; l, l; -l, l; 0, 0], [ 0, 0; 0, -l; 0, l; l, 0; -l, 0], [-l, -l; l, -l; l, l; -l, l; 0, 0; 0, -l; 0, l; l, 0; -l, 0] };
robotPosition = formations{formation_type};
robotPosition = spawnAgents(20, 1);

N_max = size(robotPosition, 1);   
if use_fixed == false
    max_event = 5;
    max_change = 3;
    [event_time, event_type, event_agent, history_n_agent] = gen_random_events(N_max, rng_seed, T_end, max_event, max_change);
else
    event_time = [20, 30, 40]; % times for which something happend
    event_type = [-2, 1, 1 ]; % what happen to the number of agents
    event_agent = { [1,3], [1], [3] };  % what agent are added or removed
    history_n_agent = zeros(1,T_end);
    history_n_agent(1:19)   = 5;  
    history_n_agent(20:29)  = 3; 
    history_n_agent(30:39)  = 4;  
    history_n_agent(40:end) = 5;  
end

%% ---------------  CONNECTIVITY WEIGHTS (gamma, alpha, beta, fov) --------------- 
% Communication range (eq. (8))
Dgamma   = 2.3;  % MAX range (beyond is 0 )
dgamma   = 1;    % Min range ( within is 1 )

% Collision avoidance (eq. (10)–(11))
dAlpha_min = 0.2;  % lower bound ( security distance )
Dalpha     = 0.5;  % upper bound ( beyond is 1 )

% Formazione "soft" (eq. (9))
dBeta_des  = 0.9;      % desired distance
sigmaBeta  = (0.8)^2;  % gaussian width

% Field-of-View (eq. (12)–(13))
fov_r = 2; % radius of the circunference

%% ---------------  BICONNECTIVITY CONTROLLER --------------- 
% Potenziale e guadagno (eq. (15), (18)–(19))
lambda_bar   = 0.1;    % target inferiore per biconnettività
eps_lambda   = 0.02;   % epsilon piccolo per evitare singolarità

% Perturbazione e soglie (Sec. III-B, eq. (5)–(7))
delta                 = 1e-2;    % ampiezza perturbazione
sigma_lambda_local    = 1e-6;    % soglia "local biconnectivity"

% Saturazioni/limiti di sicurezza
lambda_star_min = 0.0;                       % lower bound per lambda*
lambda_star_max = lambda_bar + eps_lambda;   % upper bound per lambda* dinamico
force_sat_max   = 5.0;                       % saturazione modulo forza di controllo (placeholder)

%% --------------- TARGET ADAPTATION  --------------- 
delta        = 0.02;
k1           = 1.0;
k2           = (1 - delta) * k1;
sigma_lambda = 1e-3;
rho0         = 0.0;   % inizializzazione per gli attivi

%% --------------- main ---------------
initRobotPosition = robotPosition;
history_robot_position = nan(numel(t_grid), N_max, 2);
history_lambda2 = nan(numel(t_grid), 1);
history_lambda2_tilde = nan(numel(t_grid), 1);
rho_prev = ones(N_max, 1) * lambda_bar;
lambda_bar_vec = ones(N_max, 1) * lambda_bar;
lambda2_local = ones(N_max, 1);
eps = 1e-3;

event_select = 1;
for k = 1:numel(t_grid)
    t = t_grid(k);

    if t > 0
        [robotPosition, initRobotPosition] = update_dynamics(robotPosition, initRobotPosition, t, dt,N_max);
    end

    if event_select <= numel(event_time) && t == event_time(event_select)
        ids = event_agent{event_select};
        if event_type(event_select) > 0
            % Agent join
            for id = ids
                robotPosition(id, :) = initRobotPosition(id, :);
            end
        else
            % Agent remove
            for id = ids
                robotPosition(id, :) = [NaN, NaN];
            end
        end
        event_select = event_select + 1;
    end

    % --- Builing matrix on active robots
    idxActive = find(all(~isnan(robotPosition), 2));
    A = buildAdjacency(robotPosition(idxActive, :), Dgamma, dgamma, dBeta_des, sigmaBeta, dAlpha_min, Dalpha, fov_r);

     % --- Laplacian and lambda2
    L = buildLaplacian(A);
    lambda2 = computeSecondSmallestEigenvalue(L);
    history_lambda2(k) = lambda2;

    % --- rho, eps, E, A_tilde
    [rho_act, eps_act, E, lambda2_local_act] = computeRho(rho_prev(idxActive), A, k1, k2, sigma_lambda, delta, dt);
    rho_prev(idxActive) = rho_act;
    lambda2_local = lambda_bar * ones(N_max, 1);   % inactives = lambda_bar
    lambda2_local(idxActive) = lambda2_local_act;  % actives = computed values

    Atilde = buildPerturbedAdjacency(A, E);
    Ltilde  = buildLaplacian(Atilde);
    lam2til = computeSecondSmallestEigenvalue(Ltilde);

    history_lambda2_tilde(k) = lam2til;

    % --- log position
    history_robot_position(k, :, :) = robotPosition;

    % --- live plot
    %plot_formation_live(robotPosition, t, event_time, event_type, event_agent, 1);
    %plot_data(t_grid(1:k), history_lambda2_tilde(1:k), 'lambda_tilde', 2)

    %pause(0.1);
end

%% --------------- plot finale ---------------
if ~exist('plot','dir')
    mkdir('plot');
end

subfolder = fullfile('plot', sprintf('formation_%d_%.2f', formation_type, l));
if ~exist(subfolder,'dir')
    mkdir(subfolder);
end

plot_formation(t_grid, history_robot_position, event_time, 2); saveas(gcf, fullfile(subfolder,'formation.png'));
plot_data(t_grid, history_lambda2_tilde, 'lamba tilde', 3); saveas(gcf, fullfile(subfolder,'lambda2tilde.png'));
plot_data(1:T_end, history_n_agent, 'agents', 4); saveas(gcf, fullfile(subfolder,'agents.png'));

%% --------------- Auxiliar function ---------------
function A = buildAdjacency(X, Dgamma, dgamma, dBeta_des, sigmaBeta, dAlpha_min, Dalpha, fov_r)
    N = size(X,1);
    A = zeros(N);

    for i = 1:N
        for j = 1:N
            dij = norm(X(i,:) - X(j,:));

            g  = gammaWeight(dij, Dgamma, dgamma);
            b  = betaWeight(dij, dBeta_des, sigmaBeta);
            av = alphaWeight(dij, dAlpha_min, Dalpha);
            f  = fovWeight(dij, fov_r);

            aij = g * b * av * f;  

            A(i,j) = aij;
            A(j,i) = aij;
        end
    end
end

function g = gammaWeight(d, Dgamma, dgamma)
    if d >= Dgamma
        g = 0.0;
    elseif d <= dgamma
        g = 1.0;
    else
        g = 0.5;
    end
end

function b = betaWeight(d, dBeta_des, sigmaBeta)
    b = exp(- ((d - dBeta_des)^2) / sigmaBeta);
end

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

function f = fovWeight(d, fov_r)
    if d <= fov_r
        f = 1.0;
    else
        f = 0.0;
    end
end

function L = buildLaplacian(A)
    d = sum(A, 2);
    L = diag(d) - A;
end

function lambda2 = computeSecondSmallestEigenvalue(L)
    n = size(L,1);
    if n < 2
        lambda2 = NaN;  % not defined for <2 nodi
        return;
    end
    
    S = (L + L')/2; % simmetring for numerical errore
    ev = sort(real(eig(S)), 'ascend');
    lambda2 = ev(2); 
end

function [rho, eps, E, lambda2_local] = computeRho(rho_prev, A_active, k1, k2, sigma_lambda, delta, dt)

    Nact = size(A_active,1);
    lambda2_local = zeros(Nact,1);
    
    for i = 1:Nact     
        idx = true(Nact,1);
        idx(i) = false;
        A_loc = A_active(idx, idx);
        if size(A_loc,1) >= 3
            L_loc = buildLaplacian(A_loc);
            lambda2_local(i) = computeSecondSmallestEigenvalue(L_loc);
        else
            lambda2_local(i) = 0;   
        end
    end

    % --- rho dynamics: 0.5*(1 + sign(sigma - λ²_loc))
    u = 0.5 * (1 + sign(sigma_lambda - lambda2_local));

    % --- explicit integration of rho with clamp in [0, 1 - delta]
    drho = -k1 .* rho_prev + k2 .* u;
    rho  = rho_prev + dt .* drho;
    rho  = min(max(rho, 0), 1 - delta);

    % --- computing epsilon
    eps = 1 - rho;
    E   = min(eps, eps.');
end

function Atilde = buildPerturbedAdjacency(A, E)
    Atilde = E .* A;           % simmetrica se A ed E sono simmetriche
end

function [robotPosition, initRobotPosition] = update_dynamics(robotPosition, initRobotPosition, t, dt, N_max)

    active = all(~isnan(robotPosition), 2);
    ids = find(active);

    for i = 1:N_max
        current_phase_offset = i*(2/5);
        v = cos(t + current_phase_offset) + 1;  
        v_safe = 0.0; % min(max(v, 0), 0.1);
        robotPosition(i,1) = robotPosition(i,1) + v_safe * dt;  % move along x
        initRobotPosition(i, 1) = initRobotPosition(i, 1) + v_safe * dt; 
    end
end

function [event_time, event_type, event_agent, history_n_agent] = gen_random_events(N_max, rng_seed, T_end, event_number, max_change)
    rng(rng_seed);

    % event times (unique integers between 1 and T_end)
    event_time = sort(randperm(T_end, event_number));

    % initial state: all agents active
    active   = 1:N_max;
    inactive = [];

    event_type  = zeros(1, event_number);
    event_agent = cell(1, event_number);
    history_n_agent = zeros(1, T_end);
    history_n_agent(1:end) = N_max;
    current_agents = N_max;

    % process events
    for i = 1:event_number
        canRemove = numel(active) > 1;      % keep at least one active
        canAdd    = numel(active) < N_max;  % cannot exceed N_max

        % choose action size and direction
        if ~canAdd && canRemove
            action = -randi(max_change);
        elseif ~canRemove && canAdd
            action = +randi(max_change);
        else
            if rand < 0.5
                action = -randi(max_change);
            else
                action = +randi(max_change);
            end
        end

        current_agents = current_agents + action;
        current_agents = max(min(current_agents, N_max), 0);
        history_n_agent(event_time(i):end) = current_agents;

        ids_changed = [];

        if action < 0
            % removal
            k = min(abs(action), numel(active)-1); % at least 1 remains
            if k > 0
                ids_changed = randsample(active, k);
                active(ismember(active, ids_changed)) = [];
                inactive = [inactive, ids_changed];
            end
            event_type(i) = -numel(ids_changed);

        elseif action > 0
            % addition
            k = min(abs(action), numel(inactive)); % cannot exceed available
            if k > 0
                ids_changed = randsample(inactive, k);
                inactive(ismember(inactive, ids_changed)) = [];
                active = [active, ids_changed];
            end
            event_type(i) = +numel(ids_changed);
        end

        event_agent{i} = ids_changed;
    end


end

function positions = spawnAgents(numAgents, radius)

    theta = 2*pi*rand(numAgents,1);        %
    r = radius * sqrt(rand(numAgents,1)); 

    x = r .* cos(theta);
    y = r .* sin(theta);

    positions = [x, y];
end


function plot_formation(t_grid, history_robot_position, event_time, fig_id)
    figure(fig_id);
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

function plot_formation_live(robotPosition, t, event_time, event_type, event_agent, fig_id)
    figure(fig_id);
    clf; hold on; axis equal; grid on;
    xlabel('x'); ylabel('y');

    str_time  = sprintf('event\\_time  = %s', mat2str(event_time));
    str_type  = sprintf('event\\_type  = %s', mat2str(event_type));
    str_agent = sprintf('event\\_agent = %s', strjoin(cellfun(@mat2str, event_agent, 'UniformOutput', false), ', '));

    title({sprintf('t = %.1f', t), str_time, str_type, str_agent}, 'FontSize', 9);

    % plot only active agents
    active = all(~isnan(robotPosition), 2);
    XY = robotPosition(active, :);

    plot(XY(:,1), XY(:,2), 'o', 'MarkerFaceColor','auto', 'MarkerSize',7);

    ids = find(active);
    for ii = 1:numel(ids)
        text(XY(ii,1), XY(ii,2), sprintf('  %d', ids(ii)), ...
            'VerticalAlignment','middle','FontSize',9);
    end

    xlim([-3 3]); ylim([-3 3]); 
    drawnow;
end

function plot_data(t_vec, lambda2_vec, data_type, fig_id)
    figure(fig_id); clf; hold on; grid on;
    plot(t_vec, lambda2_vec, '-', 'LineWidth', 1.5, 'Marker','o');
    xlabel('t'); ylabel(data_type);
    title(data_type);

    if numel(t_vec) > 1
        xlim([t_vec(1), t_vec(end)]);
    else
        xlim([t_vec(1)-1, t_vec(1)+1]);
    end
    yl = ylim;              
    ylim([0, yl(2)]);        
    drawnow;
end
