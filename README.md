# Analysis and Control of Multi-Robot Systems

This repository provides MATLAB code to **simulate biconnectivity levels** in multi-agent formations, inspired by the approach described in the paper:  
👉 *[https://ieeexplore.ieee.org/abstract/document/10886012]([https://ieeexplore.ieee.org/abstract/document/10886012]*)*

The simulation models how the **algebraic connectivity** (\(\lambda_2\)) and the team formation evolve when agents are dynamically **added** or **removed** through discrete events. This allows testing robustness of the team’s communication graph under disturbances and scheduling changes.

---

## How it works

- Agents are initially placed in a **square formation** with grid spacing \(l\).  
- Random **events** occur over the simulation horizon:
  - **Add events**: new agents join the formation.  
  - **Remove events**: existing agents leave the formation.  
- After each event, connectivity metrics are recomputed and plotted.  
- The simulation tracks the **number of active agents over time** (`history_n_agent`), alongside connectivity values such as \(\lambda_2\).

---

## Parameters

Key parameters to configure scenarios are defined in the script:

```matlab
%% --------------- OMRS / TEAM SCHEDULING --------------- 
N_max      = 20;   % total number of agents available (global IDs)
max_event  = 10;   % number of events to generate
max_change = 10;   % maximum number of agents added/removed in one event

[event_time, event_type, event_agent, history_n_agent] = ...
    gen_random_events(N_max, rng_seed, T_end, max_event, max_change);
