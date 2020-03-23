This folder contains:
1. test_generation.py: Test-case generation / environment strategy synthesis based on coverage of all system action nodes. For now, contains qualitative analysis of 2-player reachability games as well.
2. reachability_games.ipynb: Correct-by-construction controller synthesis for the system using TuLiP
3. strategy_simple_runner_blocker: Controller output by TuLiP for the system
4. qualitative_analysis.py: Coming soon! Will contain qualitative analysis of stochastic games with Rabin/Streett acceptance conditions. This file will include a reduction of 21/2 player turn-based games with omega-regular acceptance conditions to 2-player turn-based games with parity objectives. The 2-player turn-based games withparity objectives will be fed into ParityGame Solver (PGSolver) for qualitative analysis. This file will also include functions to convert reachability, safety, and Buchi acceptance conditions to Rabin/Streett acceptance conditions.
