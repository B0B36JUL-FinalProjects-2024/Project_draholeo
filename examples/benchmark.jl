using Revise
using PathFinder
using BenchmarkTools

include("test_environments.jl")

ps_easy = PlanningSpace(
    dim=2,
    limits=[0 1; 0 1],
    collision_check=conf -> check_collision(conf, easy_env),
    collision_resolution=0.01)
ps_hard = PlanningSpace(
    dim=2,
    limits=[0 1; 0 1],
    collision_check=conf -> check_collision(conf, hard_env),
    collision_resolution=0.01)

rrt = RRTPlanner(step_size=0.05, max_iters=10000, tolerance=0.05)
prm = PRMPlanner(roadmap_size=200, k_neighbors=8)

# RRT on easy env 
@benchmark plan([0.1, 0.5], [0.9, 0.5], ps_easy, rrt)

# RRT on hard env
@benchmark plan([0.1, 0.1], [0.9, 0.9], ps_hard, rrt)

# PRM on easy env
@benchmark plan([0.1, 0.5], [0.9, 0.5], ps_easy, prm)

# PRM on hard env
@benchmark plan([0.1, 0.1], [0.9, 0.9], ps_hard, prm)
