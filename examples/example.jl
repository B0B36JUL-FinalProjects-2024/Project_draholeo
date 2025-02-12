using Revise
using PathFinder

include("test_environments.jl")

# PLANNING SPACE DEFINITIONS ---------------------------------------------

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

# RRT ALGORITHM ----------------------------------------------------------

planner = RRTPlanner(step_size=0.05, max_iters=10000, tolerance=0.05)

path, tree = plan([0.1, 0.5], [0.9, 0.5], ps_easy, planner)
plot(easy_env, path, tree)
savefig("./examples/results/rrt_easy.svg")

path, tree = plan([0.1, 0.1], [0.9, 0.9], ps_hard, planner)
plot(hard_env, path, tree)
savefig("./examples/results/rrt_hard.svg")

# PRM ALGORITHM ----------------------------------------------------------

planner = PRMPlanner(roadmap_size=200, k_neighbors=8)

path, graph = plan([0.1, 0.5], [0.9, 0.5], ps_easy, planner)
plot(easy_env, path, graph)
savefig("./examples/results/prm_easy.svg")
path, graph = plan([0.1, 0.1], [0.9, 0.9], ps_hard, planner)
plot(hard_env, path, graph)
savefig("./examples/results/prm_hard.svg")

# EMPTY ENV VISUALIZATION

plot(easy_env)
savefig("./examples/results/easy_env.svg")

plot(hard_env)
savefig("./examples/results/hard_env.svg")
