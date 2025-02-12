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
create_animation("./examples/results/rrt_easy.gif", easy_env, path, tree) # uncomment to create animation (takes a long time)

path, tree = plan([0.1, 0.1], [0.9, 0.9], ps_hard, planner)
plot(hard_env, path, tree)
savefig("./examples/results/rrt_hard.svg")
create_animation("./examples/results/rrt_hard.gif", hard_env, path, tree) # uncomment to create animation (takes a long time)

# PRM ALGORITHM ----------------------------------------------------------

planner = PRMPlanner(roadmap_size=200, k_neighbors=8)

path, graph = plan([0.1, 0.5], [0.9, 0.5], ps_easy, planner)
plot(easy_env, path, graph)
savefig("./examples/results/prm_easy.svg")
create_animation("./examples/results/prm_easy.gif", easy_env, path, graph) # uncomment to create animation (takes a long time)

path, graph = plan([0.1, 0.1], [0.9, 0.9], ps_hard, planner)
plot(hard_env, path, graph)
savefig("./examples/results/prm_hard.svg")
create_animation("./examples/results/prm_hard.gif", hard_env, path, graph) # uncomment to create animation (takes a long time)

# EMPTY ENV VISUALIZATION

plot(easy_env; start=[0.1, 0.5], goal=[0.9, 0.5])
savefig("./examples/results/easy_env.svg")

plot(hard_env; start=[0.1, 0.1], goal=[0.9, 0.9])
savefig("./examples/results/hard_env.svg")
