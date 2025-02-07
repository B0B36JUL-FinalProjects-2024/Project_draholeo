using Revise
using PathFinder

include("test_environments.jl")

# rrt algorithm in the easy_env
ps = PlanningSpace(2, [0 1; 0 1], conf -> check_collision(conf, easy_env), 0.1)
planner = RRTPlanner(step_size = 0.05, max_iters = 10000, tolerance = 0.05)
path, tree = plan([0.1, 0.5], [0.9, 0.5], ps, planner)
plot(easy_env, path, tree)
savefig("./examples/results/rrt_easy.svg")

# rrt algorithm in the hard_env
ps = PlanningSpace(2, [0 1; 0 1], conf -> check_collision(conf, hard_env), 0.1)
planner = RRTPlanner(step_size = 0.05, max_iters = 10000, tolerance = 0.05)
path, tree = plan([0.1, 0.1], [0.9, 0.9], ps, planner)
plot(hard_env, path, tree)
savefig("./examples/results/rrt_hard.svg")
