using LinearAlgebra

Base.@kwdef struct RRTPlanner <: AbstractPlanner
    step_size::Float64 = 0.1
    max_iters::Int64 = 10000
    tolerance::Float64 = 0.05
end

steer(nearest, random, planner::RRTPlanner) = nearest + (random - nearest) * planner.step_size / norm(random - nearest)

function get_result_path(node)
    path = node.coords
    while !isnothing(node.parent)
        path = hcat(path, node.parent)
        node = node.parent
    end
    reverse!(path, dims=2)
    return path
end

function plan(start, goal, ps::PlanningSpace, planner::RRTPlanner)
    tree = Tree(start)

    for i in 1:planner.max_iters
        random_conf = rand(ps.dim)
        nearest_node = nn_search(tree, random_conf)
        new_conf = steer(nearest_node.coords, random_conf, planner)

        if !line_collides(nearest_node.coords, new_conf, ps)
            new_node = add_node!(tree, nearest_node, new_conf)
            if norm(new_conf - goal) <= tolerance
                return get_result_path(new_node), tree
            end
        end
    end

    return nothing, tree
end
