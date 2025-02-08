using LinearAlgebra

Base.@kwdef struct PRMPlanner <: AbstractPlanner
    roadmap_size::Int64 = 1000
    k_neighbors::Int64 = 12
end

function construct_roadmap(ps::PlanningSpace, planner::PRMPlanner)
    graph = Graph(ps.dim)
    while length(get_nodes(graph)) < planner.roadmap_size
        random_conf = sample_random_conf(ps)
        if !ps.collision_check(random_conf)
            add_node!(graph, random_conf)
        end
    end

    for node in get_nodes(graph)
        neighbors = knn_search(graph, node.coords, planner.k_neighbors + 1)
        for n in neighbors
            if n.coords == node.coords
                continue
            end
            if !line_collides(node.coords, n.coords, ps)
                add_edge!(node, n)
            end
        end
    end

    return graph
end

function dijkstra()
    # TODO
end

function plan(start, goal, ps::PlanningSpace, planner::PRMPlanner)
    roadmap = construct_roadmap(ps, planner)
    #path = dijkstra(roadmap)
    return nothing, roadmap
end

