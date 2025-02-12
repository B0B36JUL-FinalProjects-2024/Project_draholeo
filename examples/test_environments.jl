"""
2 simple environments for a circular robot in 2D.
"""

using SeparatingAxisTheorem2D
import Plots: plot, plot!
using Plots
using PathFinder: Tree, Graph, get_nodes, AbstractGraph, TreeNode, Node

OBS_COLOR = RGB(222 / 255, 196 / 255, 132 / 255)
GRAPH_COLOR = RGB(87 / 255, 126 / 255, 137 / 255)
PATH_COLOR = RGB(225 / 255, 163 / 255, 111 / 255)

struct Env
    x_max::Float64
    y_max::Float64
    robot_radius::Float64
    obstacles::Vector{Polygon}
end

# TEST ENVIRONMENTS

easy_env = Env(1.0, 1.0, 0.02, [
    Polygon([0.29, 0.56], [0.32, 0.37], [0.49, 0.29], [0.66, 0.35], [0.70, 0.52], [0.58, 0.69], [0.42, 0.68])])

hard_env = Env(1.0, 1.0, 0.02, [
    Polygon([0.35, 0.11], [0.53, 0.06], [0.45, 0.22]),
    Polygon([0.12, 0.30], [0.26, 0.37], [0.16, 0.48]),
    Polygon([0.69, 0.22], [0.82, 0.16], [0.90, 0.40]),
    Polygon([0.50, 0.90], [0.66, 0.71], [0.88, 0.73]),
    Polygon([0.14, 0.65], [0.35, 0.85], [0.14, 0.80]),
    Polygon([0.35, 0.72], [0.30, 0.55], [0.47, 0.70]),
    Polygon([0.43, 0.42], [0.57, 0.33], [0.73, 0.42], [0.70, 0.58], [0.50, 0.60])
])

# COLLISION DETECTION

function check_collision(conf, env::Env)
    for obs in env.obstacles
        if intersecting(Circle(conf, env.robot_radius), obs)
            return true
        end
    end

    return false
end

# PLOTTING FUNCTIONS

function plot!(obs::Polygon)
    vertices = Vector(obs.points)
    xs = [v[1] for v in vertices]
    ys = [v[2] for v in vertices]
    plot!(xs, ys, seriestype=:shape, fillcolor=OBS_COLOR, linewidth=0)
end

function plot(env::Env)
    plot([], [], axis=false, grid=false, legend=false, xlims=(0, env.x_max), ylims=(0, env.y_max),
        size=(2000, 2000), aspect_ratio=1) # empty plot
    for obs in env.obstacles
        plot!(obs)
    end
end

function plot!(node::TreeNode; width=9)
    scatter!([node.coords[1]], [node.coords[2]], color=GRAPH_COLOR, markerstrokewidth=0, markersize=1.5 * width)
    for child in node.children
        plot!([node.coords[1], child.coords[1]], [node.coords[2], child.coords[2]], lw=width, color=GRAPH_COLOR)
    end
end

function plot!(node::Node; width=9)
    scatter!([node.coords[1]], [node.coords[2]], color=GRAPH_COLOR, markerstrokewidth=0, markersize=1.5 * width)
    for n in node.neighbors
        plot!([node.coords[1], n.coords[1]], [node.coords[2], n.coords[2]], lw=width, color=GRAPH_COLOR)
    end
end

function plot!(graph::AbstractGraph; width=9)
    for node in get_nodes(graph)
        plot!(node; width)
    end
end

function plot!(path::Matrix{Float64}; width=9)
    # start (triangle) and goal (star) node
    scatter!([path[1, 1]], [path[2, 1]], markershape=:circle, color=PATH_COLOR, markerstrokewidth=0, markersize=2 * width)
    scatter!([path[1, end]], [path[2, end]], markershape=:star5, color=PATH_COLOR, markerstrokewidth=0, markersize=3 * width)
    # other nodes
    scatter!(path[1, 2:end-1], path[2, 2:end-1], color=PATH_COLOR, markerstrokewidth=0, markersize=1.5 * width)
    for i in 1:size(path, 2)-1
        plot!([path[1, i], path[1, i+1]], [path[2, i], path[2, i+1]], lw=width, color=PATH_COLOR)
    end
end

function plot(env::Env, path::Union{Matrix{Float64},Nothing}, graph::AbstractGraph; width=9)
    plot(env)
    plot!(graph; width)
    if !isnothing(path)
        plot!(path; width=1.1 * width)
    end
end
