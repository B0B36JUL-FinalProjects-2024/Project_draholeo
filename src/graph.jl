using AdaptiveKDTrees.KNN

abstract type AbstractGraph end

function knn_search(graph::AbstractGraph, query_coords::Vector{T}, k::Int) where {T<:Real}
    len = length(get_nodes(graph))
    if len <= 0
        return nothing
    elseif len < k
        k = len
    end

    kd = get_kd_tree(graph)
    ind, _ = knn(kd, Float64.(query_coords), k)
    return graph.nodes[ind]
end

function nn_search(graph::AbstractGraph, query_coords::Vector{T}) where {T<:Real}
    if length(get_nodes(graph)) >= 1
        return knn_search(graph, query_coords, 1)[1]
    end
    return nothing
end

struct Node
    neighbors::Vector{Node}
    coords::Vector{Float64}

    function Node(coords::Vector{T}) where {T<:Real}
        new(Node[], Float64.(coords))
    end
end

struct Graph <: AbstractGraph
    nodes::Vector{Node}
    knnTree::KDTree

    function Graph(dim::Int) # dimensionality of the C-space
        return new(Node[], KDTree(dim))
    end
end

function get_nodes(graph::Graph)
    return graph.nodes
end

function get_kd_tree(graph::Graph)
    return graph.knnTree
end

function add_node!(graph::Graph, coords::Vector{T}) where {T<:Real}
    new_node = Node(coords)
    push!(graph.nodes, new_node)
    add_point!(graph.knnTree, coords)
    return new_node
end

function add_edge!(node1::Node, node2::Node)
    push!(node1.neighbors, node2)
    push!(node2.neighbors, node1)
end
