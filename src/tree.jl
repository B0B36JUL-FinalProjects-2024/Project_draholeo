using AdaptiveKDTrees.KNN

struct TreeNode
    parent::Union{Nothing, TreeNode}
    children::Vector{TreeNode}
    coords::Vector{Float64}

    function TreeNode(parent::Union{Nothing, TreeNode}, coords::Vector{T}) where T <: Real
        new(parent, TreeNode[], Float64.(coords))
    end
end

function add_child!(parent::TreeNode, child::TreeNode)
    push!(parent.children, child)
end

struct Tree
    nodes::Vector{TreeNode}
    knnTree::KDTree

    function Tree(root::TreeNode)
        X = reshape(root.coords, :, 1)
        return new([root], KDTree(X))
    end
end

Tree(root_coords::Vector{T}) where T <: Real = Tree(TreeNode(nothing, root_coords))

function add_node!(tree::Tree, parent::TreeNode, coords::Vector{T}) where T <: Real
    new_node = TreeNode(parent, coords)
    push!(tree.nodes, new_node)
    add_point!(tree.knnTree, coords)
    add_child!(parent, new_node)
    return new_node
end

function nn_search(tree::Tree, query_coords::Vector{T}) where T <: Real
    ind, _ = nn(tree.knnTree, query_coords)
    return tree.nodes[ind]
end

function get_root(tree::Tree)
    if length(tree.nodes) >= 1
        return tree.nodes[1]
    end
    return nothing
end
