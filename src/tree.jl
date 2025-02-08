using AdaptiveKDTrees.KNN

struct TreeNode
    parent::Union{Nothing,TreeNode}
    children::Vector{TreeNode}
    coords::Vector{Float64}

    function TreeNode(parent::Union{Nothing,TreeNode}, coords::Vector{T}) where {T<:Real}
        new(parent, TreeNode[], Float64.(coords))
    end
end

function add_child!(parent::TreeNode, child::TreeNode)
    push!(parent.children, child)
end

struct Tree <: AbstractGraph
    nodes::Vector{TreeNode}
    knnTree::KDTree

    function Tree(root::TreeNode) # Tree must be initialized with a root node
        X = reshape(root.coords, :, 1)
        return new([root], KDTree(X))
    end
end

Tree(root_coords::Vector{T}) where {T<:Real} = Tree(TreeNode(nothing, root_coords))

function get_nodes(tree::Tree)
    return tree.nodes
end

function get_kd_tree(tree::Tree)
    return tree.knnTree
end

function get_root(tree::Tree)
    return tree.nodes[1]
end

function add_node!(tree::Tree, parent::TreeNode, coords::Vector{T}) where {T<:Real}
    new_node = TreeNode(parent, coords)
    push!(tree.nodes, new_node)
    add_point!(tree.knnTree, coords)
    add_child!(parent, new_node)
    return new_node
end
