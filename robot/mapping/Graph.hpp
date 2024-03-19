#pragma once

#include <math.h>

#include "Tuple.hpp"  // Must appear before LinkedList.hpp.

//
#include "LinkedList.hpp"
#include "utilities.hpp"

namespace mtrn3100 {

template <typename N, typename E>
class Graph {
public:
    template <typename OuterIteratorType, typename InnerIteratorType>
    class iterator;
    struct directed_edge_comparator;
    struct edge_comparator;
    using value_type = Tuple<N, N, E>;
    using pointer_type = value_type*;
    using node_type = N;
    using edge_type = Tuple<N*, E>;
    using edges_type = LinkedList<edge_type, edge_comparator>;
    using adjacent_edges_type = Tuple<node_type, edges_type>;
    using graph_type = LinkedList<adjacent_edges_type, directed_edge_comparator>;
    using iterator_type = iterator<typename graph_type::iterator_type, typename edges_type::iterator_type>;
    using const_iterator_type =
        iterator<typename graph_type::const_iterator_type, typename edges_type::const_iterator_type>;

    // Custom comparator for graph_type.
    struct directed_edge_comparator {
        using is_transparent = void;
        constexpr bool operator()(adjacent_edges_type const& lhs, adjacent_edges_type const& rhs) const {
            return get<0>(lhs) < get<0>(rhs);
        }
    };

    // Custom comparator for edges_type.
    struct edge_comparator {
        using is_transparent = void;
        constexpr bool operator()(edge_type const& lhs, edge_type const& rhs) const {
            // Compares nodes then compares edges if nodes are the same.
            return *get<0>(lhs) != *get<0>(rhs) ? *get<0>(lhs) < *get<0>(rhs) : get<1>(lhs) < get<1>(rhs);
        }
    };

    template <typename OuterIteratorType, typename InnerIteratorType>
    class iterator {
    public:
        friend class Graph;
        using value_type = Graph::value_type;
        using pointer_type = Graph::pointer_type;

        iterator() = default;

        value_type operator*() { return {get<0>(outer_->value), *get<0>(inner_->value), get<1>(inner_->value)}; }

        value_type operator*() const { return {get<0>(outer_->value), *get<0>(inner_->value), get<1>(inner_->value)}; }

        // !!! Cannot use without RAII pointers because returning address of temporary object.
        // pointer_type operator->() { return &operator*(); }
        // pointer_type operator->() const { return &operator*(); }

        iterator& operator++() {
            inner_++;
            if (inner_ == get<1>(outer_->value).end()) {
                skip_forward_empty_inner();
            }
            return *this;
        }

        iterator operator++(int) {
            auto temp = *this;
            ++(*this);
            return temp;
        }

        iterator& operator--() {
            if (outer_ != end_ && inner_ != get<1>(outer_->value).begin()) {
                --inner_;
            } else {
                --outer_;
                skip_backward_empty_inner();
            }
            return *this;
        }

        iterator operator--(int) {
            auto temp = *this;
            --(*this);
            return temp;
        }

        bool operator==(iterator const& other) const {
            return outer_ == other.outer_ && (outer_ == end_ || inner_ == other.inner_);
        }

        bool operator!=(iterator const& other) const { return !operator==(other); }

    private:
        InnerIteratorType skip_forward_empty_inner() {
            while (outer_ != end_ && inner_ == get<1>(outer_->value).end()) {
                ++outer_;
                if (outer_ != end_) {
                    inner_ = get<1>(outer_->value).begin();
                }
            }
            return inner_;
        }

        InnerIteratorType skip_backward_empty_inner() {
            while (outer_ != begin_ && get<1>(outer_->value).begin() == get<1>(outer_->value).end()) {
                --outer_;
            }
            if (get<1>(outer_->value).begin() != get<1>(outer_->value).end()) {
                inner_ = get<1>(outer_->value).end();
                --inner_;
            }
            return inner_;
        }

        iterator(OuterIteratorType first, OuterIteratorType last) : iterator(first, last, first) {}

        iterator(OuterIteratorType first, OuterIteratorType last, OuterIteratorType outer)
            : iterator(first, last, outer, first == last ? InnerIteratorType{} : get<1>(first->value).begin()) {}

        iterator(OuterIteratorType first, OuterIteratorType last, OuterIteratorType outer, InnerIteratorType inner)
            : begin_(first), end_(last), outer_(outer), inner_(inner) {
            skip_forward_empty_inner();
        }

        OuterIteratorType begin_;
        OuterIteratorType end_;
        OuterIteratorType outer_;
        InnerIteratorType inner_;
    };

    Graph() = default;

    // Creates a graph with multiple nodes.
    // Example: Graph g(0, 1, 2);
    template <typename... Args>
    Graph(Args... values) {
        insert_node(values...);
    }

    // Inserts a single node.
    bool insert_node(N const& value) {
        if (is_node(value) == false) {
            internal_.insert({value, {}});
            return true;
        }
        return false;
    }

    // Inserts multiple nodes.
    // Example: g.insert_node(0, 1, 2);
    template <typename... Args>
    bool insert_node(N const& value, Args... values) {
        return insert_node(value) && insert_node(values...);
    }

    bool erase_node(N const& value) {
        if (is_node(value) == false) {
            return false;
        }

        // Check all nodes and remove value if there exists an edge.
        for (auto node_iter = internal_.begin(); node_iter != internal_.end(); node_iter++) {
            auto edges = get<1>(node_iter->value);
            for (auto edges_iter = edges.begin(); edges_iter != edges.end(); edges_iter++) {
                if (*get<0>(edges_iter->value) == value) {
                    edges.erase(edges_iter);
                }
            }
        }

        // Remove key last.
        internal_.erase(find_node(value));
        return true;
    }

    bool insert_edge(N const& src, N const& dst, E const& weight) {
         if (is_node(src) == false || is_node(dst) == false) {
             while (1) { 
             };
         }

        // Check if the edge doesn't already exist.
        if (find(src, dst, weight) == end()) {
            auto src_iter = find_node(src);
            auto dst_iter = find_node(dst);
            get<1>(src_iter->value).insert({&get<0>(dst_iter->value), weight});
            return true;
        }
        return false;
    }

    bool erase_edge(N const& src, N const& dst, E const& weight) {
        if (is_node(src) == false || is_node(dst) == false) {
            while (1) {
            };
        }

        auto edge_iter = find(src, dst, weight);
        if (edge_iter == end()) {
            return false;
        }

        get<1>(edge_iter.outer_->value).erase(edge_iter.inner_);
        return true;
    }

    iterator_type erase_edge(iterator_type i) {
        return i.outer_ == i.end_ ? end()
                                  : iterator_type(i.begin_, i.end_, i.outer_, get<1>(i.outer_->value).erase(i.inner_));
    }

    // Erases edges between i and s.
    iterator_type erase_edge(iterator_type i, iterator_type s) {
        while (i != s && i != end()) {
            i = erase_edge(i);
        }
        return i;
    }

    // Returns all nodes.
    LinkedList<N> nodes() const {
        LinkedList<N> ret;
        for (auto it = internal_.begin(); it != internal_.end(); it++) {
            ret.push_back(get<0>(it->value));
        }
        return ret;
    }

    // Returns all nodes connected to src.
    LinkedList<N> nodes(N const& src) const {
        if (is_node(src) == false) {
            while (1) {
            };
        }

        auto const& edges = get<1>(find_node(src)->value);
        LinkedList<N> ret;
        for (auto it = edges.begin(); it != edges.end(); it++) {
            ret.push_back(*get<0>(it->value));
        }
        return ret;
    }

    // Returns all edges.
    LinkedList<Tuple<N, N, E>> edges() const {
        LinkedList<Tuple<N, N, E>> ret;
        for (auto it = internal_.begin(); it != internal_.end(); it++) {
            auto const& edges = get<1>(it->value);
            for (auto edge = edges.begin(); edge != edges.end(); edge++) {
                ret.push_back({get<0>(it->value), *get<0>(edge->value), get<1>(edge->value)});
            }
        }
        return ret;
    }

    // Returns all nodes and weights connected to src.
    LinkedList<Tuple<N, E>> edges(N const& src) const {
        if (is_node(src) == false) {
            while (1) {
            };
        }

        auto const& edges = get<1>(find_node(src)->value);
        LinkedList<Tuple<N, E>> ret;
        for (auto it = edges.begin(); it != edges.end(); it++) {
            ret.push_back({*get<0>(it->value), get<1>(it->value)});
        }
        return ret;
    }

    // Returns all weights connected between src and dst.
    LinkedList<E> weights(N const& src, N const& dst) const {
        if (is_node(src) == false || is_node(dst) == false) {
            while (1) {
            };
        }

        auto const& edges = get<1>(find_node(src)->value);
        LinkedList<E> ret;
        for (auto it = edges.begin(); it != edges.end(); it++) {
            if (*get<0>(it->value) == dst) {
                ret.push_back(get<1>(it->value));
            }
        }
        return ret;
    }

    // Returns the number of nodes in the graph.
    size_t size() const { return internal_.size(); }

    void clear() { internal_.clear(); }

    bool is_node(N const& value) const {
        for (auto it = internal_.begin(); it != internal_.end(); it++) {
            if (get<0>(it->value) == value) {
                return true;
            }
        }
        return false;
    }

    bool empty() const { return size() == 0; }

    bool is_connected(N const& src, N const& dst) const {
        auto const& node_iter = find_node(src);
        auto const& edges = get<1>(node_iter->value);
        for (auto it = edges.begin(); it != edges.end(); it++) {
            if (*get<0>(it->value) == dst) {
                return true;
            }
        }
        return false;
    }

    iterator_type find(N const& src, N const& dst, E const& weight) {
        auto const& src_iter = find_node(src);
        if (src_iter == internal_.end()) {
            return end();
        }

        auto const& edges = get<1>(src_iter->value);
        auto dst_iter = find_node(dst);
        auto edge_iter = edges.find({&get<0>(dst_iter->value), weight});
        if (edge_iter == edges.end()) {
            return end();
        }

        return {internal_.begin(), internal_.end(), src_iter, edge_iter};
    }

    iterator_type begin() { return {internal_.begin(), internal_.end()}; }

    iterator_type begin() const { return {internal_.begin(), internal_.end()}; }

    const_iterator_type cbegin() const { return {internal_.cbegin(), internal_.cend()}; }

    iterator_type end() { return {internal_.begin(), internal_.end(), internal_.end()}; }

    iterator_type end() const { return {internal_.begin(), internal_.end(), internal_.end()}; }

    const_iterator_type cend() const { return {internal_.cbegin(), internal_.cend(), internal_.cend()}; }

    friend bool operator==(Graph const& lhs, Graph const& rhs) {
        if (lhs.nodes() != rhs.nodes()) {
            return false;
        }

        if ((lhs.begin() == lhs.end()) ^ (lhs.begin() == rhs.end())) {
            return false;
        }

        return lhs.internal_ == rhs.internal_;
    }

    friend bool operator!=(Graph const& lhs, Graph const& rhs) { return !operator==(lhs, rhs); }

private:
    typename graph_type::iterator_type find_node(N const& node) {
        auto it = internal_.begin();
        for (; it != internal_.end(); it++) {
            if (get<0>(it->value) == node) {
                break;
            }
        }
        return it;
    }

    typename graph_type::iterator_type find_node(N const& node) const {
        auto it = internal_.begin();
        for (; it != internal_.end(); it++) {
            if (get<0>(it->value) == node) {
                break;
            }
        }
        return it;
    }

    graph_type internal_;
};

}  // namespace mtrn3100
