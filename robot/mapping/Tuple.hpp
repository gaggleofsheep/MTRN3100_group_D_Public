#pragma once

#include <stddef.h>

namespace mtrn3100 {

namespace _tuple {

template <size_t i, typename T>
struct Node {
    Node() = default;
    Node(T t) : value(t) {}
    T value = {};
};

// Partial template specialisation which enables recursion.
template <size_t i, typename... Args>
struct Tuple;

// Partial template specialisation which ends recursion when there are no more Args to parse.
template <size_t i>
struct Tuple<i> {};

// Creates a tuple.
// Example: Tuple<int, char, float> tup(1, '3', 2.1);
template <size_t i, typename Arg, typename... Args>
struct Tuple<i, Arg, Args...> : public Node<i, Arg>, public Tuple<i + 1, Args...> {
    Tuple() = default;
    Tuple(Arg const& arg, Args const&... args) : Node<i, Arg>(arg), Tuple<i + 1, Args...>(args...) {}
};

}  // namespace _tuple

// Accesses the tuple by index.
// Example: int first = mtrn3100::get<0>(tup);
template <size_t i, typename Arg, typename... Args>
Arg& get(_tuple::Tuple<i, Arg, Args...>& t) {
    using namespace _tuple;
    return t.Node<i, Arg>::value;
}

// Accesses the tuple by index.
// Example: int first = mtrn3100::get<0>(tup);
template <size_t i, typename Arg, typename... Args>
const Arg& get(_tuple::Tuple<i, Arg, Args...> const& t) {
    using namespace _tuple;
    return t.Node<i, Arg>::value;
}

template <typename... Args>
using Tuple = _tuple::Tuple<0, Args...>;

}  // namespace mtrn3100

template <size_t i, typename Arg>
bool operator==(mtrn3100::_tuple::Tuple<i, Arg> const& lhs, mtrn3100::_tuple::Tuple<i, Arg> const& rhs) {
    return mtrn3100::get<i>(lhs) == mtrn3100::get<i>(rhs);
}

template <size_t i, typename Arg1, typename Arg2, typename... Args>
bool operator==(mtrn3100::_tuple::Tuple<i, Arg1, Arg2, Args...> const& lhs,
                mtrn3100::_tuple::Tuple<i, Arg1, Arg2, Args...> const& rhs) {
    return mtrn3100::get<i>(lhs) == mtrn3100::get<i>(rhs) && operator==(
                                                                 mtrn3100::_tuple::Tuple<i + 1, Arg2, Args...>(lhs),
                                                                 mtrn3100::_tuple::Tuple<i + 1, Arg2, Args...>(rhs));
}

template <size_t i, typename Arg1, typename Arg2, typename... Args>
bool operator!=(mtrn3100::_tuple::Tuple<i, Arg1, Arg2, Args...> const& lhs,
                mtrn3100::_tuple::Tuple<i, Arg1, Arg2, Args...> const& rhs) {
    return !operator==(lhs, rhs);
}
