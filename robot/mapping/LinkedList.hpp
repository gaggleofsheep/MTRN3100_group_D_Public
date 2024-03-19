#pragma once

#include <stddef.h>

#include "utilities.hpp"

namespace mtrn3100 {

template <typename T, typename Compare = util::less<T>>
class LinkedList {
private:
    struct LinkedListNode {
        LinkedListNode(const T& value, LinkedListNode* prev, LinkedListNode* next)
            : prev(prev), next(next), value(value) {
            if (prev != nullptr) {
                prev->next = this;
            }
            if (next != nullptr) {
                next->prev = this;
            }
        }

        LinkedListNode(const LinkedListNode&) = default;

        LinkedListNode& operator=(const LinkedListNode&) = default;

        ~LinkedListNode() {
            if (prev != nullptr) {
                prev->next = next;
            }
            if (next != nullptr) {
                next->prev = prev;
            }
        }

        bool operator==(const LinkedListNode& other) const {
            return prev == other.prev && next == other.next && value == other.value;
        }

        bool operator!=(const LinkedListNode& other) { return !operator==(other); }

        LinkedListNode* prev = nullptr;
        LinkedListNode* next = nullptr;
        T value = {};
    };

public:
    template <typename IteratorType>
    class iterator;

    using value_type = LinkedListNode;
    using const_value_type = const LinkedListNode;
    using pointer_type = LinkedListNode*;
    using const_pointer_type = const LinkedListNode*;
    using iterator_type = iterator<pointer_type>;
    using const_iterator_type = iterator<const_pointer_type>;

    template <typename IteratorType>
    class iterator {
    public:
        friend class LinkedList;

        iterator() = default;

        iterator& operator++() {
            curr = curr->next;
            return *this;
        }

        iterator operator++(int) {
            auto temp = *this;
            ++(*this);
            return temp;
        }

        iterator& operator--() {
            if (curr == nullptr) {
                curr = sentinel;
                return *this;
            }
            curr = curr->prev;
            return *this;
        }

        iterator operator--(int) {
            auto temp = *this;
            --(*this);
            return temp;
        }

        value_type operator*() { return *curr; }

        const_value_type operator*() const { return *curr; }

        pointer_type operator->() { return curr; }

        const_pointer_type operator->() const { return curr; }

        bool operator==(const iterator& other) const { return curr == other.curr; }

        bool operator!=(const iterator& other) const { return !operator==(other); }

        operator IteratorType() { return curr; }

    private:
        iterator(IteratorType curr) : curr(curr) {}

        iterator(iterator first, iterator last) : curr(first), sentinel(last) {}

        IteratorType curr = nullptr;
        IteratorType sentinel = nullptr;  // Always one before end.
    };

    LinkedList() = default;

    // Creates a linked list with multiple nodes.
    // Example: LinkedList<int> ll0(0);
    //          LinkedList<int> ll1(0, 1, 2, 3);
    //
    // Author: JeJo
    // Link: https://stackoverflow.com/a/53281524/13133452
    // License: https://creativecommons.org/licenses/by-sa/4.0/
    // Changes:
    template <typename... Args>
    LinkedList(Args... elements) {
        using dummy = T[];
        (void)dummy{0, (push_back(elements), 0)...};  // What?
    }

    LinkedList(const LinkedList& other) { copy(other); }

    LinkedList& operator=(const LinkedList& other) {
        copy(other);
        return *this;
    }

    ~LinkedList() { clear(); }

    iterator_type begin() { return {head, tail}; }

    iterator_type begin() const { return {head, tail}; }

    const_iterator_type cbegin() const { return {head, tail}; }

    iterator_type end() { return {nullptr, tail}; }

    iterator_type end() const { return {nullptr, tail}; }

    const_iterator_type cend() const { return {nullptr, tail}; }

    size_t size() const { return size_; }

    bool empty() const { return size_ == 0; }

    // Gets the first element.
    const T& front() const { return head->value; }

    // Gets the last element.
    const T& back() const { return tail->value; }

    // Inserts elements in ascending order (by default) or by whatever comparator is set e.g.
    // LinkedList<int, comparator>.
    void insert(const T& element) {
        if (size() == 0) {
            insert(0, element);
            return;
        }

        iterator_type curr = head;
        for (size_t i = 0; i < size(); i++) {
            if (curr && compare(element, curr->value) > 0) {
                insert(i, element);
                return;
            }
            ++curr;
        }

        insert(size(), element);
    }

    // Inserts elements at the index. Every other insert-related method will use this method.
    void insert(size_t index, const T& element) {
        if (index < 0 || index > size_) {
            while (1) {
            };
        }

        // Insertion into empty list.
        if (empty()) {
            iterator_type node = new value_type(element, nullptr, nullptr);
            head = node;
            tail = node;
            size_++;
            return;
        }

        // Insertion at head of list.
        if (index == 0) {
            iterator_type node = new value_type(element, nullptr, head);
            head = node;
            size_++;
            return;
        }

        // Insertion in middle of list.
        iterator_type prev_node = get(index - 1);
        iterator_type next_node = prev_node->next;
        iterator_type node = new value_type(element, prev_node, next_node);
        if (index == size_) {
            tail = node;
        }
        size_++;
    }

    // Deletes element at the index and returns it.
    T remove(size_t index) {
        if (index < 0 || index >= size_) {
            while (1) {
            };
        }

        return remove(get(index));
    }

    // Removes the iterator and returns the element removed.
    T remove(iterator_type iter) {
        if (iter == end()) {
            while (1) {
            };
        }

        iterator_type curr_iter = iter;

        if (iter == begin()) {
            head = curr_iter->next;
        }

        if (iter == end()) {
            tail = curr_iter->prev;
        }

        T temp = curr_iter->value;
        delete curr_iter;
        curr_iter = nullptr;
        size_--;

        return temp;
    }

    // Removes the iterator and returns the next iterator.
    iterator_type erase(iterator_type iter) {
        if (iter == end()) {
            while (1) {
            };
        }

        iterator_type curr_iter = iter;

        if (iter == begin()) {
            head = curr_iter->next;
        }

        if (iter == end()) {
            tail = curr_iter->prev;
        }

        iterator_type next = curr_iter->next;
        delete curr_iter;
        curr_iter = nullptr;
        size_--;

        return next;
    }

    // Pushes elements to beginning of linked list.
    void push_front(const T& element) { insert(0, element); }

    // Removes first element and returns it.
    T pop_front() { return remove(0); }

    // Pushes elements to back of linked list.
    void push_back(const T& element) { insert(size_, element); }

    // Removes last element and returns it.
    T pop_back() { return remove(size_ - 1); }

    // Gets the iterator of the element at the index.
    iterator_type get(const size_t index) {
        if (index < 0 || index >= size_) {
            while (1) {
            };
        }
        iterator_type curr = head;
        for (size_t i = 0; i < index; i++) {
            ++curr;
        }
        return {curr, tail};
    }

    // Gets the iterator of the element at the index.
    iterator_type get(const size_t index) const {
        if (index < 0 || index >= size_) {
            while (1) {
            };
        }
        iterator_type curr = head;
        for (size_t i = 0; i < index; i++) {
            ++curr;
        }
        return {curr, tail};
    }

    // Finds an iterator for the element if it exists. Returns node-after-last otherwise.
    iterator_type find(const T& element) {
        iterator_type curr = head;
        for (size_t i = 0; i < size_; i++) {
            if (curr->value == element) {
                break;
            }
            ++curr;
        }
        return {curr, tail};
    }

    // Finds an iterator for the element if it exists. Returns node-after-last otherwise.
    iterator_type find(const T& element) const {
        iterator_type curr = head;
        for (size_t i = 0; i < size_; i++) {
            if (curr->value == element) {
                break;
            }
            ++curr;
        }
        return {curr, tail};
    }

    bool contains(const T& element) const { return find(element) != end(); }

    // Used to access the linked list.
    // Example: ll[0] will get the first element.
    T operator[](const size_t index) { return get(index)->value; }

    // Used to access the linked list.
    // Example: ll[0] will get the first element.
    const T operator[](const size_t index) const { return get(index)->value; }

    void clear() {
        while (!empty()) {
            remove(0);
        }
    }

    friend bool operator==(const LinkedList& lhs, const LinkedList& rhs) {
        if (lhs.size_ != rhs.size_) {
            return false;
        }

        for (size_t i = 0; i < lhs.size_; i++) {
            if (lhs[i] != rhs[i]) {
                return false;
            }
        }

        return true;
    }

    friend bool operator!=(const LinkedList& lhs, const LinkedList& rhs) { return !operator==(lhs, rhs); }

private:
    void copy(const LinkedList& other) {
        for (size_t i = 0; i < other.size_; i++) {
            push_back(other[i]);
        }
    }

    iterator_type head = nullptr;
    iterator_type tail = nullptr;
    size_t size_ = 0;
    Compare compare = Compare();
};

}  // namespace mtrn3100
