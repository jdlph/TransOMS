/**
 * @file minheap.h, part of the project TransOMS under Apache License 2.0
 * @author jdlph (jdlph@hotmail.com) and xzhou99 (xzhou74@asu.edu)
 * @brief a special min-heap that guarantees logarithmic time pop()
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 */

#ifndef GUARD_MIN_HEAP_H
#define GUARD_MIN_HEAP_H

#include <global.h>

#include <cmath>
#include <utility>
#include <vector>

namespace transoms
{
class MinHeap {
public:
    MinHeap() = delete;

    explicit MinHeap(unsigned short d_) : d {d_}, num {0}
    {
    }

    MinHeap(const MinHeap&) = delete;
    MinHeap& operator=(const MinHeap&) = delete;

    MinHeap(MinHeap&&) = delete;
    MinHeap& operator=(MinHeap&&) = delete;

    ~MinHeap()
    {
        delete[] keys;
        delete[] nodes;
        delete[] posns;
    }

    bool empty() const
    {
        return num == 0;
    }

    void emplace(size_type i, double v)
    {
        keys[i] = v;
        if (nodes[posns[i]] != i)
        {
            posns[i] = num;
            nodes[num++] = i;
        }

        shiftup(i);
    }

    void pop()
    {
        auto j = nodes[--num];
        swap(nodes[0], j);
        shiftdown(j);
    }

    void reserve(size_type sz_)
    {
        sz = sz_;
        keys = new double[sz_];
        posns = new size_type[sz_];
        nodes = new size_type[sz_];
    }

    void reset()
    {
        num = 0;
        for (auto i = 0; i != sz; ++i)
        {
            nodes[i] = 0;
            posns[i] = 0;
        }
    }

    auto top() const
    {
        return std::make_pair(keys[nodes[0]], nodes[0]);
    }

private:

    bool is_leaf(size_type i) const
    {
        return (posns[i] - 1) * d + 3 >= num;
    }

    size_type pred(size_type i) const
    {
        if (!posns[i])
            return i;

        size_type pos = std::ceil((posns[i]) / d);
        return nodes[pos];
    }

    auto succ(size_type i) const
    {
        auto s = (posns[i] - 1) * d + 3;
        auto e = std::min(num - 1, s + d - 1);

        return std::make_pair(s, e);
    }

    auto minchild(size_type i) const
    {
        auto r = succ(i);
        auto pos_ = r.first;        
        auto v = keys[nodes[pos_]];
    
        for (auto pos = ++r.first; pos <= r.second; ++pos)
        {
            if (keys[nodes[pos]] < v)
            {
                v = keys[nodes[pos]];
                pos_ = pos;
            }
        }

        return nodes[pos_];
    }

    void shiftdown(size_type i)
    {
        while (!is_leaf(i) && keys[i] > keys[minchild(i)])
            swap(i, minchild(i));
    }

    void shiftup(size_type i)
    {
        while (posns[i] && keys[i] < keys[pred(i)])
            swap(i, pred(i));
    }

    // bottleneck??
    void swap(size_type i, size_type j)
    {
        auto temp = posns[i];
        posns[i] = posns[j];
        posns[j] = temp;

        temp = nodes[posns[i]];
        nodes[posns[i]] = nodes[posns[j]];
        nodes[posns[j]] = temp;
    }

private:
    // node_no to key
    double* keys;
    // pos to node_no
    size_type* nodes;
    // node_no to pos
    size_type* posns;

    // node_no, key
    std::vector<std::pair<size_type, double>> dheap;

    unsigned short d;
    size_type num;
    size_type sz;
};

} // namespace transoms

#endif