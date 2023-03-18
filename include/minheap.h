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

namespace transoms
{
class MinHeap {
public:
    MinHeap() = delete;

    explicit MinHeap(unsigned short d_) : d {d_}
    {
    }

    MinHeap(size_type n, unsigned short d_)
        : keys {new double[n]}, posns {new size_type[n]}, d {d_}
    {
    }

    MinHeap(const MinHeap&) = delete;
    MinHeap& operator=(const MinHeap&) = delete;

    MinHeap(MinHeap&&) = delete;
    MinHeap& operator=(MinHeap&&) = delete;

    ~MinHeap()
    {
        delete[] keys;
        delete[] posns;
    }

    bool empty() const
    {
        return num == 0;
    }

    auto top() const
    {
        return std::make_pair(keys[nodes[0]], nodes[0]);
    }

    void emplace(size_type i, double v)
    {
        if (i == 552)
            int debug = 1;
        
        keys[i] = v;
        if (!marked[i])
        {
            posns[i] = num;
            nodes[num++] = i;
            marked[i] = true;
        }

        // if (!num)
        //     posns[i] = num++;
        // else
        //     posns[i] = ++num;
        shiftup(i);
    }

    void pop()
    {
        auto j = nodes[--num];
        swap(nodes[0], j);
        // --num;
        shiftdown(j);
    }

    void reset()
    {
        num = min_node = last = 0;
        // no need to reset keys and posns?
        for (auto i = 0; i != sz; ++i)
        {
            posns[i] = 0;
            nodes[i] = 0;
            keys[i] = 0;
            marked[i] = false;
        }
    }

    void reserve(size_type sz_)
    {
        sz = sz_;
        keys = new double[sz_];
        posns = new size_type[sz_];
        nodes = new size_type[sz_];
        marked = new bool[sz_];

        for (auto i = 0; i != sz; ++i)
            marked[i] = false;
    }

private:
    size_type pred(size_type i) const
    {
        if (posns[i] <= 0)
            return i;

        size_type pos = std::ceil((posns[i]) / d);
        return nodes[pos];
    }

    auto succ(size_type i) const
    {
        auto s = (posns[i] - 1) * d + 3;
        auto e = std::min(num - 1, posns[i] * d + 2);

        return std::make_pair(s, e);
    }

    bool is_leaf(size_type i) const
    {
        return (posns[i] - 1) * d + 2 >= num;
    }

    auto minchild(size_type i) const
    {
        auto r = succ(i);

        auto pos = r.first;
        auto pos_ = r.first;
        auto v = keys[nodes[pos++]];
        for (; pos <= r.second; ++pos)
        {
            if (keys[nodes[pos]] < v)
            {
                v = keys[nodes[pos]];
                pos_ = pos;
            }
        }

        return nodes[pos_];
    }

    void swap(size_type i, size_type j)
    {
        auto temp = posns[i];
        posns[i] = posns[j];
        posns[j] = temp;

        temp = nodes[posns[i]];
        nodes[posns[i]] = nodes[posns[j]];
        nodes[posns[j]] = temp;
    }

    void shiftdown(size_type i)
    {
        while (!is_leaf(i) && keys[i] > keys[minchild(i)])
        {
            swap(i, minchild(i));
        }
    }

    void shiftup(size_type i)
    {
        while (posns[i] && keys[i] < keys[pred(i)])
        {
            swap(i, pred(i));
        }

        if (posns[i] == 0)
            min_node = i;
    }

private:
    double* keys;
    // node_no to pos
    size_type* posns;
    // pos to node_no
    size_type* nodes;
    bool* marked;

    unsigned short d;
    size_type num = 0;
    size_type min_node = 0;
    size_type last = 0;
    size_type last_pos = 0;
    size_type sz;
};

} // namespace transoms

#endif