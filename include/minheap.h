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
        : keys {new double[n]}, posns {new long[n]}, d {d_}
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
        return std::make_pair(keys[min_node], min_node);
    }

    void emplace(size_type i, double v)
    {
        keys[i] = v;
        posns[i] = num;
        nodes[num++] = i;
        // if (!num)
        //     posns[i] = num++;
        // else
        //     posns[i] = ++num;
        shiftup(i);
    }

    void pop()
    {
        // --num;
        shiftup(nodes[--num]);
    }

    void reset()
    {
        num = min_node = last = 0;
        // no need to reset keys and posns?
    }

    void reserve(size_type sz)
    {
        keys = new double[sz];
        posns = new long[sz];
        nodes = new long[sz];
    }

private:
    size_type pred(size_type i) const
    {
        if (posns[i] <= 0)
            return i;
        
        size_type pos = std::ceil((posns[i] - 1.0) / d);
        return nodes[pos];
    }

    void swap(size_type i, size_type j)
    {
        auto temp = posns[i];
        posns[i] = posns[j];
        posns[j] = temp;
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
    long* posns;
    // pos to node_no
    long* nodes;

    unsigned short d;
    size_type num = 0;
    size_type min_node = 0;
    size_type last = 0;
    size_type last_pos = 0;
};

} // namespace transoms

#endif