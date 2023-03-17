/**
 * @file global.h, part of the project TransOMS under Apache License 2.0
 * @author jdlph (jdlph@hotmail.com) and xzhou99 (xzhou74@asu.edu)
 * @brief Alias, constants, and forward declarations of some classes.
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 */

#ifndef GUARD_GLOBAL_H
#define GUARD_GLOBAL_H

#define HEAP_DIJKSTRA

#include <string>
#include <tuple>

namespace transoms
{
using size_type = unsigned long;
// origin zone no, destination zone no, demand period no, agent type no
using ColumnVecKey = std::tuple<size_type, size_type, unsigned short, unsigned short>;

// some constants
constexpr unsigned short COORD_X = 91;
constexpr unsigned short COORD_Y = 181;
constexpr unsigned short MINUTES_IN_HOUR = 60;

const std::string ALL_MODES {"all"};
const std::string AT_DEFAULT_NAME {"passenger"};
const std::string AT_LEGACY_NAME {"auto"};

// forward declarations of supply classes to be used in demand.h
class Column;
class SpecialEvent;

// forward declarations of demand classes to be used in supply.h
class Agent;
class AgentType;
class DemandPeriod;

struct HeapNode {
    HeapNode(size_type no, double cost) : node_no {no}, cost {cost}
    {
    }

    size_type node_no;
    double cost;
};

struct HeapNodeLess {
    bool operator()(const HeapNode& h1, const HeapNode& h2)
    {
        return h1.cost > h2.cost;
    }
};

} // namespace transoms

#endif