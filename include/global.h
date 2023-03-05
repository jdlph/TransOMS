#ifndef GUARD_GLOBAL_H
#define GUARD_GLOBAL_H

#include <string>
#include <tuple>

namespace opendta
{
using size_type = unsigned long;

// origin zone no, destination zone no, demand period no, agent type no
using ColumnVecKey = std::tuple<size_type, size_type, unsigned short, unsigned short>;
// distance, node_no. reserved for heap Dijkstra's algorithm
using HeapElement = std::pair<double, size_type>;

// some constants
constexpr unsigned short COORD_X = 91;
constexpr unsigned short COORD_Y = 181;
constexpr unsigned short MINUTES_IN_HOUR = 60;

constexpr long NULL_NODE = -1;
constexpr long PAST_NODE = -3;

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

} // namespace opendta

#endif