#ifndef GUARD_DEMAND_H
#define GUARD_DEMAND_H

// cross inclusion if including demand.h in network.h?
#include <network.h>

#include <cstddef>
#include <string>
#include <vector>

class Agent {
public:
    Agent() = delete;

    Agent(size_t id_, unsigned at_id_, unsigned dp_id_,
          std::string oz_id_, std::string dz_id_, const Column* c = nullptr)
        : id {id_}, at_id {at_id_}, dp_id {dp_id_}, oz_id {oz_id_}, dz_id {dz_id_},
          col {c}, pce {1}
    {
    }

    Agent(const Agent&) = delete;
    Agent& operator=(const Agent&) = delete;

    Agent(Agent&&) = default;
    Agent& operator=(Agent&&) = default;

    ~Agent() = default;

    unsigned get_agent_type_id() const
    {
        return at_id;
    }

    unsigned get_demand_period_id() const
    {
        return dp_id;
    }

    const std::string& get_dest_zone_id() const
    {
        return dz_id;
    }

    const std::string& get_orig_zone_id() const
    {
        return oz_id;
    }

    size_t get_id() const
    {
        return id;
    }

    const std::vector<size_t>& get_link_path() const
    {
        return col->get_links();
    }

    const std::vector<size_t>& get_node_path() const
    {
        return col->get_nodes();
    }

    double get_pce() const
    {
        return pce;
    }

    // simulation
    bool at_last_link() const
    {
        return curr_link_no == 0;
    }

    size_t get_arr_interval() const
    {
        return arr_intvls[curr_link_no];
    }

    size_t get_dep_interval() const
    {
        return dep_intvls[curr_link_no];
    }

    size_t get_next_link_no() const
    {
        return get_link_path()[curr_link_no - 1];
    }

    double get_orig_dep_time() const
    {
        return dep_time;
    }

    size_t get_orig_dep_interval() const
    {
        return dep_intvls.back();
    }

    void move_to_next_link()
    {
        if (curr_link_no > 0)
            --curr_link_no;
    }

    void set_arr_interval(unsigned i, size_t increment = 0)
    {
        arr_intvls[curr_link_no - increment] = i;
    }

    void set_dep_interval(unsigned i)
    {
        dep_intvls[curr_link_no] = i;
    }

private:
    size_t id;
    unsigned at_id;
    unsigned dp_id;

    // use unsigned instead?
    std::string oz_id;
    std::string dz_id;

    const Column* col;

    double pce;

    // simulation
    size_t curr_link_no;
    double dep_time;

    std::vector<size_t> arr_intvls;
    std::vector<size_t> dep_intvls;
};

class AgentType {
public:
    AgentType() : id {0}, flow_type {0}, ffs {60}, pce {1},
                  vot {10}, name {"auto"}, is_link_ffs {true}
    {
    }

    AgentType(unsigned id_, unsigned flow_type_, double ffs_, double pce_,
              double vot_, std::string&& name_, bool use_link_ffs_)
        : id {id_}, flow_type {flow_type_}, ffs {ffs_}, pce {pce_},
          vot {vot_}, name {name_}, is_link_ffs {use_link_ffs_}
    {
    }

    AgentType(const AgentType&) = delete;
    AgentType& operator=(const AgentType&) = delete;

    AgentType(AgentType&&) = default;
    AgentType& operator=(AgentType&&) = default;

    ~AgentType() = default;

    unsigned get_id() const
    {
        return id;
    }

    unsigned get_flow_type() const
    {
        return flow_type;
    }

    double get_ffs() const
    {
        return ffs;
    }

    const std::string& get_name() const
    {
        return name;
    }

    double get_pce() const
    {
        return pce;
    }

    bool use_link_ffs() const
    {
        return is_link_ffs;
    }

public:
    static const std::string& get_default_name()
    {
        return default_name;
    }

    static const std::string& get_legacy_name()
    {
        return legacy_name;
    }

    static const std::string default_name;
    static const std::string legacy_name;

private:
    unsigned id;
    unsigned flow_type;

    double ffs;
    double pce;
    double vot;

    std::string name;
    bool is_link_ffs;
};

class SpecialEvent {

};

// can be combined with DemandPeriod?
class Demand {

};

class DemandPeriod {

};

#endif