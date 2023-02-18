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
          col {c}, pce_factor {1}
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

    // simulation
    bool at_last_link() const
    {
        return curr_link_no == 0;
    }

    size_t get_arr_time() const
    {
        return arr_intvls[curr_link_no];
    }

    size_t get_curr_dep_interval() const
    {
        return dep_intvls[curr_link_no];
    }

    double get_dep_time() const
    {
        return dep_time;
    }

    size_t get_next_link_no() const
    {
        return get_link_path()[curr_link_no - 1];
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

    void set_arr_time(unsigned i, size_t increment = 0)
    {
        arr_intvls[curr_link_no - increment] = i;
    }

    void set_dep_time(unsigned i)
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

    double pce_factor;

    // simulation
    size_t curr_link_no;
    double dep_time;

    std::vector<size_t> arr_intvls;
    std::vector<size_t> dep_intvls;
};

class AgentType {

};

class SpecialEvent {

};

class Demand {

};

class DemandPeriod {

};

#endif