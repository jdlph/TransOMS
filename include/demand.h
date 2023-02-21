#ifndef GUARD_DEMAND_H
#define GUARD_DEMAND_H

#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace opendta
{
// forward declarations
class Column;
class SpecialEvent;

class Agent {
public:
    Agent() = delete;

    Agent(size_t id_, unsigned at_id_, unsigned dp_id_,
          std::string oz_id_, std::string dz_id_, const Column* c = nullptr)
        : id {id_}, at_id {at_id_}, dp_id {dp_id_}, oz_id {oz_id_}, dz_id {dz_id_},
          col {c}, pce {1}
    {
        initialize_intervals();
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

    const std::vector<size_t>& get_link_path() const;
    const std::vector<size_t>& get_node_path() const;

    double get_pce() const
    {
        return pce;
    }

    // simulation
    bool at_last_link() const
    {
        return curr_link_no == 0;
    }

    unsigned get_arr_interval() const
    {
        return arr_intvls[curr_link_no];
    }

    unsigned get_dep_interval() const
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

    // can be combined with set_arr_interval()?
    void set_orig_arr_interval(unsigned i)
    {
        arr_intvls.back() = i;
    }

private:
    //  move it outside as col is an incomplete pointer?
    void initialize_intervals()
    {
        // throw an error or terminate?
        if (!col)
            return;

        std::vector<size_t>::size_type n = col->get_node_num();

        arr_intvls.resize(n);
        dep_intvls.resize(n);
        curr_link_no = n - 1;
    }

private:
    size_t id;
    // to do: use unsigned short
    unsigned at_id;
    unsigned dp_id;

    // use unsigned instead?
    std::string oz_id;
    std::string dz_id;

    const Column* col;

    // to do: float?
    double pce;

    // simulation
    size_t curr_link_no;
    double dep_time;

    std::vector<unsigned> arr_intvls;
    std::vector<unsigned> dep_intvls;
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

    double get_vot() const
    {
        return vot;
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

class DemandPeriod {
public:
    DemandPeriod() : id {0}, at_name {"auto"}, filename {"demand.csv"},
                     period {"AM"}, time_period {"0700_0800"}, se {nullptr}
    {
    }

    DemandPeriod(unsigned id_, std::string&& at_name_, std::string&& filename_,
                 std::string&& period_, std::string&& time_period_, const SpecialEvent* se_)
        : id {id_}, at_name {at_name_}, filename {filename_},
          period {period_}, time_period {time_period_}, se {se_}
    {
    }

    DemandPeriod(const DemandPeriod&) = delete;
    DemandPeriod& operator=(const DemandPeriod&) = delete;

    DemandPeriod(DemandPeriod&&) = delete;
    DemandPeriod& operator=(DemandPeriod&&) = delete;

    ~DemandPeriod()
    {
        delete se;
    }

    const std::string& get_agent_type_name() const
    {
        return at_name;
    }

    double get_agent_vot() const
    {
        at->get_vot();
    }

    const std::string& get_demand_file_name() const
    {
        return filename;
    }

    unsigned get_id() const
    {
        return id;
    }

    const std::string& get_period() const
    {
        return period;
    }

    const std::string& get_time_period() const
    {
        return time_period;
    }

    bool contain_iter_no(unsigned iter_no) const;
    double get_cap_reduction_ratio(size_t link_no, unsigned iter_no) const;

private:
    unsigned id;

    // to do: no need any more after at is added as member?
    std::string at_name;
    std::string filename;
    std::string period;
    std::string time_period;

    const AgentType* at;
    // change back to SpecialEvent later
    const SpecialEvent* se;
};

} // namespace opendta

#endif