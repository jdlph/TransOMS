#ifndef GUARD_DEMAND_H
#define GUARD_DEMAND_H

#include <supply.h>

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

    Agent(size_type no_, unsigned short at_no_, unsigned short dp_no_,
          size_type oz_no_, size_type dz_no_, const Column* c = nullptr)
        : no {no_}, at_no {at_no_}, dp_no {dp_no_}, oz_no {oz_no_}, dz_no {dz_no_},
          col {c}, pce {1}
    {
        initialize_intervals();
    }

    Agent(const Agent&) = delete;
    Agent& operator=(const Agent&) = delete;

    Agent(Agent&&) = default;
    Agent& operator=(Agent&&) = default;

    ~Agent() = default;

    auto get_agent_type_no() const
    {
        return at_no;
    }

    auto get_demand_period_no() const
    {
        return dp_no;
    }

    auto get_dest_zone_no() const
    {
        return dz_no;
    }

    auto get_orig_zone_no() const
    {
        return oz_no;
    }

    size_type get_no() const
    {
        return no;
    }

    const std::vector<size_type>& get_link_path() const;
    const std::vector<size_type>& get_node_path() const;

    double get_pce() const
    {
        return pce;
    }

    // simulation
    bool at_last_link() const
    {
        return curr_link_no == 0;
    }

    auto get_arr_interval() const
    {
        return arr_intvls[curr_link_no];
    }

    auto get_dep_interval() const
    {
        return dep_intvls[curr_link_no];
    }

    size_type get_next_link_no() const
    {
        return get_link_path()[curr_link_no - 1];
    }

    double get_orig_dep_time() const
    {
        return dep_time;
    }

    size_type get_orig_dep_interval() const
    {
        return dep_intvls.back();
    }

    void move_to_next_link()
    {
        if (curr_link_no > 0)
            --curr_link_no;
    }

    void set_arr_interval(unsigned i, size_type increment = 0)
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

        auto n = col->get_node_num();

        arr_intvls.resize(n);
        dep_intvls.resize(n);
        curr_link_no = n - 1;
    }

private:
    size_type no;

    unsigned short at_no;
    unsigned short dp_no;

    // use unsigned short instead?
    size_type oz_no;
    size_type dz_no;

    const Column* col;

    // to do: float?
    double pce;

    // simulation
    size_type curr_link_no;
    double dep_time;

    std::vector<unsigned> arr_intvls;
    std::vector<unsigned> dep_intvls;
};

class AgentType {
public:
    AgentType() : no {0}, flow_type {0}, ffs {60}, pce {1},
                  vot {10}, name {"auto"}, is_link_ffs {true}
    {
    }

    AgentType(unsigned short no_, unsigned short flow_type_, double ffs_, double pce_,
              double vot_, std::string&& name_, bool use_link_ffs_)
        : no {no_}, flow_type {flow_type_}, ffs {ffs_}, pce {pce_},
          vot {vot_}, name {name_}, is_link_ffs {use_link_ffs_}
    {
    }

    AgentType(const AgentType&) = delete;
    AgentType& operator=(const AgentType&) = delete;

    AgentType(AgentType&&) = default;
    AgentType& operator=(AgentType&&) = default;

    ~AgentType() = default;

    auto get_no() const
    {
        return no;
    }

    auto get_flow_type() const
    {
        return flow_type;
    }

    auto get_ffs() const
    {
        return ffs;
    }

    const std::string& get_name() const
    {
        return name;
    }

    auto get_pce() const
    {
        return pce;
    }

    auto get_vot() const
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
    unsigned short no;
    unsigned short flow_type;

    double ffs;
    double pce;
    double vot;

    std::string name;
    bool is_link_ffs;
};

class Demand {
public:
    Demand() = delete;

    explicit Demand(const AgentType* at_) : at {at_}
    {
    }

    Demand(unsigned short no_, std::string&& filename_, const AgentType* at_)
        : no {no_}, filename {filename_}, at {at_}
    {
    }

    Demand(const Demand&) = default;
    Demand& operator=(const Demand&) = delete;

    Demand(Demand&&) = default;
    Demand& operator=(Demand&&) = default;

    ~Demand() = default;

    auto get_no() const
    {
        return no;
    }

    const std::string& get_agent_type_name() const
    {
        return at->get_name();
    }

    auto get_agent_type_no() const
    {
        return at->get_no();
    }

    const std::string& get_file_name() const
    {
        return filename;
    }

private:
    unsigned short no = 0;
    std::string filename = "demand.csv";

    const AgentType* at;
};

class DemandPeriod {
public:
    DemandPeriod() : no {0}, period {"AM"}, time_period {"0700_0800"}, se {nullptr}
    {
    }

    explicit DemandPeriod(Demand&& dp)
    {
        ds.push_back(dp);
    }

    DemandPeriod(unsigned short no_, std::string&& at_name_,
                 std::string&& period_, std::string&& time_period_, const SpecialEvent* se_)
        : no {no_}, period {period_}, time_period {time_period_}, se {se_}
    {
    }

    DemandPeriod(const DemandPeriod&) = default;
    DemandPeriod& operator=(const DemandPeriod&) = delete;

    DemandPeriod(DemandPeriod&&) = default;
    DemandPeriod& operator=(DemandPeriod&&) = delete;

    ~DemandPeriod()
    {
        delete se;
    }

    void add_agent_type(const AgentType* at)
    {
        ats.push_back(at);
    }

    void attached_special_event(const SpecialEvent* s)
    {
        se = s;
    }

    auto get_no() const
    {
        return no;
    }

    const std::string& get_period() const
    {
        return period;
    }

    const std::string& get_time_period() const
    {
        return time_period;
    }

    const auto& get_demands() const
    {
        return ds;
    }

    bool contain_iter_no(unsigned short iter_no) const;
    double get_cap_reduction_ratio(size_type link_no, unsigned short iter_no) const;

private:
    unsigned short no;

    std::string period;
    std::string time_period;

    std::vector<const AgentType*> ats;
    std::vector<Demand> ds;
    // change back to SpecialEvent later
    const SpecialEvent* se;
};

} // namespace opendta

#endif