/**
 * @file handles.h, part of the project TransOMS under Apache License 2.0
 * @author jdlph (jdlph@hotmail.com) and xzhou99 (xzhou74@asu.edu)
 * @brief Definitions of handle classes with user API
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 */

#ifndef GUARD_HANDLES_H
#define GUARD_HANDLES_H

#include <demand.h>
#include <supply.h>

namespace transoms
{
enum class TrafficFlowModel {
    point_queue, spatial_queue, kinematic_wave
};

class NetworkHandle {
public:
    NetworkHandle() = default;

    NetworkHandle(const NetworkHandle&) = delete;
    NetworkHandle& operator=(const NetworkHandle&) = delete;

    NetworkHandle(NetworkHandle&&) = delete;
    NetworkHandle& operator=(NetworkHandle&&) = delete;

    ~NetworkHandle()
    {
        for (auto at : ats)
            delete at;

        for (auto dp : dps)
            delete dp;

        for (auto spn : spns)
            delete spn;
    }

    void find_ue(unsigned short column_gen_num, unsigned short column_opt_num);
    void run_simulation();

    void read_demands(const std::string& dir);
    void read_network(const std::string& dir);
    void read_settings(const std::string& dir);

    void output_columns(const std::string& dir = ".", const std::string& filename = "agents.csv");
    void output_link_performance(const std::string& = ".", const std::string& filename = "link_performance.csv");
    void output_trajectories(const std::string& = ".", const std::string& filename = "trajectories.csv");

private:
    const Link* get_link(size_type link_no) const
    {
        return net.get_links()[link_no];
    }

    Link* get_link(size_type link_no)
    {
        return net.get_link(link_no);
    }

    const Link* get_link(const std::string& link_id) const
    {
        return net.get_link(link_id);
    }

    Link* get_link(const std::string& link_id)
    {
        return net.get_link(link_id);
    }

    const Node* get_node(size_type node_no) const
    {
        return net.get_nodes()[node_no];
    }

    Node* get_node(size_type node_no)
    {
        return net.get_nodes()[node_no];
    }

    const std::string& get_head_node_id(const Link* link) const
    {
        return get_node(link->get_head_node_no())->get_id();
    }

    const std::string& get_tail_node_id(const Link* link) const
    {
        return get_node(link->get_tail_node_no())->get_id();
    }

    const std::string& get_zone_id(size_type zone_no) const
    {
        return net.get_zone_id(zone_no);
    }

    const auto get_agent_type(const std::string& at_name) const
    {
        for (const auto at : ats)
        {
            if (at->get_name() == at_name)
                return at;
        }

        throw std::exception{};
    }

    bool contains_agent_name(const std::string& at_name) const
    {
        for (const auto at : ats)
        {
            if (at->get_name() == at_name)
                return true;
        }

        return false;
    }

private:
    void read_demand(const std::string& dir, unsigned short dp_no, unsigned short at_no);
    void read_settings_yml(const std::string& file_path);
    void auto_setup();

    void read_links(const std::string& dir, const std::string& filename = "link.csv");
    void read_nodes(const std::string& dir, const std::string& filename = "node.csv");

    void update_column_attributes();
    void update_column_gradient_and_flow(unsigned short iter_no);
    void update_link_and_column_volume(unsigned short iter_no, bool reduce_path_vol = true);
    void update_link_travel_time();

    void build_connectors();
    void setup_spnetworks();
    void delete_spnetworks();

    void setup_agents();
    void setup_link_queues();

    ColumnVec& get_column_vec(size_type i);
    std::string get_link_path_str(const Column& c);
    std::string get_node_path_str(const Column& c);
    std::string get_node_path_coordinates(const Column& c);

    unsigned short cast_interval_to_minute(size_type i) const;
    double cast_interval_to_minute_double(size_type i) const;
    size_type cast_minute_to_interval(unsigned short m) const;

    size_type get_beg_simulation_interval(unsigned short k) const;
    size_type get_end_simulation_interval(unsigned short k) const;

    double get_real_time(size_type i) const;

    // unsigned short get_simulation_resolution() const;
    size_type get_simulation_intervals() const;

    const std::vector<size_type>& get_agents_at_interval(size_type i) const;
    Agent& get_agent(size_type no);
    const Agent& get_agent(size_type no) const;

    LinkQueue& get_link_queue(size_type i);

    bool has_dep_agents(size_type i) const;
    bool uses_point_queue_model() const;
    bool uses_spatial_queue_model() const;
    bool uses_kinematic_wave_model() const;

    void update_simulation_settings(const std::string& model, unsigned short res);

    static std::string get_time_stamp(double t);

private:
    ColumnPool cp;

    PhyNetwork net;
    std::vector<SPNetwork*> spns;

    std::vector<const AgentType*> ats;
    std::vector<const DemandPeriod*> dps;

    unsigned short thread_nums = 4;

    // simulation
    // number of seconds per simulation interval
    unsigned short simu_res = 6;
    // simulation duration in minutes
    unsigned short simu_dur = 60;

    TrafficFlowModel tfm = TrafficFlowModel::point_queue;

    std::vector<LinkQueue> link_queues;

    // the following two can be combined
    std::vector<Agent> agents;
    // time-dependent agents for simulation
    std::map<size_type, std::vector<size_type>> td_agents;
};

} // namespace transoms

#endif