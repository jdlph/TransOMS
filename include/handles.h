/**
 * @file handles.h, part of the project openDTA under GPL-3.0 license
 * @author jdlph (jdlph@hotmail.com) and xzhou99 (xzhou74@asu.edu)
 * @brief Definitions of handle classes with user API
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 */

#ifndef GUARD_HANDLES_H
#define GUARD_HANDLES_H

#include <demand.h>
#include <supply.h>

namespace opendta
{
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

        for (auto spn : spns)
            delete spn;
    }

    void find_ue(unsigned short column_gen_num, unsigned short column_opt_num);

    const Link* get_link(size_type link_no) const
    {
        return net.get_links()[link_no];
    }

    Link* get_link(size_type link_no)
    {
        return net.get_links()[link_no];
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

    void read_demands(const std::string& dir);
    void read_network(const std::string& dir);
    void read_settings(const std::string& dir);

    void output_columns(const std::string& dir = ".", const std::string& filename = "agents.csv");
    void output_columns_par(const std::string& dir = ".", const std::string& filename = "agents.csv");
    void output_link_performance(const std::string& = ".", const std::string& filename = "link_performance.csv");

private:
    void read_demand(const std::string& dir, unsigned short dp_no, unsigned short at_no);
    void auto_setup();

    void read_links(const std::string& dir, const std::string& filename = "link.csv");
    void read_nodes(const std::string& dir, const std::string& filename = "node.csv");

    void update_column_attributes();
    void update_column_gradient_and_flow(unsigned short iter_no);
    void update_link_and_column_volume(unsigned short iter_no, bool reduce_path_vol = true);
    // a little bit ugly
    void update_link_travel_time(const std::vector<DemandPeriod>* dps = nullptr, short iter_no = -1);

    void build_connectors();
    void setup_spnetworks();
    void delete_spnetworks();

    std::string get_link_path_str(const Column& c);
    std::string get_node_path_str(const Column& c);
    std::string get_node_path_coordinates(const Column& c);

private:
    ColumnPool cp;

    PhyNetwork net;
    std::vector<SPNetwork*> spns;

    std::vector<const AgentType*> ats;
    std::vector<DemandPeriod> dps;
};

} // namespace opendta

#endif