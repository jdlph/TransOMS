#include <demand.h>
#include <handles.h>
#include <supply.h>

using namespace opendta;

inline const std::vector<size_type>& Agent::get_link_path() const
{
    return col->get_links();
}

inline const std::vector<size_type>& Agent::get_node_path() const
{
    return col->get_nodes();
}

void Link::update_period_travel_time(const std::vector<DemandPeriod>* dps, short iter_no)
{
    for (auto i = 0; i != vdfps.size(); ++i)
    {
        // make sure vdfps has the same size as dps
        auto reduction_ratio = 1.0;
        if (dps)
            reduction_ratio = (*dps)[i].get_cap_reduction_ratio(this->get_no(), iter_no);

        vdfps[i].run_bpr(reduction_ratio);
    }
}

inline bool DemandPeriod::contain_iter_no(unsigned short iter_no) const
{
    if (!se)
        return false;

    if (iter_no < se->get_beg_iter_no() - 1 || iter_no > se->get_end_iter_no() - 1)
        return false;

    return true;
}

double DemandPeriod::get_cap_reduction_ratio(size_type link_no, unsigned short iter_no) const
{
    if (!contain_iter_no(iter_no))
        return 1;

    try
    {
        return se->get_cap_reduction_ratio(link_no);
    }
    catch (std::runtime_error& re)
    {
        return 1;
    }
}

void SPNetwork::generate_columns(unsigned short iter_no)
{
    if (!iter_no)
        initialize();

    update_link_costs();

    for (auto s : get_orig_nodes())
    {
        single_source_shortest_path(s);
        backtrace_shortest_path_tree(s, iter_no);
        reset();
    }
}

void SPNetwork::initialize()
{
    const auto n = get_node_num();
    const auto m = get_link_num();

    link_costs = new double [m];
    node_costs = new double [n];
    next_nodes = new long [n];
    link_preds = new long [n];
    node_preds = new long [n];

    for (size_type i = 0; i != m; ++i)
        link_costs[i] = 0;

    for (size_type i = 0; i != n; ++i)
    {
        node_costs[i] = std::numeric_limits<double>::max();
        next_nodes[i] = link_preds[i] = node_preds[i] = nullnode;
    }
}

void SPNetwork::reset()
{
    for (size_type i = 0, n = get_node_num(); i != n; ++i)
    {
        node_costs[i] = std::numeric_limits<double>::max();
        next_nodes[i] = link_preds[i] = node_preds[i] = nullnode;
    }
}

void SPNetwork::update_link_costs()
{
    auto dp_no = dp->get_no();
    auto vot = at->get_vot();

    for (auto p : get_links())
    {
        if (!p->get_length())
            continue;

        link_costs[p->get_no()] = p->get_generalized_cost(dp_no, vot);
    }
}

void SPNetwork::backtrace_shortest_path_tree(size_type src_node_no, unsigned short iter_no)
{
    const auto p = get_nodes()[src_node_no];
    if (p->get_outgoing_links().empty())
        return;

    const auto oz_no = p->get_zone_no();

    for (const auto c : get_centroids())
    {
        auto dz_no = c->get_zone_no();
        if (oz_no == dz_no)
            continue;

        ColumnVecKey cvk {oz_no, dz_no, dp->get_no(), at->get_no()};
        if (!cp->contains_key(cvk))
            continue;

        auto& cv = cp->get_column_vec(cvk);
        if (cv.is_route_fixed())
            continue;

        std::vector<size_type> link_path;
        std::vector<size_type> node_path;

        double dist = 0;
        // use long intensionally as node_preds is long*
        // otherwise, size_t (later size_type) will be automatically deduced via auto.
        long cur_node = c->get_no();
        while (cur_node >= 0)
        {
            node_path.push_back(cur_node);

            auto cur_link = link_preds[cur_node];
            if (cur_link >= 0)
            {
                link_path.push_back(cur_link);
                dist += get_links()[cur_link]->get_length();
            }

            cur_node = node_preds[cur_node];
        }

        if (link_path.size() < 3)
            continue;

        // move temporary Column
        cv.update(Column{cv.get_column_num(), cv.get_volume(), dist, link_path, node_path, node_costs[c->get_no()]}, iter_no);
    }
}

void SPNetwork::single_source_shortest_path(size_type src_node_no)
{
    node_costs[src_node_no] = 0;
    next_nodes[src_node_no] = pastnode;

    for (long cur_node = src_node_no, deq_head = nullnode, deq_tail = nullnode;;)
    {
        // if (src_node_no == 933 && cur_node == 933)
        //     std::cout << "check\n";

        if (cur_node < get_last_thru_node_no() || cur_node == src_node_no)
        {
            for (const auto link : get_nodes()[cur_node]->get_outgoing_links())
            {
                if (!is_mode_compatible(link->get_allowed_modes(), at->get_name()))
                    continue;

                size_type new_node = link->get_tail_node_no();
                // if (new_node == 933)
                //     std::cout << "check\n";

                double new_cost = node_costs[cur_node] + link_costs[link->get_no()];
                if (new_cost < node_costs[new_node])
                {
                    node_costs[new_node] = new_cost;
                    link_preds[new_node] = link->get_no();
                    node_preds[new_node] = link->get_head_node_no();

                    if (next_nodes[new_node] == pastnode)
                    {
                        next_nodes[new_node] = deq_head;
                        deq_head = new_node;

                        if (deq_tail == nullnode)
                            deq_tail = new_node;
                    }
                    else if (next_nodes[new_node] == nullnode && new_node != deq_tail)
                    {
                        if (deq_tail == nullnode)
                        {
                            deq_head = deq_tail = new_node;
                            next_nodes[deq_tail] = nullnode;
                        }
                        else
                        {
                            next_nodes[deq_tail] = new_node;
                            deq_tail = new_node;
                        }
                    }
                }
            }
        }

        if (deq_head < 0)
            break;

        cur_node = deq_head;
        deq_head = next_nodes[cur_node];
        next_nodes[cur_node] = pastnode;

        if (deq_tail == cur_node)
            deq_tail = nullnode;
    }
}

void NetworkHandle::setup_spnetworks()
{
    using SPNKey = std::tuple<unsigned short, unsigned short, unsigned short>;
    std::map<SPNKey, size_type> spn_map;

    this->build_connectors();

    constexpr unsigned memory_blocks = 1;

    size_type i = 0;
    for (auto& [k, z] : this->net.get_zones())
    {
        if (k == "-1")
            continue;

        for (auto& dp : dps)
        {
            for (auto& d : dp.get_demands())
            {
                auto at_no = d.get_agent_type_no();
                auto at = this->ats[at_no];
                if (i < memory_blocks)
                {
                    unsigned short no = spns.size();
                    spn_map[{dp.get_no(), at_no, i}] = no;
                    auto* sp = new SPNetwork {no, this->net, this->cp, dp, at};
                    sp->add_orig_nodes(z);
                    spns.push_back(sp);
                }
                else
                {
                    unsigned short m = i % memory_blocks;
                    auto sp_no = spn_map[{dp.get_no(), at_no, m}];
                    auto sp = this->spns[sp_no];
                    sp->add_orig_nodes(z);
                }
            }
        }

        ++i;
    }
}