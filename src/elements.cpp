/**
 * @file elements.cpp, part of the project TransOMS under Apache License 2.0
 * @author jdlph (jdlph@hotmail.com) and xzhou99 (xzhou74@asu.edu)
 * @brief Implementations of member functions from demand and supply classes
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 */

#include <demand.h>
#include <handles.h>
#include <supply.h>

#include <stdexcept>
#include <iostream>

using namespace transoms;

inline const std::vector<size_type>& Agent::get_link_path() const
{
    return col->get_links();
}

inline const std::vector<size_type>& Agent::get_node_path() const
{
    return col->get_nodes();
}

void Agent::initialize_intervals()
{
    // throw an error or terminate?
    if (!col)
        return;

    auto n = col->get_node_num();

    arr_intvls.resize(n);
    dep_intvls.resize(n);
    curr_link_no = n - 1;
}

inline bool DemandPeriod::contain_iter_no(unsigned short iter_no) const
{
    if (!se)
        return false;

    if (iter_no < se->get_beg_iter_no() - 1 || iter_no > se->get_end_iter_no() - 1)
        return false;

    return true;
}

double DemandPeriod::get_cap_reduction_ratio(const std::string& link_id, unsigned short iter_no) const
{
    if (!contain_iter_no(iter_no))
        return 1;

    try
    {
        return se->get_cap_reduction_ratio(link_id);
    }
    catch (std::out_of_range& re)
    {
        return 1;
    }
}

bool ColumnVec::has_column(const Column& c) const
{
    if (cols.find(c) != cols.end())
        return true;

    return false;
}

void ColumnVec::update(Column&& c, unsigned short iter_no)
{
    // k_path_prob = 1 / (iter_no + 1)
    auto v = vol / (iter_no + 1);

    auto it = cols.find(c);
    if (it != cols.end())
        const_cast<Column&>(*it).increase_volume(v);
    else
    {
        c.increase_volume(v);
        add_new_column(c);
    }
}

void Link::update_period_travel_time(const std::vector<const DemandPeriod*>* dps, short iter_no)
{
    for (auto i = 0; i != vdfps.size(); ++i)
    {
        auto reduction_ratio = 1.0;
        if (dps)
            reduction_ratio = (*dps)[i]->get_cap_reduction_ratio(this->get_id(), iter_no);

        vdfps[i].run_bpr(reduction_ratio);
    }
}

void SPNetwork::generate_columns(unsigned short iter_no)
{
    if (!iter_no)
        initialize();

    update_link_costs();

    for (auto s : get_orig_nodes())
    {
#ifdef MLC_DEQUE
        single_source_shortest_path(s);
#else
        single_source_shortest_path_dijkstra(s);
#endif
        backtrace_shortest_path_tree(s, iter_no);
        reset();
    }
}

void SPNetwork::initialize()
{
    const auto n = get_node_num();
    const auto m = get_link_num();

    link_costs = double_alloc.allocate(m);
    node_costs = double_alloc.allocate(n);
    link_preds = link_alloc.allocate(n);

#ifdef MLC_DEQUE
    next_nodes = long_alloc.allocate(n);
#else
    marked = bool_alloc.allocate(n);
    min_heap.reserve(n);
#endif

    for (size_type i = 0; i != m; ++i)
        link_costs[i] = 0;

    for (size_type i = 0; i != n; ++i)
    {
        node_costs[i] = std::numeric_limits<double>::max();
        link_preds[i] = nullptr;
#ifdef MLC_DEQUE
        next_nodes[i] = null_node;
#else
        marked[i] = false;
        min_heap.reset();
#endif
    }
}

inline void SPNetwork::reset()
{
    for (size_type i = 0, n = get_node_num(); i != n; ++i)
    {
        node_costs[i] = std::numeric_limits<double>::max();
        link_preds[i] = nullptr;
#ifdef MLC_DEQUE
        next_nodes[i] = null_node;
#else
        marked[i] = false;
        min_heap.reset();
#endif
    }
}

void SPNetwork::update_link_costs()
{
    auto dp_no = dp->get_no();
    auto vot = at->get_vot();

    for (auto p : get_links())
    {
        if (!p->get_length())
            break;

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

        double dist = 0;
        for (auto link = link_preds[c->get_no()]; link;)
        {
            // the first and the last are connectors, use link->get_length() to eliminate them
            if (link->get_length())
                link_path.push_back(link->get_no());

            dist += link->get_length();
            link = link_preds[link->get_head_node_no()];
        }

        if (link_path.empty())
            continue;

        // move temporary Column. note that link_path will be moved as well!
        cv.update(Column{cv.get_column_num(), cv.get_volume(), dist, link_path}, iter_no);
    }
}

#ifdef MLC_DEQUE
// the most efficient deque implementation of the MLC algorithm adopted from Path4GMNS
void SPNetwork::single_source_shortest_path(size_type src_node_no)
{
    node_costs[src_node_no] = 0;
    next_nodes[src_node_no] = past_node;
    // use long intentionally
    for (long cur_node = src_node_no, deq_head = null_node, deq_tail = null_node;;)
    {
        // no centroid traversing
        if (cur_node < get_last_thru_node_no() || cur_node == src_node_no)
        {
            for (const auto link : get_nodes()[cur_node]->get_outgoing_links())
            {
                if (!is_mode_compatible(link->get_allowed_modes(), at->get_name()))
                    continue;

                auto new_node = link->get_tail_node_no();
                auto new_cost = node_costs[cur_node] + link_costs[link->get_no()];
                if (new_cost < node_costs[new_node])
                {
                    node_costs[new_node] = new_cost;
                    link_preds[new_node] = link;

                    /**
                     * three cases
                     *
                     * case i: new_node was in deque before, add it to the begin of deque
                     * case ii: new_node is not in deque, and wasn't there before, add it to the end of deque
                     * case iii: new_node is in deque, do nothing
                     */
                    if (next_nodes[new_node] == past_node)
                    {
                        next_nodes[new_node] = deq_head;
                        deq_head = new_node;

                        if (deq_tail == null_node)
                            deq_tail = new_node;
                    }
                    else if (next_nodes[new_node] == null_node && new_node != deq_tail)
                    {
                        if (deq_tail == null_node)
                        {
                            deq_head = deq_tail = new_node;
                            next_nodes[deq_tail] = null_node;
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
        next_nodes[cur_node] = past_node;

        if (deq_tail == cur_node)
            deq_tail = null_node;
    }
}

#else
// heap-Dijkstra adopted from https://github.com/jdlph/shortest-path-algorithms
void SPNetwork::single_source_shortest_path_dijkstra(size_type src_node_no)
{
    node_costs[src_node_no] = 0;
    min_heap.emplace(src_node_no, 0);
    // std::cout << src_node_no << '\n';
    while (!min_heap.empty())
    {
        // auto& min_node = min_heap.top();
        // auto cur_node = min_node.node_no;
        // auto cur_cost = min_node.cost;
        auto [cur_cost, cur_node] = min_heap.top();

        /**
         * @brief this is not an O(logn) time operation with std::priority_queue!
         *
         * it takes O(logn) time to sort heap and an additional O(n) time from
         * its underlying std::vector to perform pop_front(), where n is the number
         * of elements in min_heap at runtime. Therefore, its time bound is
         * O(logn) + O(n) = O(n), which is linear rather than logarithmic!
         *
         * pop_front() from std::deque takes O(logn) time. Replacing the underlying
         * container of std::priority_queue with std::deque will literally make
         * the time bound as O(logn) (i.e., O(logn) + O(logn) = O(logn)). However,
         * the performance is subject to OS and complier with respect to the choice
         * of block size in deque's implementation. The running time may be better
         * or WORSE than that of using std::vector. See the following link for more
         * discussions.
         *
         * A special min-heap, which guarantees logarithmic time pop(), will be
         * introduced later.
         *
         * https://github.com/jdlph/shortest-path-algorithms#more-discussion-on-the-deque-implementations-in-c
         */
        min_heap.pop();

        // cur_node has been scanned / labeled
        if (marked[cur_node])
            continue;

        marked[cur_node] = true;

        std::cout << src_node_no << "; " << cur_node << "; " << cur_cost << '\n';
        
        if (cur_node == 31 && src_node_no == 936)
            std::cout << "check\n";

        // no centroid traversing
        if (cur_node < get_last_thru_node_no() || cur_node == src_node_no)
        {
            for (const auto link : get_nodes()[cur_node]->get_outgoing_links())
            {
                auto new_node = link->get_tail_node_no();
                auto new_cost = cur_cost + link_costs[link->get_no()];
                if (new_cost < node_costs[new_node])
                {
                    if (src_node_no == 936 && new_node == 506)
                        int kk = 1;                    
                    node_costs[new_node] = new_cost;
                    link_preds[new_node] = link;
                    min_heap.emplace(new_node, new_cost);
                }
            }
        }
    }
}
#endif

void NetworkHandle::setup_spnetworks()
{
    using SPNKey = std::tuple<unsigned short, unsigned short, unsigned short>;
    std::map<SPNKey, size_type> spn_map;

    build_connectors();

    constexpr unsigned memory_blocks = 1;

    size_type i = 0;
    for (auto& [k, z] : this->net.get_zones())
    {
        for (auto& dp : dps)
        {
            for (auto& d : dp->get_demands())
            {
                auto at_no = d.get_agent_type_no();
                auto at = this->ats[at_no];
                if (i < memory_blocks)
                {
                    unsigned short no = spns.size();
                    spn_map[{dp->get_no(), at_no, i}] = no;
                    auto sp = new SPNetwork {no, this->net, this->cp, dp, at};
                    sp->add_orig_nodes(z);
                    spns.push_back(sp);
                }
                else
                {
                    unsigned short m = i % memory_blocks;
                    auto sp_no = spn_map[{dp->get_no(), at_no, m}];
                    auto sp = this->spns[sp_no];
                    sp->add_orig_nodes(z);
                }
            }
        }

        ++i;
    }
}

void NetworkHandle::build_connectors()
{
    auto node_no = this->net.get_node_num();
    auto link_no = this->net.get_link_num();

    this->net.set_last_thru_node_no(node_no);

    for (auto& [k, z] : this->net.get_zones())
    {
        auto [x, y] = z->get_coordinate();
        if (x == COORD_X || y == COORD_Y)
        {
            auto node_no_ = z->get_nodes()[0];
            auto node = this->net.get_nodes()[node_no_];
            x = node->get_coordinate().first;
            y = node->get_coordinate().second;
        }

        auto node = new Node {node_no, std::string {"c_" + std::to_string(z->get_no())}, x, y, z->get_no()};
        z->set_centroid(node);
        this->net.add_node(node);

        // build connectors
        for (auto i : z->get_nodes())
        {
            auto head_node = net.get_nodes()[node_no];
            auto tail_node = net.get_nodes()[i];

            auto forward_link = new Link {std::string{"conn_" + std::to_string(link_no)},
                                          link_no, head_node->get_no(), tail_node->get_no()};

            auto backward_link = new Link {std::string{"conn_" + std::to_string(link_no + 1)},
                                           link_no + 1, tail_node->get_no(), head_node->get_no()};

            this->net.add_link(forward_link);
            this->net.add_link(backward_link);

            link_no += 2;
        }

        ++node_no;
    }

    this->net.collect_centroids();
}

void NetworkHandle::delete_spnetworks()
{
    for (auto spn : this->spns)
        delete spn;

    this->spns.clear();
}