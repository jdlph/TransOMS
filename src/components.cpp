#include <demand.h>
#include <network.h>

inline const std::vector<size_t>& Agent::get_link_path() const
{
    return col->get_links();
}

inline const std::vector<size_t>& Agent::get_node_path() const
{
    return col->get_nodes();
}

void Link::update_period_travel_time(unsigned iter_no)
{
    for (auto& v: vdfps)
        v.run_bpr();
}