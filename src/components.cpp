#include <demand.h>
#include <supply.h>

inline const std::vector<size_t>& Agent::get_link_path() const
{
    return col->get_links();
}

inline const std::vector<size_t>& Agent::get_node_path() const
{
    return col->get_nodes();
}

// incomplete
void Link::update_period_travel_time(unsigned iter_no)
{
    for (auto& v: vdfps)
        v.run_bpr();
}

inline bool DemandPeriod::contain_iter_no(unsigned iter_no) const
{
    if (!se)
        return false;

    if (iter_no < se->get_beg_iter_no() - 1 || iter_no > se->get_end_iter_no() - 1)
        return false;

    return true;
}

double DemandPeriod::get_cap_reduction_ratio(size_t link_no, unsigned iter_no) const
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