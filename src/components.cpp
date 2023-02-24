#include <demand.h>
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