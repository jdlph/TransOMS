#ifndef GUARD_HANDLES_H
#define GUARD_HANDLES_H

#include <demand.h>
#include <supply.h>

#include <iostream>

namespace opendta
{
class NetworkHandle {
public:
    NetworkHandle() = default;

    NetworkHandle(const NetworkHandle&) = delete;
    NetworkHandle& operator=(const NetworkHandle&) = delete;

    NetworkHandle(NetworkHandle&&) = delete;
    NetworkHandle& operator=(NetworkHandle&&) = delete;

    ~NetworkHandle() = default;

    void find_ue(unsigned short column_gen_num, unsigned short column_opt_num)
    {
        for (auto i = 0; i != column_gen_num; ++i)
        {
            update_link_travel_time(&dps, i);
            update_link_and_column_volume(i);
            for (auto& spn : spns)
                spn.generate_columns(i);
        }

        for (auto i = 0; i != column_opt_num; ++i)
        {
            update_link_and_column_volume(i);
            update_link_travel_time();
            update_column_gradient_and_flow(i);
        }

        // post-processing on link flow and link link travel time
        // according to the path flow from the last iteration
        // note that we would not change path flow any more after the last iteration
        update_link_and_column_volume(column_gen_num, false);
        update_link_travel_time();
        update_column_attributes();
    }

    void read_links(const std::string& dir);
    void read_nodes(const std::string& dir);
    void read_demand(const std::string& dir, unsigned short dp_no, unsigned short at_no);
    void read_network(const std::string& dir);

private:
    void update_column_attributes()
    {
        for (auto& [k, cv] : cp.get_column_vecs())
        {
            // oz_no, dz_no, dp_no, at_no
            auto dp_no = std::get<2>(k);
            for (auto& col : cv.get_columns())
            {
                double tt = 0;
                double pt = 0;
                for (auto i : col.get_links())
                {
                    auto link = net.get_links()[i];
                    tt += link->get_period_travel_time(dp_no);
                    pt += link->get_toll();
                }

                const_cast<Column&>(col).set_travel_time(tt);
                const_cast<Column&>(col).set_toll(pt);
            }
        }
    }

    void update_link_and_column_volume(unsigned short iter_no, bool reduce_path_vol = true)
    {
        if (!iter_no)
            return;

        // reset link flow
        for (auto link : net.get_links())
        {
            if (!link->get_length())
                continue;

            link->reset_period_vol();
        }

        for (auto& [k, cv] : cp.get_column_vecs())
        {
            // oz_no, dz_no, dp_no, at_no
            auto dp_no = std::get<2>(k);
            auto at_no = std::get<3>(k);
            auto pce = ats[at_no].get_pce();
            // col is const
            for (auto& col : cv.get_columns())
            {
                auto vol = col.get_volume() * pce;
                for (auto i : col.get_links())
                    net.get_links()[i]->increase_period_vol(dp_no, vol);

                if (reduce_path_vol && !cv.is_route_fixed())
                    const_cast<Column&>(col).reduce_volume(iter_no);
            }
        }
    }

    // a little bit ugly
    void update_link_travel_time(const std::vector<DemandPeriod>* dps = nullptr, short iter_no = -1)
    {
        for (auto link : net.get_links())
        {
            if (!link->get_length())
                continue;

            link->update_period_travel_time(dps, iter_no);
        }
    }

    void update_column_gradient_and_flow(unsigned short iter_no)
    {
        double total_gap = 0;
        double total_travel_time = 0;

        for (auto& [k, cv] : cp.get_column_vecs())
        {
            // oz_no, dz_no, dp_no, at_no
            auto dp_no = std::get<2>(k);
            auto at_no = std::get<3>(k);
            auto vot = ats[at_no].get_vot();

            const Column* p = nullptr;
            double least_gradient_cost = std::numeric_limits<double>::max();

            for (auto& col : cv.get_columns())
            {
                double path_gradient_cost = 0;
                for (auto i : col.get_links())
                    path_gradient_cost += net.get_links()[i]->get_generalized_cost(dp_no, vot);

                const_cast<Column&>(col).set_gradient_cost(path_gradient_cost);
                if (path_gradient_cost < least_gradient_cost)
                {
                    least_gradient_cost = path_gradient_cost;
                    p = &col;
                }
            }

            double total_switched_out_vol = 0;
            if (cv.get_column_num() >= 2)
            {
                for (auto& col : cv.get_columns())
                {
                    if (&col == p)
                        continue;

                    const_cast<Column&>(col).update_gradient_cost_diffs(least_gradient_cost);

                    total_gap += col.get_gap();
                    total_travel_time += col.get_sys_travel_time();
                    total_switched_out_vol += const_cast<Column&>(col).shift_volume(iter_no);
                }
            }

            if (p)
            {
                const_cast<Column*>(p)->increase_volume(total_switched_out_vol);
                total_travel_time += p->get_sys_travel_time();
            }
        }

        auto rel_gap = total_travel_time > 0 ? total_gap / total_travel_time : std::numeric_limits<double>::max();

        std::cout << "total gap: " << total_gap << "\nrelative gap: " << rel_gap << "\n";
    }

private:
    ColumnPool cp;

    PhyNetwork net;
    std::vector<SPNetwork> spns;

    std::vector<AgentType> ats;
    std::vector<DemandPeriod> dps;
};

} // namespace opendta

#endif