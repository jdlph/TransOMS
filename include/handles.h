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
            update_link_travel_time(i);
            update_link_and_column_volume(i);
            for (auto& spn : spns)
                spn.generate_columns(i);
        }

        for (auto i = 0; i != column_opt_num; ++i)
            update_column_gradient_and_flow(i);

        update_link_and_column_volume(column_gen_num, false);
        update_link_travel_time(column_gen_num);
        update_column_attributes();
    }

private:
    void update_column_attributes()
    {
        for (auto& [k, cv] : cp.get_column_vecs())
        {
            // oz_id, dz_id, dp_id
            auto dp_id = std::get<2>(k);
            for (auto& col : cv.get_columns())
            {
                double tt = 0;
                double pt = 0;
                for (auto i : col.get_links())
                {
                    auto link = net.get_links()[i];
                    tt += link->get_period_travel_time(dp_id);
                    pt += link->get_toll();

                    const_cast<Column&>(col).set_travel_time(tt);
                    // useless?
                    const_cast<Column&>(col).set_toll(pt);
                }
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
            // col is const??
            for (auto& col : cv.get_columns())
            {
                auto vol = col.get_volume();
                for (auto i : col.get_links())
                    net.get_links()[i]->increase_period_vol(vol);

                if (reduce_path_vol && !cv.is_route_fixed())
                    const_cast<Column&>(col).reduce_volume(vol);
            }
        }
    }

    void update_link_travel_time(unsigned short iter_no)
    {
        if (!iter_no)
            return;

        for (auto link : net.get_links())
        {
            if (!link->get_length())
                continue;

            link->update_period_travel_time(iter_no);
        }
    }

    void update_column_gradient_and_flow(unsigned short iter_no)
    {
        update_link_and_column_volume(iter_no);
        update_link_travel_time(iter_no);

        double total_gap = 0;
        double total_travel_time = 0;

        for (auto& [k, cv] : cp.get_column_vecs())
        {
            // oz_id, dz_id, dp_id
            auto dp_id = std::get<2>(k);
            double least_gradient_cost = std::numeric_limits<double>::max();
            const Column* p = nullptr;

            for (auto& col : cv.get_columns())
            {
                double path_gradient_cost = 0;
                for (auto i : col.get_links())
                {
                    // update it later
                    path_gradient_cost += net.get_links()[i]->get_generalized_cost(dp_id, 10);
                    const_cast<Column&>(col).set_gradient_cost(path_gradient_cost);

                    if (path_gradient_cost < least_gradient_cost)
                    {
                        least_gradient_cost = path_gradient_cost;
                        p = &col;
                    }
                }
            }

            double total_switched_out_vol = 0;
            if (cv.get_column_num() >= 2)
            {
                for (auto& col : cv.get_columns())
                {
                    if (&col == p)
                        continue;

                    // the api's can be optimized
                    const_cast<Column&>(col).set_gradient_cost_abs_diff(col.get_gradient_cost() - least_gradient_cost);
                    const_cast<Column&>(col).set_gradient_cost_rel_diff(col.get_gradient_cost_abs_diff() / std::max(0.00001, least_gradient_cost));

                    total_gap += col.get_gradient_cost_abs_diff() * col.get_volume();
                    total_travel_time += col.get_gradient_cost() * col.get_volume();

                    auto step_size = 1.0 / (iter_no + 2) * cv.get_volume();
                    auto prev_path_vol = col.get_volume();
                    auto vol = std::max(0.0, (prev_path_vol - step_size * col.get_gradient_cost_rel_diff()));

                    const_cast<Column&>(col).set_volume(vol);
                    total_switched_out_vol += prev_path_vol - col.get_volume();
                }
            }

            if (p)
            {
                const_cast<Column*>(p)->increase_volume(total_switched_out_vol);
                total_travel_time += p->get_gradient_cost() * p->get_volume();
            }
        }

        auto rel_gap = total_travel_time > 0 ? total_gap / total_travel_time : std::numeric_limits<double>::max();

        std::cout << "total gap: " << total_gap << "\n";
        std::cout << "relative gap: " << rel_gap << "\n";
    }

    // useless
    void optimize_column_pool(unsigned short update_num)
    {
        for (auto i = 0; i != update_num; ++i)
            update_column_gradient_and_flow(i);
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