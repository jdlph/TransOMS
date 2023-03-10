/**
 * @file ue.cpp, part of the project openDTA under GPL-3.0 license
 * @author jdlph (jdlph@hotmail.com) and xzhou99 (xzhou74@asu.edu)
 * @brief Implementations of Path-based Static User Equilibrium (UE)
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 */

#include <handles.h>

#include <iostream>

using namespace opendta;

void NetworkHandle::find_ue(unsigned short column_gen_num, unsigned short column_opt_num)
{
    setup_spnetworks();

    for (auto i = 0; i != column_gen_num; ++i)
    {
        std::cout << "column generation: " << i << '\n';
        update_link_and_column_volume(i);
        update_link_travel_time(&this->dps, i);
        for (auto spn : this->spns)
            spn->generate_columns(i);
    }

    for (auto i = 0; i != column_opt_num; ++i)
    {
        update_link_and_column_volume(i, false);
        update_link_travel_time();
        update_column_gradient_and_flow(i);
    }

    /**
     * @brief update link flow and link travel time per path flow from the last iteration.
     *
     * path flow will keep constant any more after the last iteration.
     */
    update_link_and_column_volume(column_gen_num, false);
    update_link_travel_time();
    update_column_attributes();
}

void NetworkHandle::update_column_gradient_and_flow(unsigned short iter_no)
{
    double total_gap = 0;
    double total_sys_travel_time = 0;

    for (auto& [k, cv] : this->cp.get_column_vecs())
    {
        // oz_no, dz_no, dp_no, at_no
        auto dp_no = std::get<2>(k);
        auto at_no = std::get<3>(k);
        auto vot = ats[at_no]->get_vot();

        const Column* p = nullptr;
        double least_gradient_cost = std::numeric_limits<double>::max();

        for (auto& col : cv.get_columns())
        {
            double path_gradient_cost = 0;
            for (auto i : col.get_links())
                path_gradient_cost += this->get_link(i)->get_generalized_cost(dp_no, vot);

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
                total_sys_travel_time += col.get_sys_travel_time();
                total_switched_out_vol += const_cast<Column&>(col).shift_volume(iter_no);
            }
        }

        if (p)
        {
            total_sys_travel_time += p->get_sys_travel_time();
            const_cast<Column*>(p)->increase_volume(total_switched_out_vol);
        }
    }

    auto rel_gap = total_sys_travel_time > 0 ? total_gap / total_sys_travel_time : std::numeric_limits<double>::max();
    std::cout << "column updating: " << iter_no
              << "\ntotal gap: " << total_gap << "; relative gap: " << rel_gap * 100 << "%\n";
}

void NetworkHandle::update_column_attributes()
{
    for (auto& [k, cv] : this->cp.get_column_vecs())
    {
        // oz_no, dz_no, dp_no, at_no
        auto dp_no = std::get<2>(k);
        for (auto& col : cv.get_columns())
        {
            double tt = 0;
            double pt = 0;

            for (auto i : col.get_links())
            {
                const auto link = this->get_link(i);
                tt += link->get_period_travel_time(dp_no);
                pt += link->get_toll();
            }

            const_cast<Column&>(col).set_travel_time(tt);
            const_cast<Column&>(col).set_toll(pt);
        }
    }
}

void NetworkHandle::update_link_and_column_volume(unsigned short iter_no, bool reduce_path_vol)
{
    if (!iter_no)
        return;

    // reset link flow
    for (auto link : this->net.get_links())
    {
        if (!link->get_length())
            break;

        link->reset_period_vol();
    }

    for (auto& [k, cv] : this->cp.get_column_vecs())
    {
        // oz_no, dz_no, dp_no, at_no
        auto dp_no = std::get<2>(k);
        auto at_no = std::get<3>(k);
        auto pce = ats[at_no]->get_pce();
        // col is const
        for (auto& col : cv.get_columns())
        {
            auto vol = col.get_volume() * pce;
            for (auto i : col.get_links())
            {
                auto link = this->get_link(i);
                link->increase_period_vol(dp_no, vol);
            }

            if (reduce_path_vol && !cv.is_route_fixed())
                const_cast<Column&>(col).reduce_volume(iter_no);
        }
    }
}

void NetworkHandle::update_link_travel_time(const std::vector<DemandPeriod>* dps, short iter_no)
{
    for (auto link : this->net.get_links())
    {
        if (!link->get_length())
            break;

        link->update_period_travel_time(dps, iter_no);
    }
}