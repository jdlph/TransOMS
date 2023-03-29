/**
 * @file ue.cpp, part of the project TransOMS under Apache License 2.0
 * @author jdlph (jdlph@hotmail.com) and xzhou99 (xzhou74@asu.edu)
 * @brief Implementations of Path-based Static User Equilibrium (UE)
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 */

#include <handles.h>

#include <chrono>
#include <cmath>
#include <iostream>

#ifdef PARALLEL
#include <omp.h>
#endif

using namespace transoms;
using namespace std::chrono;

void NetworkHandle::find_ue(unsigned short column_gen_num, unsigned short column_opt_num)
{
    auto ts = high_resolution_clock::now();
    setup_spnetworks();

    for (auto i = 0; i != column_gen_num; ++i)
    {
        std::cout << "column generation: " << i << '\n';
        update_link_and_column_volume(i);
        update_link_travel_time();

#ifdef _OPENMP
        #pragma omp parallel for
#endif
        for (auto spn : this->spns)
            spn->generate_columns(i);
    }

    auto te = high_resolution_clock::now();
    std::cout << "TransOMS completes column generation in " << duration_cast<milliseconds>(te - ts).count() << " milliseconds\n";

    for (auto i = 0; i != column_opt_num;)
    {
        update_column_gradient_and_flow(i);
        update_link_and_column_volume(++i, false);
        update_link_travel_time();
    }

    /**
     * @brief update link flow and link travel time per path flow from the last iteration.
     *
     * path flow will keep constant any more after the last iteration.
     */
    update_column_attributes();
}

void NetworkHandle::update_column_gradient_and_flow(unsigned short iter_no)
{
#ifndef _OPENMP
    double total_gap = 0;
    double total_sys_travel_time = 0;
#endif

#ifdef _OPENMP
    #pragma omp parallel for schedule(dynamic, CHUNK)
#endif
    for (auto& cv : this->cp.get_column_vecs())
    {
        if (!cv.get_column_num())
            continue;

        // oz_no, dz_no, dp_no, at_no
        auto dp_no = std::get<2>(cv.get_key());
        auto at_no = std::get<3>(cv.get_key());
        auto vot = ats[at_no]->get_vot();

        const Column* p = nullptr;
        double least_gradient_cost = std::numeric_limits<double>::max();
        // col is const
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

#ifndef _OPENMP
                total_gap += col.get_gap();
                total_sys_travel_time += col.get_sys_travel_time();
#endif
                total_switched_out_vol += const_cast<Column&>(col).shift_volume(iter_no);
            }
        }

        if (p)
        {
#ifndef _OPENMP
            total_sys_travel_time += p->get_sys_travel_time();
#endif
            const_cast<Column*>(p)->reset_gradient_diffs();
            const_cast<Column*>(p)->increase_volume(total_switched_out_vol);
        }
    }

#ifndef _OPENMP
    auto rel_gap = total_sys_travel_time > 0 ? total_gap / total_sys_travel_time : std::numeric_limits<double>::max();
    std::cout << "column updating: " << iter_no
              << "\ntotal gap: " << total_gap << "; relative gap: " << rel_gap * 100 << "%\n";
#else
    std::cout << "column updating: " << iter_no << '\n';
#endif
}

void NetworkHandle::update_column_attributes()
{
    double total_gap = 0;
    double total_sys_travel_time = 0;

#ifdef _OPENMP
    #pragma omp parallel for shared(total_gap, total_sys_travel_time) schedule(dynamic, CHUNK)
#endif
    for (auto& cv : this->cp.get_column_vecs())
    {
        // oz_no, dz_no, dp_no, at_no
        auto dp_no = std::get<2>(cv.get_key());
        // col is const
        for (auto& col : cv.get_columns())
        {
            // avoid data racing
            #pragma omp atomic
            total_gap += col.get_gap();
            #pragma omp atomic
            total_sys_travel_time += col.get_sys_travel_time();

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

    auto rel_gap = total_sys_travel_time > 0 ? total_gap / total_sys_travel_time : std::numeric_limits<double>::max();
    std::cout << "Final UE Convergency | total gap: " << total_gap << "; relative gap: " << rel_gap * 100 << "%\n";
}

void NetworkHandle::update_link_and_column_volume(unsigned short iter_no, bool reduce_path_vol)
{
    if (!iter_no)
        return;

    // reset link flow
#ifdef _OPENMP
    #pragma omp parallel for
#endif
    for (auto link : this->net.get_links())
    {
        if (!link->get_length())
#ifdef _OPENMP
            continue;
#else
            break;
#endif

        link->reset_period_vol();
    }

    for (auto& cv : this->cp.get_column_vecs())
    {
        if (!cv.get_column_num())
            continue;

        // oz_no, dz_no, dp_no, at_no
        auto dp_no = std::get<2>(cv.get_key());
        auto at_no = std::get<3>(cv.get_key());
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

void NetworkHandle::update_link_travel_time()
{
#ifdef _OPENMP
    #pragma omp parallel for
#endif
    for (auto link : this->net.get_links())
    {
        if (!link->get_length())
#ifdef _OPENMP
            continue;
#else
            break;
#endif

        link->update_period_travel_time();
    }
}