/**
 * @file ue.cpp, part of the project TransOMS under Apache License 2.0
 * @author jdlph (jdlph@hotmail.com) and xzhou99 (xzhou74@asu.edu)
 * @brief Implementations of Path-based Static User Equilibrium (UE)
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 */

#include <config.h>
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
        for (auto j = 0; j < this->spns.size(); ++j)
            this->spns[j]->generate_columns(i);
    }

    auto te = high_resolution_clock::now();
    std::cout << "TransOMS completes column generation in " << duration_cast<milliseconds>(te - ts).count() << " milliseconds\n";

    for (auto i = 0; i != column_opt_num;)
    {
        update_link_and_column_volume(i, false);
        update_link_travel_time();
        update_column_gradient_and_flow(i++);
    }

    /**
     * @brief update link flow and link travel time per path flow from the last iteration.
     * @note path flow will remain unchanged after the last iteration.
     */
    update_link_and_column_volume(1, false);
    update_link_travel_time();
    update_column_attributes();
}

void NetworkHandle::update_column_gradient_and_flow(unsigned short iter_no)
{
    const auto cv_num = this->cp.get_column_vecs().size();

#ifndef _OPENMP
    double total_gap = 0;
    double total_sys_travel_time = 0;
#else
    #pragma omp parallel for schedule(dynamic, CHUNK)
#endif
    for (auto i = 0; i < cv_num; ++i)
    {
        auto& cv = this->cp.get_column_vec(i);
        if (!cv.get_column_num())
            continue;

        // oz_no, dz_no, dp_no, at_no
        const auto dp_no = std::get<2>(cv.get_key());
        const auto at_no = std::get<3>(cv.get_key());
        const auto vot = ats[at_no]->get_vot();

        const Column* p = nullptr;
        double least_gradient_cost = std::numeric_limits<double>::max();
        // col is const
        for (auto& col : cv.get_columns())
        {
            double path_gradient_cost = 0;
            for (auto j : col.get_links())
                path_gradient_cost += this->get_link(j)->get_generalized_cost(dp_no, vot);

            const_cast<Column&>(col).set_gradient_cost(path_gradient_cost);

            if (path_gradient_cost < least_gradient_cost && col.get_volume() >= EPSILON)
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

                if (!col.get_volume())
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
    const auto cv_num = this->cp.get_column_vecs().size();

#ifdef _OPENMP
    #pragma omp parallel for shared(total_gap, total_sys_travel_time) schedule(dynamic, CHUNK)
#endif
    for (auto i = 0; i < cv_num; ++i)
    {
        auto& cv = this->cp.get_column_vec(i);
        if (!cv.get_column_num())
            continue;

        // oz_no, dz_no, dp_no, at_no
        const auto dp_no = std::get<2>(cv.get_key());
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

            for (auto j : col.get_links())
            {
                const auto link = this->get_link(j);
                tt += link->get_period_travel_time(dp_no);
                pt += link->get_toll();
            }

            const_cast<Column&>(col).set_travel_time(tt);
            const_cast<Column&>(col).set_toll(pt);
        }
    }

    auto rel_gap = total_sys_travel_time > 0 ? total_gap / total_sys_travel_time : std::numeric_limits<double>::max();
    std::cout << "column updating: postprocessing\n"
              << "total gap: " << total_gap << "; relative gap: " << rel_gap * 100 << "%\n";
}

void NetworkHandle::update_link_and_column_volume(unsigned short iter_no, bool reduce_path_vol)
{
    if (!iter_no)
        return;

    const auto m = this->net.get_link_num();

    // reset link flow
#ifdef _OPENMP
    #pragma omp parallel for
#endif
    for (auto i = 0; i < m; ++i)
    {
        auto link = this->get_link(i);
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
        const auto dp_no = std::get<2>(cv.get_key());
        const auto at_no = std::get<3>(cv.get_key());
        const auto pce = ats[at_no]->get_pce();
        // col is const
        for (auto& col : cv.get_columns())
        {
            auto vol = col.get_volume() * pce;
            for (auto j : col.get_links())
            {
                auto link = this->get_link(j);
                link->increase_period_vol(dp_no, vol);
            }

            if (reduce_path_vol && !cv.is_route_fixed())
                const_cast<Column&>(col).reduce_volume(iter_no);
        }
    }
}

void NetworkHandle::update_link_travel_time()
{
    const auto m = this->net.get_link_num();

#ifdef _OPENMP
    #pragma omp parallel for
#endif
    for (auto i = 0; i < m; ++i)
    {
        auto link = this->get_link(i);
        if (!link->get_length())
#ifdef _OPENMP
            continue;
#else
            break;
#endif

        link->update_period_travel_time();
    }
}