/**
 * @file simulation.cpp, part of the project TransOMS under Apache License 2.0
 * @author jdlph (jdlph@hotmail.com) and xzhou99 (xzhou74@asu.edu)
 * @brief Implementations of simulation-based Dynamic Traffic Assignment (DTA)
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 */

#include <handles.h>

#include <cmath>
#include <iostream>

using namespace transoms;

inline unsigned short NetworkHandle::get_simulation_intervals() const
{
    return std::ceil(this->simu_dur * SECONDS_IN_MINUTE / this->simu_res);
}

const std::vector<size_type>& NetworkHandle::get_agents_at_interval(unsigned short i) const
{
    return this->net.get_agents_at_interval(i);
}

Agent& NetworkHandle::get_agent(size_type no)
{
    return this->net.get_agent(no);
}

const Agent& NetworkHandle::get_agent(size_type no) const
{
    return this->net.get_agent(no);
}

void NetworkHandle::setup_agent_dep_time()
{

}

void NetworkHandle::run_simulation()
{
    size_type cum_arr = 0;
    size_type cum_dep = 0;
    // number of simulation intervals in one minute
    const unsigned short num = std::ceil(SECONDS_IN_MINUTE / this->get_simulation_resolution());

    for (auto t = 0; t != this->get_simulation_intervals(); ++t)
    {
        if (t % num == 0)
            std::cout << "simulation time = " << t / num << " min, CA = "
                      << cum_arr << ", CD = " << cum_dep << '\n';

        for (auto at_no : this->get_agents_at_interval(t))
        {
            const auto& at = this->get_agent(at_no);
            if (!at.get_link_num())
                continue;

            auto link = this->get_link(at.get_last_link_no());
            ++cum_arr;
        }

        for (auto link : this->net.get_links())
        {

        }

        for (auto node : this->net.get_nodes())
        {

        }
    }
}