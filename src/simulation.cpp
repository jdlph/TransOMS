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

inline size_type NetworkHandle::get_simulation_intervals() const
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

LinkQueue& NetworkHandle::get_link_queue(size_type i)
{
    return this->link_queues[i];
}

void NetworkHandle::setup_agent_dep_time()
{

}

void NetworkHandle::setup_link_queues()
{
    for (const auto link : this->net.get_links())
    {
        link_queues.push_back(LinkQueue{link, this->get_simulation_intervals(),
                              this->simu_dur, this->simu_res});
    }
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

        for (auto a_no : this->get_agents_at_interval(t))
        {
            const auto& agent = this->get_agent(a_no);
            if (!agent.get_link_num())
                continue;

            auto& link_que = this->get_link_queue(agent.get_last_link_no());
            link_que.append_entr_queue(a_no);
            ++cum_arr;
        }

        for (auto& link_que : this->link_queues)
        {
            while (!link_que.is_entr_queue_empty())
            {
                auto a_no = link_que.get_entr_queue_front();
                auto& agent = this->get_agent(a_no);
                auto tt = std::ceil(link_que.get_period_travel_time(0) * num);

                link_que.append_exit_queue(a_no);
                agent.update_deq_interval(tt);
            }
        }

        for (auto node : this->net.get_nodes())
        {
            for (size_type i = 0, m = node->get_incoming_links().size(); i != m; ++i)
            {
                auto j = (t + i) % m;
                auto link_no = node->get_incoming_links()[j]->get_no();
                auto& link_que = this->get_link_queue(link_no);

                while (link_que.has_outflow_cap(t) && !link_que.is_exit_queue_empty())
                {
                    auto a_no = link_que.get_exit_queue_front();
                    auto& agent = this->get_agent(a_no);

                    if (agent.get_dep_interval() > t)
                        break;

                    if (agent.reaches_last_link())
                    {
                        link_que.increment_cum_dep(t);
                        ++cum_dep;
                    }
                    else
                    {
                        auto link_no_ = agent.get_next_link_no();
                        auto& next_link_que = this->get_link_queue(link_no_);

                        next_link_que.append_entr_queue(a_no);
                        agent.set_dep_interval(t);
                        agent.set_arr_interval(t);

                        auto travel_time = t - agent.get_arr_interval();
                        auto waiting_time = travel_time - link_que.get_period_travel_time(0);
                        auto arrival_time = std::floor(agent.get_arr_interval() / num);

                        link_que.update_waiting_time(arrival_time, waiting_time);
                        link_que.increment_cum_dep(t);
                        next_link_que.increment_cum_arr(t);
                    }

                    agent.move_to_next_link();
                    // remove agent (i.e., a_no) from exit queue
                    link_que.pop_exit_queue_front();
                    link_que.deduct_outflow_cap(t);
                }
            }
        }
    }
}