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

inline const std::vector<size_type>& Agent::get_link_path() const
{
    return col->get_links();
}

inline const std::vector<size_type>& Agent::get_node_path() const
{
    return col->get_nodes();
}

inline std::vector<size_type>::size_type Agent::get_link_num() const
{
    return col->get_link_num();
}

inline size_type Agent::get_last_link_no() const
{
    return col->get_last_link_no();
}

void Agent::initialize_intervals()
{
    // throw an error or terminate?
    if (!col)
        return;

    auto n = col->get_link_num();

    arr_intvls.resize(n);
    dep_intvls.resize(n);
    curr_link_no = n - 1;
}

inline size_type NetworkHandle::get_simulation_intervals() const
{
    return std::ceil(this->simu_dur * SECONDS_IN_MINUTE / this->simu_res);
}

const std::vector<size_type>& NetworkHandle::get_agents_at_interval(unsigned short i) const
{
    return this->td_agents.at(i);;
}

Agent& NetworkHandle::get_agent(size_type no)
{
    return this->agents[no];
}

const Agent& NetworkHandle::get_agent(size_type no) const
{
    return this->agents[no];
}

unsigned short NetworkHandle::get_demand_period_no(size_type i) const
{
    for (auto j = 0; j != this->time_periods.size(); ++j)
    {
        if (i < this->time_periods[j])
            return j;
    }

    throw std::string{std::to_string(i) + " is out of the simulation duration"};
}

LinkQueue& NetworkHandle::get_link_queue(size_type i)
{
    return this->link_queues[i];
}

void NetworkHandle::setup_agents()
{
    size_type agent_no = 0;
    for (const auto& cv : this->cp.get_column_vecs())
    {
        // oz_no, dz_no, dp_no, at_no
        auto oz_no = std::get<0>(cv.get_key());
        auto dz_no = std::get<1>(cv.get_key());
        auto dp_no = std::get<2>(cv.get_key());
        auto at_no = std::get<3>(cv.get_key());

        for (const auto& col : cv.get_columns())
        {
            for (size_type i = 0, vol = std::ceil(col.get_volume()); i != vol; ++i)
            {
                // avoid copy by constructing Agent object in place
                this->agents.emplace_back(agent_no, at_no, dp_no, oz_no, dz_no, &col);
                auto delta = static_cast<unsigned short>(i / col.get_volume() * this->dps[dp_no]->get_duration());

                auto intvl = delta * SECONDS_IN_MINUTE / this->simu_res;
                auto dep_time = this->dps[dp_no]->get_start_time() + delta;

                auto& agent = this->get_agent(agent_no);
                agent.set_arr_interval(intvl);
                agent.set_dep_time(dep_time);

                this->td_agents[intvl].push_back(agent_no++);
            }
        }
    }

    // this requires that time periods are consecutive
    size_type beg_intvl = 0;
    for (const auto dp : this->dps)
    {
        beg_intvl += dp->get_duration() * SECONDS_IN_MINUTE / this->simu_res;
        this->time_periods.push_back(beg_intvl);
    }
}

void NetworkHandle::setup_link_queues()
{
    for (const auto link : this->net.get_links())
    {
        link_queues.emplace_back(link, this->get_simulation_intervals(),
                                 this->simu_dur, this->simu_res);
    }
}

void NetworkHandle::run_simulation()
{
    this->setup_agents();
    if (this->td_agents.empty())
        return;

    this->setup_link_queues();
    if (this->link_queues.empty())
        return;

    size_type cum_arr = 0;
    size_type cum_dep = 0;
    // number of simulation intervals in one minute
    const unsigned short num = std::ceil(SECONDS_IN_MINUTE / this->simu_res);

    for (auto t = 0; t != this->get_simulation_intervals(); ++t)
    {
        auto dp_no = this->get_demand_period_no(t);

        if (t % num == 0)
            std::cout << "simulation time = " << t / num << " min, CA = "
                      << cum_arr << ", CD = " << cum_dep << '\n';

        if (t)
        {
            for (auto& link_que : this->link_queues)
                link_que.update_cum_states(t);
        }

        for (auto a_no : this->get_agents_at_interval(t))
        {
            const auto& agent = this->get_agent(a_no);
            if (!agent.get_link_num())
                continue;

            auto& link_que = this->get_link_queue(agent.get_last_link_no());
            link_que.increment_cum_arr(t);
            link_que.append_entr_queue(a_no);
            ++cum_arr;
        }

        for (auto& link_que : this->link_queues)
        {
            while (!link_que.is_entr_queue_empty())
            {
                auto a_no = link_que.get_entr_queue_front();
                auto& agent = this->get_agent(a_no);
                auto tt = std::ceil(link_que.get_period_fftt(dp_no) * num);

                link_que.append_exit_queue(a_no);
                agent.update_dep_interval(tt);
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
                        auto next_link_no = agent.get_next_link_no();
                        auto& next_link_que = this->get_link_queue(next_link_no);

                        next_link_que.append_entr_queue(a_no);
                        agent.set_dep_interval(t);
                        agent.set_arr_interval(t);

                        auto travel_time = t - agent.get_arr_interval();
                        auto waiting_time = travel_time - link_que.get_period_fftt(dp_no);
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