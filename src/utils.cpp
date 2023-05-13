/**
 * @file utils.cpp, part of the project TransOMS under Apache License 2.0
 * @author jdlph (jdlph@hotmail.com) and xzhou99 (xzhou74@asu.edu)
 * @brief Implementations of utilities
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 */

#ifdef _WIN32
#define YAML_CPP_STATIC_DEFINE
#endif

#include <handles.h>
#include <stdcsv.h>

#ifdef __cpp_lib_filesystem
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

// #include <future>
// #include <memory>

#include <yaml-cpp/yaml.h>

using namespace transoms;

#ifdef __cpp_lib_filesystem
using namespace std::filesystem;
#else
using namespace std::experimental::filesystem;
#endif

/**
 * @brief a helper struct to store the positions of headers related to VDFPeriod in link.csv
 *
 * @details it is to facilitate the operation on VDFPeriod in read_links()
 */
struct HeaderPos {
    HeaderPos() : alpha_pos {-1}, beta_pos {-1}, cap_pos {-1}, fftt_pos {-1}
    {
    }

    int alpha_pos;
    int beta_pos;
    int cap_pos;
    int fftt_pos;
};

void NetworkHandle::read_nodes(const std::string& dir, const std::string& filename)
{
    auto reader = miocsv::DictReader(dir + '/' + filename);

    size_type node_no = 0;
    for (const auto& line : reader)
    {
        std::string node_id;
        try
        {
            node_id = line["node_id"];
        }
        catch(const std::exception& e)
        {
            continue;
        }

        std::string zone_id;
        try
        {
            zone_id = line["zone_id"];
        }
        catch(const std::exception& e)
        {
            continue;
        }

        double cx = COORD_X;
        double cy = COORD_Y;
        try
        {
            cx = std::stod(line["x_coord"]);
            cy = std::stod(line["y_coord"]);
        }
        catch(const std::exception& e)
        {
            // do nothing
        }

        bool activity_node = false;
        try
        {
            auto b = std::stoi(line["is_boundary"]);
            if (b)
                activity_node = true;
        }
        catch(const std::exception& e)
        {
            // do nothing
        }

        auto node = new Node {node_no, node_id, cx, cy, activity_node};
        this->net.add_node(node);

        unsigned short bin_index = 0;
        try
        {
            bin_index = std::stoi(line["bin_index"]);
        }
        catch(const std::exception& e)
        {
            // do nothing
        }

        // skip zone with empty id
        if (!zone_id.empty())
        {
            if (this->net.get_zones().find(zone_id) == this->net.get_zones().end())
            {
                size_type no = this->net.get_zones().size();
                this->net.add_zone(new Zone{no, zone_id, bin_index});
            }

            this->net.get_zones()[zone_id]->add_node(node_no);
            if (activity_node)
                this->net.get_zones()[zone_id]->add_activity_node(node_no);
        }

        ++node_no;
    }

    std::cout << "the number of nodes is " << node_no << '\n'
              << "the number of zones is " << this->net.get_zones().size() << '\n';

}

void NetworkHandle::read_links(const std::string& dir, const std::string& filename)
{
    auto reader = miocsv::DictReader(dir + '/' + filename);

    const auto& headers = reader.get_fieldnames();
    std::vector<HeaderPos> vec;

    for (unsigned short i = 0; i != this->dps.size(); ++i)
    {
        auto dp_id = std::to_string(i + 1);

        auto header_vdf_alpha {"VDF_alpha" + dp_id};
        auto header_vdf_beta {"VDF_beta" + dp_id};
        auto header_vdf_cap {"VDF_cap" + dp_id};
        auto header_vdf_fftt {"VDF_fftt" + dp_id};

        HeaderPos hp;

        try
        {
            hp.alpha_pos = headers.at(header_vdf_alpha);
        }
        catch(const std::out_of_range& re)
        {
            // do nothing
        }

        try
        {
            hp.beta_pos = headers.at(header_vdf_beta);
        }
        catch(const std::out_of_range& re)
        {
            // do nothing
        }

        try
        {
            hp.cap_pos = headers.at(header_vdf_cap);
        }
        catch(const std::out_of_range& re)
        {
            // do nothing
        }

        try
        {
            hp.fftt_pos = headers.at(header_vdf_fftt);
        }
        catch(const std::out_of_range& re)
        {
            // do nothing
        }

        vec.emplace_back(hp);
    }

    size_type link_no = 0;
    for (const auto& line : reader)
    {
        std::string link_id;
        try
        {
            link_id = line["link_id"];
        }
        catch(const std::exception& e)
        {
            continue;
        }

        std::string head_node_id;
        try
        {
            head_node_id = line["from_node_id"];
        }
        catch(const std::exception& e)
        {
            continue;
        }

        std::string tail_node_id;
        try
        {
            tail_node_id = line["to_node_id"];
        }
        catch(const std::exception& e)
        {
            continue;
        }

        size_type head_node_no, tail_node_no;
        try
        {
            head_node_no = this->net.get_node_no(head_node_id);
            tail_node_no = this->net.get_node_no(tail_node_id);
        }
        catch(const std::exception& e)
        {
            continue;
        }

        double len;
        try
        {
            len = std::stod(line["length"]);
        }
        catch(const std::exception& e)
        {
            continue;
        }

        unsigned short lane_num = 1;
        try
        {
            lane_num = std::stoi(line["lanes"]);
        }
        catch(const std::exception& e)
        {
            // do nothing
        }

        // unsigned short lane_type = 1;
        // try
        // {
        //     lane_type = std::stoi(line["link_type"]);
        // }
        // catch(const std::exception& e)
        // {
        //     // do nothing
        // }

        double ffs = 60;
        try
        {
            ffs = std::stoi(line["free_speed"]);
        }
        catch(const std::exception& e)
        {
            // do nothing
        }

        double cap = 1999;
        try
        {
            cap = std::stoi(line["capacity"]);
        }
        catch(const std::exception& e)
        {
            // do nothing
        }

        std::string modes {ALL_MODES};
        try
        {
            modes = line["allowed_uses"];
        }
        catch(const std::exception& e)
        {
            // do nothing
        }

        std::string geo;
        try
        {
            geo = line["geometry"];
        }
        catch(const std::exception& e)
        {
            // do nothing
        }

        auto link = new Link {
            link_id, link_no, head_node_no, tail_node_no,
            lane_num, cap, ffs, len, modes, geo
        };

        unsigned short dp_no = 0;
        for (const auto& hp : vec)
        {
            double vdf_alpha = 0.15;
            try
            {
                vdf_alpha = std::stod(line[hp.alpha_pos]);
            }
            catch(const std::exception& e)
            {
                // do nothing
            }

            double vdf_beta = 4;
            try
            {
                vdf_beta = std::stod(line[hp.beta_pos]);
            }
            catch(const std::exception& e)
            {
                // do nothing
            }

            double vdf_cap = link->get_cap();
            try
            {
                vdf_cap = std::stod(line[hp.cap_pos]);
            }
            catch(const std::exception& e)
            {
                // do nothing
            }

            double vdf_fftt = link->get_fftt();
            try
            {
                vdf_fftt = std::stod(line[hp.fftt_pos]);
            }
            catch(const std::exception& e)
            {
                // do nothing
            }

            link->add_vdfperiod(VDFPeriod{dp_no++, vdf_alpha, vdf_beta, vdf_cap, vdf_fftt});
        }

        this->net.add_link(link);
        ++link_no;
    }

    std::cout << "the number of links is " << link_no << '\n';
}

void NetworkHandle::read_demand(const std::string& dir, unsigned short dp_no, unsigned short at_no)
{
    auto reader = miocsv::DictReader(dir);

    double total_vol = 0;
    for (const auto& line : reader)
    {
        double vol = 0;
        try
        {
            vol = std::stod(line["volume"]);
        }
        catch(const std::exception& e)
        {
            continue;
        }

        if (vol <= 0)
            continue;

        std::string oz_id;
        try
        {
            oz_id = line["o_zone_id"];
        }
        catch(const std::exception& e)
        {
            continue;
        }

        std::string dz_id;
        try
        {
            dz_id = line["d_zone_id"];
        }
        catch(const std::exception& e)
        {
            continue;
        }

        size_type oz_no;
        size_type dz_no;
        try
        {
            oz_no = this->net.get_zone_no(oz_id);
            dz_no = this->net.get_zone_no(dz_id);
        }
        catch(const std::exception& e)
        {
            continue;
        }

        this->cp.update(ColumnVecKey{oz_no, dz_no, dp_no, at_no}, vol);

        total_vol += vol;
    }

    std::cout << "the total demand is " << total_vol << '\n';
}

void NetworkHandle::read_network(const std::string& dir)
{
    read_nodes(dir);
    read_links(dir);
}

void NetworkHandle::read_settings_yml(const std::string& file_path)
{
    YAML::Node settings = YAML::LoadFile(file_path);

    unsigned short i = 0;
    const auto& agents = settings["agent_type"];
    for (const auto& a : agents)
    {
        try
        {
            // auto type_ = a["type"];
            auto name = a["name"].as<std::string>();
            if (this->contains_agent_name(name))
            {
                std::cout << "duplicate agent type found: " << name << '\n';
                continue;
            }

            auto flow_type = a["flow_type"].as<unsigned short>();
            auto pce = a["pce"].as<double>();
            auto vot = a["vot"].as<double>();
            auto ffs = a["free_speed"].as<double>();
            auto use_ffs = a["use_link_ffs"].as<bool>();

            const auto at = new AgentType{i++, name, flow_type, pce, vot, ffs, use_ffs};
            this->ats.push_back(at);
        }
        catch(const std::exception& e)
        {
            // do nothing
        }
    }

    // it is possible that no AgentType is set up
    if (this->ats.empty())
        this->ats.push_back(new AgentType());

    unsigned short j = 0;
    const auto& demand_periods = settings["demand_period"];
    for (const auto& dp : demand_periods)
    {
        unsigned short k = 0;
        auto period = dp["period"].as<std::string>();
        auto time_period = dp["time_period"].as<std::string>();

        const auto& demands = dp["demand"];
        for (const auto& d : demands)
        {
            auto file_name = d["file_name"].as<std::string>();
            auto at_name = d["agent_type"].as<std::string>();
            try
            {
                const auto at = this->get_agent_type(at_name);
                // special event
                std::unique_ptr<SpecialEvent> se = nullptr;
                try
                {
                    const auto& special_event = dp["special_event"];
                    auto enable = special_event["enable"].as<bool>();
                    if (enable)
                    {
                        auto name = special_event["name"].as<std::string>();
                        se = std::make_unique<SpecialEvent>(name);

                        const auto& affected_links = special_event["affected_link"];
                        for (const auto& link : affected_links)
                        {
                            auto link_id = link["link_id"].as<std::string>();
                            auto rr = link["capacity_ratio"].as<double>();
                            se->add_affected_link(link_id, rr);
                        }
                    }
                }
                catch(const std::exception& e)
                {
                    // do nothing
                }

                const auto dp_ = new DemandPeriod{
                    j++, period, time_period, Demand{k++, file_name, at}, se
                };

                this->dps.push_back(dp_);
            }
            catch(const std::exception& e)
            {
                std::cout << at_name << " is not existing in settings.yml\n";
                continue;
            }
        }
    }

    if (this->dps.empty())
    {
        const auto at = this->ats.front();
        this->dps.push_back(new DemandPeriod{Demand{at}});
    }

    // not in use as the simulation module is not implemented yet!
    const YAML::Node& simulation = settings["simulation"];
    auto period = simulation["period"].as<std::string>();
    auto res = simulation["resolution"].as<unsigned short>();
}

void NetworkHandle::auto_setup()
{
    const auto at = new AgentType();
    const auto dp = new DemandPeriod{Demand {at}};

    this->ats.push_back(at);
    this->dps.push_back(dp);
}

void NetworkHandle::read_settings(const std::string& dir)
{
    path file_path = dir + '/' + "settings.yml";
    if (exists(file_path))
        read_settings_yml(file_path.string());
    else
        auto_setup();
}

void NetworkHandle::read_demands(const std::string& dir)
{
    for (auto& dp : this->dps)
    {
        auto dp_no = dp->get_no();
        for (auto& d : dp->get_demands())
        {
            auto at_no = d.get_agent_type_no();
            auto file_path = dir + d.get_file_name();
            read_demand(file_path, dp_no, at_no);
        }

        // set up capacity ratio of affected links from special event
        const auto& se = dp->get_special_event();
        if (!se)
            continue;

        for (const auto& [link_id, r] : se->get_capacity_ratios())
        {
            // to do: wrap them into a single function?
            try
            {
                auto link = this->get_link(link_id);
                link->set_cap_ratio(dp_no, r);
            }
            catch (std::out_of_range& re)
            {
                continue;
            }
        }
    }
}

std::string NetworkHandle::get_link_path_str(const Column& c)
{
    std::string str;
    for (auto j = c.get_link_num() - 1; j != 0; --j)
    {
        const auto link = this->get_link(c.get_link_no(j));
        str += link->get_id();
        str += ';';
    }

    const auto link = this->get_link(c.get_link_no(0));
    str += link->get_id();

    // it will be moved
    return str;
}

std::string NetworkHandle::get_node_path_str(const Column& c)
{
    std::string str;
    for (int j = c.get_link_num() - 1; j >= 0; --j)
    {
        const auto link = this->get_link(c.get_link_no(j));
        str += this->get_head_node_id(link);
        str += ';';
    }

    const auto link = this->get_link(c.get_link_no(0));
    str += this->get_tail_node_id(link);

    // it will be moved
    return str;
}

std::string NetworkHandle::get_node_path_coordinates(const Column& c)
{
    std::string str {"\"LINESTRING ("};
    for (int j = c.get_link_num() - 1; j >= 0; --j)
    {
        auto node_no = this->get_link(c.get_link_no(j))->get_head_node_no();
        const auto node = this->get_node(node_no);
        str += node->get_coordinate_str();
        str += ';';
    }

    auto node_no = this->get_link(c.get_link_no(0))->get_tail_node_no();
    const auto node = this->get_node(node_no);
    str += node->get_coordinate_str();
    str += ")\"";

    // it will be moved
    return str;
}

void NetworkHandle::output_columns(const std::string& dir, const std::string& filename)
{
    auto writer = miocsv::Writer(dir + '/' + filename);

    writer.write_row_raw("agent_id", "o_zone_id", "d_zone_id", "path_id", "agent_type",
                         "demand_period", "volume", "toll", "travel_time", "distance",
                         "link_sequence", "node_sequence", "geometry");

    size_type i = 0;
    for (const auto& cv : cp.get_column_vecs())
    {
        // oz_no, dz_no, dp_no, at_no
        auto oz_no = std::get<0>(cv.get_key());
        auto dz_no = std::get<1>(cv.get_key());
        auto dp_no = std::get<2>(cv.get_key());
        auto at_no = std::get<3>(cv.get_key());

        auto dp_str = dps[dp_no]->get_period();
        auto at_str = ats[at_no]->get_name();

        for (const auto& col : cv.get_columns())
        {
            writer.append(++i);
            writer.append(this->get_zone_id(oz_no));
            writer.append(this->get_zone_id(dz_no));
            writer.append(col.get_no());
            writer.append(at_str);
            writer.append(dp_str);
            writer.append(col.get_volume());
            writer.append(col.get_toll());
            writer.append(col.get_travel_time());
            writer.append(col.get_dist());

            auto link_path = this->get_link_path_str(col);
            auto node_path = this->get_node_path_str(col);
            auto geo = this->get_node_path_coordinates(col);

            // the followings are not working yet!!
            // auto link_path = std::async(&NetworkHandle::get_link_path_str, this, col);
            // auto node_path = std::async(&NetworkHandle::get_node_path_str, this, col);
            // auto geo = std::async(&NetworkHandle::get_node_path_coordinates, this, col);

            writer.append(link_path);
            writer.append(node_path);
            writer.append(geo, '\n');
        }
    }

    std::cout << "check " << filename << " in " << dir <<  " for UE results\n";
}

void NetworkHandle::output_link_performance(const std::string& dir, const std::string& filename)
{
    auto writer = miocsv::Writer(dir + '/' + filename);

    writer.write_row_raw("link_id", "from_node_id", "to_node_id", "time_period", "volume",
                         "travel_time", "speed", "VOC", "queue", "density", "geometry");

    for (const auto link : this->net.get_links())
    {
        if (!link->get_length())
            continue;

        for (const auto& dp : this->dps)
        {
            auto dp_no = dp->get_no();
            auto tt = link->get_period_travel_time(dp_no);
            auto spd = tt > 0 ? link->get_length() / tt * MINUTES_IN_HOUR : std::numeric_limits<unsigned>::max();

            writer.write_row_raw(link->get_id(), this->get_head_node_id(link), this->get_tail_node_id(link),
                                 dp->get_period(), link->get_period_vol(dp_no), tt, spd, link->get_period_voc(dp_no),
                                 ' ', ' ', link->get_geometry());
        }
    }

    std::cout << "check " << filename << " in " << dir <<  " for link performance\n";
}