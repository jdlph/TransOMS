/**
 * @file utils.cpp, part of the project openDTA under GPL-3.0 license
 * @author jdlph (jdlph@hotmail.com) and xzhou99 (xzhou74@asu.edu)
 * @brief Implementations of utilities
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 *
 */

#include <handles.h>
#include <stdcsv.h>

#include <future>

using namespace opendta;

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

        auto* node = new Node {node_no, std::move(node_id), cx, cy, activity_node};
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
                auto* zone = new Zone {no, zone_id, bin_index};
                this->net.add_zone(zone);
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

        double len;
        try
        {
            len = std::stod(line["length"]);
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

        unsigned short lane_num = 1;
        try
        {
            lane_num = std::stoi(line["lanes"]);
        }
        catch(const std::exception& e)
        {
            // do nothing
        }

        unsigned short lane_type = 1;
        try
        {
            lane_type = std::stoi(line["link_type"]);
        }
        catch(const std::exception& e)
        {
            // do nothing
        }

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

        auto* link = new Link {std::move(link_id), link_no,
                               std::move(head_node_id), head_node_no,
                               std::move(tail_node_id), tail_node_no,
                               lane_num, cap, ffs, len,
                               std::move(modes), std::move(geo)};

        for (unsigned short i = 0; i != this->dps.size(); ++i)
        {
            auto dp_id = std::to_string(i + 1);

            auto header_vdf_alpha = "VDF_alpha" + dp_id;
            auto header_vdf_beta = "VDF_beta" + dp_id;
            auto header_vdf_mu = "VDF_mu" + dp_id;
            auto header_vdf_fftt = "VDF_fftt" + dp_id;
            auto header_vdf_cap = "VDF_cap" + dp_id;
            auto header_vdf_phf = "VDF_phf" + dp_id;

            double vdf_alpha = 0.15;
            try
            {
                vdf_alpha = std::stod(line[header_vdf_alpha]);
            }
            catch(const std::exception& e)
            {
                if (i)
                    break;
            }

            double vdf_beta = 4;
            try
            {
                vdf_beta = std::stod(line[header_vdf_beta]);
            }
            catch(const std::exception& e)
            {
                if (i)
                    break;
            }

            double vdf_mu = 1000;
            try
            {
                vdf_mu = std::stod(line[header_vdf_mu]);
            }
            catch(const std::exception& e)
            {
                if (i)
                    break;
            }

            double vdf_fftt = link->get_fftt();
            try
            {
                vdf_fftt = std::stod(line[header_vdf_fftt]);
            }
            catch(const std::exception& e)
            {
                // do nothing
            }

            double vdf_cap = link->get_cap();
            try
            {
                vdf_cap = std::stod(line[header_vdf_cap]);
            }
            catch(const std::exception& e)
            {
                // do nothing
            }

            double vdf_phf = -1;
            try
            {
                vdf_phf = std::stod(line[header_vdf_phf]);
            }
            catch(const std::exception& e)
            {
                // do nothing
            }

            link->add_vdfperiod(VDFPeriod {i, vdf_alpha, vdf_beta, vdf_mu, vdf_cap, vdf_fftt});
        }

        this->net.add_link(link);
        ++link_no;
    }

    std::cout << "the number of links is " << link_no << '\n';
}

void NetworkHandle::read_demand(const std::string& dir, unsigned short dp_no, unsigned short at_no)
{
    auto reader = miocsv::DictReader(dir);

    size_type num = 0;
    for (const auto& line : reader)
    {
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

        if (!this->net.contains_zone(oz_id))
            continue;

        if (!this->net.contains_zone(dz_id))
            continue;

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

        auto oz_no = this->net.get_zone_no(oz_id);
        auto dz_no = this->net.get_zone_no(dz_id);

        ColumnVecKey cvk {oz_no, dz_no, dp_no, at_no};
        this->cp.update(cvk, vol);

        num += std::ceil(vol);
    }

    std::cout << "the number of agents is " << num << '\n';
}

void NetworkHandle::read_network(const std::string& dir)
{
    read_settings(dir);
    read_nodes(dir);
    read_links(dir);
}

void NetworkHandle::auto_setup()
{
    const auto* at = new AgentType();
    DemandPeriod dp {Demand {at}};

    this->ats.push_back(at);
    this->dps.push_back(dp);
}

void NetworkHandle::read_settings(const std::string& dir)
{
    auto_setup();
}

void NetworkHandle::read_demands(const std::string& dir)
{
    for (auto& dp : this->dps)
    {
        auto dp_no = dp.get_no();
        for (auto& d : dp.get_demands())
        {
            auto at_no = d.get_agent_type_no();
            auto file_path = dir + d.get_file_name();
            read_demand(file_path, dp_no, at_no);
        }
    }
}

void NetworkHandle::output_columns(const std::string& dir, const std::string& filename)
{
    auto writer = miocsv::Writer(dir + '/' + filename);

    writer.write_row({"agent_id", "o_zone_id", "d_zone_id", "path_id", "agent_type",
                      "demand_period","volume", "toll", "travel_time", "distance",
                      "link_sequence", "node_sequence", "geometry"});

    size_type i = 0;
    for (auto& [k, cv] : cp.get_column_vecs())
    {
        // oz_id, dz_no, dp_no, at_no
        auto oz_id = std::get<0>(k);
        auto dz_id = std::get<1>(k);
        auto dp_no = std::get<2>(k);
        auto at_no = std::get<3>(k);

        auto dp_str = dps[dp_no].get_period();
        auto at_str = ats[at_no]->get_name();

        for (const auto& col : cv.get_columns())
        {
            writer.append(++i);
            writer.append(oz_id);
            writer.append(dz_id);
            writer.append(col.get_no());
            writer.append(at_str);
            writer.append(dp_str);
            writer.append(col.get_volume());
            writer.append(col.get_toll());
            writer.append(col.get_travel_time());
            writer.append(col.get_dist());

            for (auto j = col.get_link_num() - 1; j != 0; --j)
            {
                const auto& link = this->get_link(col.get_link_no(j));
                writer.append(link.get_id(), ";");
            }
            const auto& link = this->get_link(col.get_link_no(0));
            writer.append(link.get_id(), ",");

            for (auto j = col.get_node_num() - 1; j != 0; --j)
            {
                const auto& node = this->get_node(col.get_node_no(j));
                writer.append(node.get_id(), ";");
            }
            const auto& node = this->get_node(col.get_node_no(0));
            writer.append(node.get_id(), ",");

            writer.append("\"LINESTRING (", "", false);
            for (auto j = col.get_node_num() - 1; j != 0; --j)
            {
                const auto& node_ = this->get_node(col.get_node_no(j));
                writer.append(node_.get_coordinate_str(), ", ");
            }
            const auto& node_ = this->get_node(col.get_node_no(0));
            writer.append(node_.get_coordinate_str(), ")\"", true);
        }
    }

    std::cout << "check " << filename << " in " << dir <<  " for UE results\n";
}

void NetworkHandle::output_link_performance(const std::string& dir, const std::string& filename)
{
    auto writer = miocsv::Writer(dir + '/' + filename);

    writer.write_row({"link_id", "from_node_id", "to_node_id", "time_period", "volume",
                      "travel_time", "speed", "VOC", "queue", "density", "geometry"});

    for (const auto link : this->net.get_links())
    {
        if (!link->get_length())
            continue;

        for (const auto& dp : this->dps)
        {
            auto dp_no = dp.get_no();
            auto tt = link->get_period_travel_time(dp_no);
            auto spd = tt > 0 ? link->get_length() / tt * MINUTES_IN_HOUR : INT_MAX;

            writer.write_row({link->get_id(), link->get_head_node_id(), link->get_tail_node_id(),
                              dp.get_period(), link->get_period_vol(dp_no), tt, spd, link->get_period_voc(dp_no),
                              ' ', ' ', link->get_geometry()});
        }
    }

    std::cout << "check " << filename << " in " << dir <<  " for link performance\n";
}

void NetworkHandle::output_columns_par(const std::string& dir, const std::string& filename)
{
    auto writer = miocsv::Writer(dir + '/' + filename);

    writer.write_row({"agent_id", "o_zone_id", "d_zone_id", "path_id", "agent_type",
                      "demand_period","volume", "toll", "travel_time", "distance",
                      "link_sequence", "node_sequence", "geometry"});

    size_type i = 0;
    for (auto& [k, cv] : cp.get_column_vecs())
    {
        // oz_id, dz_no, dp_no, at_no
        auto oz_id = std::get<0>(k);
        auto dz_id = std::get<1>(k);
        auto dp_no = std::get<2>(k);
        auto at_no = std::get<3>(k);

        auto dp_str = dps[dp_no].get_period();
        auto at_str = ats[at_no]->get_name();

        for (const auto& col : cv.get_columns())
        {
            writer.append(++i);
            writer.append(oz_id);
            writer.append(dz_id);
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
            writer.append(geo, "\n");
        }
    }

    std::cout << "check " << filename << " in " << dir <<  " for UE results\n";
}

std::string NetworkHandle::get_link_path_str(const Column& c)
{
    std::string str;
    for (auto j = c.get_link_num() - 1; j != 0; --j)
    {
        const auto& link = this->get_link(c.get_link_no(j));
        str += link.get_id();
        str += ';';
    }

    const auto& link = this->get_link(c.get_link_no(0));
    str += link.get_id();

    // it will be moved
    return str;
}

std::string NetworkHandle::get_node_path_str(const Column& c)
{
    std::string str;
    for (auto j = c.get_node_num() - 1; j != 0; --j)
    {
        const auto& node = this->get_node(c.get_node_no(j));
        str += node.get_id();
        str += ';';
    }

    const auto& node = this->get_node(c.get_node_no(0));
    str += node.get_id();

    // it will be moved
    return str;
}

std::string NetworkHandle::get_node_path_coordinates(const Column& c)
{
    std::string str = {"\"LINESTRING ("};
    for (auto j = c.get_node_num() - 1; j != 0; --j)
    {
        const auto& node = this->get_node(c.get_node_no(j));
        str += node.get_coordinate_str();
        str += ';';
    }

    const auto& node = this->get_node(c.get_node_no(0));
    str += node.get_coordinate_str();
    str += ")\"";

    // it will be moved
    return str;
}