#include <handles.h>
#include <stdcsv.h>

using namespace opendta;

void NetworkHandle::read_nodes(const std::string& dir)
{
    auto reader = miocsv::DictReader(dir);

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

        double cx = 91;
        double cy = 181;
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

        auto* node = new Node {node_no, std::move(node_id), cx, cy, zone_id, activity_node};
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

        if (this->net.get_zones().find(zone_id) == this->net.get_zones().end())
        {
            size_type no = this->net.get_zones().size();
            auto* zone = new Zone {no, zone_id, bin_index};
            this->net.add_zone(zone);
        }

        this->net.get_zones()[zone_id]->add_node(node_no);
        if (activity_node)
            this->net.get_zones()[zone_id]->add_activity_node(node_no++);
    }
}

void NetworkHandle::read_links(const std::string& dir)
{
    auto reader = miocsv::DictReader(dir);

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

        double cap = 60;
        try
        {
            cap = std::stoi(line["capacity"]);
        }
        catch(const std::exception& e)
        {
            // do nothing
        }

        std::string modes {"all"};
        try
        {
            modes = line["allowed_uses"];
        }
        catch(const std::exception& e)
        {
            continue;
        }

        std::string geo;
        try
        {
            geo = line["geometry"];
        }
        catch(const std::exception& e)
        {
            continue;
        }

        auto* link = new Link {std::move(link_id), link_no,
                               std::move(head_node_id), head_node_no,
                               std::move(tail_node_id), tail_node_no,
                               lane_num, cap, ffs, len, std::move(modes), std::move(geo)};

        for (auto i = 0; i != this->dps.size(); ++i)
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
                vdf_fftt = std::stod(line[header_vdf_cap]);
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

            VDFPeriod vdf {i, vdf_alpha, vdf_beta, vdf_mu, vdf_cap, vdf_fftt};
            link->add_vdfperiod(vdf);

            auto head_node = this->net.get_nodes()[head_node_no];
            auto tail_node = this->net.get_nodes()[tail_node_no];

            head_node->add_outgoing_link(link);
            tail_node->add_incoming_link(link);
            this->net.add_link(link);
        }
    }
}