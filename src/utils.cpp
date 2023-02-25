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