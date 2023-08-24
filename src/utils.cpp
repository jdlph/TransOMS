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

#include <iostream>
#include <iomanip>

#include <yaml-cpp/yaml.h>

using namespace transoms;
using namespace std::string_literals;

#ifdef __cpp_lib_filesystem
using namespace std::filesystem;
#else
using namespace std::experimental::filesystem;
#endif

/**
 * @brief a helper struct to store the positions of headers related to VDFPeriod in link.csv
 *
 * @details it is to facilitate the operation on VDFPeriod in read_links()
 *
 * @note short is sufficient enough as we would not have a csv file with more than
 * 32,767 fields for this application
 */
struct HeaderPos {
    HeaderPos() : alpha_pos {-1}, beta_pos {-1}, cap_pos {-1}, fftt_pos {-1}
    {
    }

    short alpha_pos;
    short beta_pos;
    short cap_pos;
    short fftt_pos;
};

void NetworkHandle::auto_setup()
{
    const auto at = new AgentType();
    const auto dp = new DemandPeriod{Demand{at}};

    this->ats.push_back(at);
    this->dps.push_back(dp);
}

void NetworkHandle::to_lower(std::string& str)
{
    std::transform(str.cbegin(), str.cend(), str.begin(),
                   [](unsigned char c) {return std::tolower(c);});
}

void NetworkHandle::update_simulation_settings(unsigned short res, const std::string& model)
{
    this->simu_res = res;

    if (model == "spatial_queue"s || model == "spatial queue"s)
        this->tfm = TrafficFlowModel::spatial_queue;
    else if (model == "kinematic_wave"s || model == "kinematic wave"s)
        this->tfm = TrafficFlowModel::kinematic_wave;

    // set up simulation duration
    auto st = this->dps.front()->get_start_time();
    auto et = this->dps.back()->get_end_time();

    this->simu_dur = et - st;
}

void NetworkHandle::validate_demand_periods()
{
    if (this->dps.size() <= 1)
        return;

    // first, sort DemandPeriod instances according to the start time
    std::sort(this->dps.begin(), this->dps.end(),
              [](const DemandPeriod* left, const DemandPeriod* right){
                  return left->get_start_time() < right->get_start_time();
              });

    // second, check if there is overlap between two consecutive DemandPeriod instances
    auto curr_dp = this->dps.front();
    for (auto i = 1; i != this->dps.size(); ++i)
    {
        auto next_dp = this->dps[i];
        if (curr_dp->get_end_time() > next_dp->get_start_time())
        {
            std::string msg = {"Overlapping found between DemandPeriod "s
                               + curr_dp->get_time_period()
                               + " and DemandPeriod "s
                               + next_dp->get_time_period()};

            throw std::runtime_error{msg};
        }

        curr_dp = next_dp;
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

double NetworkHandle::cast_interval_to_minute_double(size_type i) const
{
    return static_cast<double>(i) * this->simu_res / SECONDS_IN_MINUTE;
}

std::string NetworkHandle::get_time_stamp(double t)
{
    static constexpr char sep = ':';

    unsigned short ti = std::floor(t);
    unsigned short hh = ti / MINUTES_IN_HOUR;
    unsigned short mm = ti % MINUTES_IN_HOUR;
    unsigned short ss = (t - ti) * SECONDS_IN_MINUTE;

    // convert time to hh::mm::ss format without leading 0 for hh
    // (e.g., 7AM will be displayed as 7:00:00 rather than 07:00:00)
    std::ostringstream os;
    os << hh << sep
       << std::setfill('0') << std::setw(2) << mm << sep << std::setw(2) << ss;

    return os.str();
}

void NetworkHandle::update_od_vol()
{
    for (auto& cv : this->cp.get_column_vecs())
    {
        auto vol = cv.get_volume();
        // col is const even without const identifier as a result of the underlying hashtable
        for (auto& col : cv.get_columns())
            const_cast<Column&>(col).set_od_vol(vol);
    }
}

void NetworkHandle::load_columns(const std::string& dir, const std::string& filename)
{
    auto reader = miocsv::DictReader(dir + '/' + filename);
    std::cout << "start loading columns from " << filename << '\n';

    size_type count = 0;
    for (const auto& line : reader)
    {
        std::string oz_id;
        try
        {
            oz_id = line["o_zone_id"];
        }
        catch(const std::exception& e)
        {
            // headers have no "o_zone_id"
            std::cerr << e.what() << '\n';
            std::terminate();
        }

        std::string dz_id;
        try
        {
            dz_id = line["d_zone_id"];
        }
        catch(const std::exception& e)
        {
            // headers have no "d_zone_id"
            std::cerr << e.what() << '\n';
            std::terminate();
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

        std::string link_seq;
        try
        {
            link_seq = line["link_sequence"];
        }
        catch(std::exception& e)
        {
            // headers have no "link_sequence"
            std::cerr << e.what() << '\n';
            std::terminate();
        }

        if (link_seq.empty())
            continue;

        std::string at_str;
        try
        {
            at_str = line["agent_type"];
        }
        catch(const std::exception& e)
        {
            // headers have no "agent_type"
            std::cerr << e.what() << '\n';
            std::terminate();
        }

        if (at_str.empty())
            continue;

        const AgentType* at = nullptr;
        try
        {
            at = this->get_agent_type(at_str);
        }
        catch(const std::exception& e)
        {
            try
            {
                // add compatiblity for Path4GMNS and DTALite
                if (at_str.front() == 'a' || at_str.front() == 'p')
                    at = this->get_agent_type("auto");
                else
                {
                    std::cerr << "agent_type " << at_str
                              << "is not existing in settings.yml."
                              << "this record is discarded!\n";

                    continue;
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << "default agent type auto is not existing\n";
                std::terminate();
            }
        }

        std::string dp_str;
        try
        {
            dp_str = line["demand_period"];
        }
        catch(const std::exception& e)
        {
            // headers have no "demand_period"
            std::cerr << e.what() << '\n';
            std::terminate();
        }

        if (dp_str.empty())
            continue;

        const DemandPeriod* dp = nullptr;
        try
        {
            dp = this->get_demand_period(dp_str);
        }
        catch(const std::exception& e)
        {
            std::cerr << "demand period " << dp_str
                      << "is not existing in settings.yml."
                      << "this record is discarded!\n";

            continue;
        }

        double vol;
        try
        {
            vol = std::stod(line["volume"]);
        }
        catch(const miocsv::NoRecord& nr)
        {
            // headers have no "demand_period"
            std::cerr << nr.what() << '\n';
            std::terminate();
        }
        catch(const std::exception& e)
        {
            // e could be either std::invalid_argument or std::out_of_range
            // skip this invalid record as we do require a valid volume
            std::cerr << e.what() << '\n';
            continue;
        }

        double toll = 0;
        try
        {
            toll = std::stod(line["toll"]);
        }
        catch(const std::exception& e)
        {
            // do nothing as toll is not critical
        }

        double dist;
        try
        {
            dist = std::stod(line["distance"]);
        }
        catch(const miocsv::NoRecord& nr)
        {
            // headers have no "distance"
            std::cerr << nr.what() << '\n';
            std::terminate();
        }
        catch(const std::exception& e)
        {
            // e could be either std::invalid_argument or std::out_of_range
            // skip this invalid record as we do require a valid distance
            continue;
        }

        std::string geo;
        try
        {
            geo = line["geometry"];
        }
        catch(const std::exception& e)
        {
            // do nothing as geo info is not critical
        }

        ColumnVecKey cvk {oz_no, dz_no, dp->get_no(), at->get_no()};
        this->cp.update(cvk, vol);

        const auto link_ids = miocsv::split(link_seq, ';');
        /**
         * 1. in case the input file is generated by DTALite which has trailing ';'.
         * 2. use int intentionally. otherwise, num will be an unsigned type which will
         * lead to problem in the link_path setup below (i will be inferred as an
         * unsigned type too).
         */
        const int num = link_ids.back() == "" ? link_ids.size() - 1: link_ids.size();

        // set up link_path
        std::vector<size_type> link_path;
        link_path.reserve(num);
        // link_path shall be in the reverse order for internal computation
        for (auto i = num - 1; i >= 0; --i)
            link_path.push_back(this->get_link(link_ids[i])->get_no());

        // create column and update column vector
        auto& cv = this->cp.get_column_vec(cvk);
        cv.update(Column{cv.get_column_num(), vol, dist, link_path, geo});

        if (count % 5000 == 0)
            std::cout << "loading columns: " << count << '\n';

        ++count;
    }

    // update OD Volume for each column
    this->update_od_vol();

    // update link properties
    this->update_link_and_column_volume(1, false);
    this->update_link_travel_time();

    std::cout << "loading columns completed. " << count << " columns are loaded.\n";
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
        catch(const miocsv::NoRecord& nr)
        {
            // headers have no "volume"
            std::cerr << nr.what() << '\n';
            std::terminate();
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
            // headers have no "o_zone_id"
            std::cerr << e.what() << '\n';
            std::terminate();
        }

        std::string dz_id;
        try
        {
            dz_id = line["d_zone_id"];
        }
        catch(const std::exception& e)
        {
            // headers have no "d_zone_id"
            std::cerr << e.what() << '\n';
            std::terminate();
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

void NetworkHandle::read_demands(const std::string& dir)
{
    for (const auto& dp : this->dps)
    {
        auto dp_no = dp->get_no();
        for (auto& d : dp->get_demands())
        {
            auto at_no = d.get_agent_type_no();
            auto file_path = dir + d.get_file_name();
            this->read_demand(file_path, dp_no, at_no);
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
            // headers have no "node_id"
            std::cerr << e.what() << '\n';
            std::terminate();
        }

        std::string zone_id;
        try
        {
            zone_id = line["zone_id"];
        }
        catch(const std::exception& e)
        {
            // headers have no "zone_id"
            std::cerr << e.what() << '\n';
            std::terminate();
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
            if (std::stoi(line["is_boundary"]))
                activity_node = true;
        }
        catch(const std::exception& e)
        {
            // do nothing
        }

        this->net.add_node(new Node{node_no, node_id, cx, cy, activity_node});

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
            if (!this->has_zone_id(zone_id))
            {
                size_type no = this->get_zone_num();
                this->net.add_zone(new Zone{no, zone_id, bin_index});
            }

            this->get_zone(zone_id)->add_node(node_no);
            if (activity_node)
                this->get_zone(zone_id)->add_activity_node(node_no);
        }

        ++node_no;
    }

    std::cout << "the number of nodes is " << node_no << '\n'
              << "the number of zones is " << this->get_zone_num() << '\n';

}

void NetworkHandle::read_links(const std::string& dir, const std::string& filename)
{
    auto reader = miocsv::DictReader(dir + '/' + filename);
    const auto& headers = reader.get_fieldnames();

    std::vector<HeaderPos> vec;
    for (unsigned short i = 0; i != this->dps.size();)
    {
        auto dp_id = std::to_string(++i);

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
            // headers have no "link_id"
            std::cerr << e.what() << '\n';
            std::terminate();
        }

        std::string head_node_id;
        try
        {
            head_node_id = line["from_node_id"];
        }
        catch(const std::exception& e)
        {
            // headers have no "from_node_id"
            std::cerr << e.what() << '\n';
            std::terminate();
        }

        std::string tail_node_id;
        try
        {
            tail_node_id = line["to_node_id"];
        }
        catch(const std::exception& e)
        {
            // headers have no "to_node_id"
            std::cerr << e.what() << '\n';
            std::terminate();
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
        catch(const miocsv::NoRecord& nr)
        {
            // headers have no "length"
            std::cerr << nr.what() << '\n';
            std::terminate();
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
                std::cerr << "duplicate agent type found: " << name << '\n';
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
                std::cerr << at_name << " is not existing in settings.yml\n";
                continue;
            }
        }
    }

    try
    {
        this->validate_demand_periods();
    }
    catch (const std::runtime_error& re)
    {
        std::cerr << re.what() << '\n';
    }

    if (this->dps.empty())
    {
        const auto at = this->ats.front();
        this->dps.push_back(new DemandPeriod{Demand{at}});
    }

    // not in use as the simulation module is not implemented yet!
    try
    {
        const YAML::Node& simulation = settings["simulation"];
        auto res = simulation["resolution"].as<unsigned short>();
        auto model = simulation["traffic_flow_model"].as<std::string>();

        this->to_lower(model);
        this->update_simulation_settings(res, model);
    }
    catch (const std::exception& e)
    {
        // do nothing and set up simulation with default settings
    }
}

void NetworkHandle::read_settings(const std::string& dir)
{
    path file_path = dir + '/' + "settings.yml";
    if (exists(file_path))
        this->read_settings_yml(file_path.string());
    else
        this->auto_setup();
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
            if (!col.get_volume())
                continue;

            writer.append(i++);
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

void NetworkHandle::output_link_performance_dta(const std::string& dir, const std::string& filename)
{
    auto writer = miocsv::Writer(dir + '/' + filename);

    writer.write_row_raw("link_id", "from_node_id", "to_node_id", "time_period", "volume",
                         "travel_time",  "waiting_time", "speed", "CA", "CD", "density", "queue");

    // number of simulation intervals in one minute
    const unsigned short num = this->cast_minute_to_interval(1);
    unsigned short dp_no = 0;
    size_type ub = this->get_end_simulation_interval(dp_no);

    for (const auto& link_que : this->link_queues)
    {
        const auto link = link_que.get_link();

        if (!link->get_length())
            continue;

        for (size_type t = 0, e = this->get_simulation_intervals(); t != e; ++t)
        {
            if (t % num != 0)
                continue;

            if (t >= ub)
            {
                // restrict dp_no as we allow user to add buffer time to simulation
                // in addition to the given demand periods
                if (dp_no < this->dps.size() - 1)
                    ub = this->get_end_simulation_interval(++dp_no);
                else
                    ub = this->get_simulation_intervals();
            }

            auto minute = this->cast_interval_to_minute(t);

            writer.append(link->get_id());
            writer.append(this->get_head_node_id(link));
            writer.append(this->get_tail_node_id(link));
            writer.append(this->dps[dp_no]->get_period());
            writer.append(link_que.get_volume(t));
            writer.append(link_que.get_travel_time(t, dp_no));
            writer.append(link_que.get_avg_waiting_time(t));
            writer.append(link_que.get_speed(t, dp_no));
            writer.append(link_que.get_cumulative_arrival(t));
            writer.append(link_que.get_cumulative_departure(t));
            writer.append(link_que.get_density(t));
            writer.append(link_que.get_queue(t, dp_no));
        }
    }

    std::cout << "check " << filename << " in " << dir <<  " for link performance\n";
}

void NetworkHandle::output_trajectories(const std::string& dir, const std::string& filename)
{
    auto writer = miocsv::Writer(dir + '/' + filename);

    writer.write_row_raw("agent_id", "o_zone_id", "d_zone_id", "dep_time", "arr_time", "trip_completed",
                         "travel_time", "PCE", "travel_distance", "node_path", "geometry", "time_sequence");

    double dt = -1;
    auto od = this->get_agent(0).get_od();
    for (const auto& agent : this->agents)
    {
        if (agent.get_orig_dep_time() == dt && agent.get_od() == od)
            continue;

        dt = agent.get_orig_dep_time();
        od = agent.get_od();

        auto at = this->get_real_time(agent.get_dest_arr_interval());
        char trip_status = agent.completes_trip() ? 'c' : 'n';

        std::string time_seq_str;
        // move assignment
        const auto vec = agent.get_time_sequence();
        for (size_type i = 0, e = vec.size() - 1; i != e; ++i)
        {
            auto t = this->get_real_time(vec[i]);
            time_seq_str += this->get_time_stamp(t);
            time_seq_str += ';';
        }
        // the last one without trailing ';'
        auto t = this->get_real_time(vec.back());
        time_seq_str += this->get_time_stamp(t);

        const auto& col = *agent.get_column();

        writer.append(agent.get_no());
        writer.append(agent.get_orig_zone_no());
        writer.append(agent.get_dest_zone_no());
        writer.append(this->get_time_stamp(dt));
        writer.append(this->get_time_stamp(at));
        writer.append(trip_status);
        writer.append(this->cast_interval_to_minute_double(agent.get_travel_interval()));
        writer.append(agent.get_pce());
        writer.append(col.get_dist());
        writer.append(this->get_node_path_str(col));
        writer.append(this->get_node_path_coordinates(col));
        writer.append(time_seq_str, '\n');
    }
}