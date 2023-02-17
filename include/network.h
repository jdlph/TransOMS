#ifndef GUARD_NETWORK_H
#define GUARD_NETWORK_H

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

// some constants
constexpr unsigned MINUTES_IN_HOUR = 60;


// forward declaration
class Link;

class Node {
public:
    Node() = default;

    Node(std::size_t no_, std::string&& id_,  double x_, double y_, std::string&& z_id, bool act_node_)
        : no {no_}, id {id_},  x {x_}, y {y_}, zone_id {z_id}, act_node {act_node_}
    {
    }

    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    Node(Node&&) = delete;
    Node& operator=(Node&&) = delete;

    ~Node() = default;

    const std::string& get_id() const
    {
        return id;
    }

    std::size_t get_no() const
    {
        return no;
    }

    const std::string& get_zone_id() const
    {
        return zone_id;
    }

    auto get_coordinate() const
    {
        return std::make_pair(x, y);
    }

    std::vector<Link*>::size_type incoming_link_num() const
    {
        return incoming_links.size();
    }

    std::vector<Link*>::size_type outgoing_link_num() const
    {
        return outgoing_links.size();
    }

    std::vector<Link*>& get_incoming_links()
    {
        return incoming_links;
    }

    const std::vector<Link*>& get_incoming_links() const
    {
        return incoming_links;
    }

    std::vector<Link*>& get_outgoing_links()
    {
        return outgoing_links;
    }

    const std::vector<Link*>& get_outgoing_links() const
    {
        return outgoing_links;
    }

    void add_incoming_link(Link* p)
    {
        incoming_links.push_back(p);
    }

    void add_outgoing_link(Link* p)
    {
        outgoing_links.push_back(p);
    }

    void setup_coordinate(double x_, double y_)
    {
        x = x_;
        y = y_;
    }

private:
    std::string id;
    std::size_t no;

    double x;
    double y;

    std::string zone_id;
    bool act_node;

    std::vector<Link*> incoming_links;
    std::vector<Link*> outgoing_links;
};

// forward declaration
class VDFPeriod;

class Link {
public:
    Link() = default;

    Link(std::string&& id_, std::size_t no_,
         std::string&& head_node_id_, std::size_t head_node_no_,
         std::string&& tail_node_id_, std::size_t tail_node_no_,
         unsigned lane_num_, double cap_, double ffs_, double len_, unsigned demand_period_num)
         : id {id_}, no {no_},
           head_node_id {head_node_id_}, head_node_no {head_node_no_},
           tail_node_id {tail_node_id_}, tail_node_no {tail_node_no},
           lane_num {lane_num_}, cap {cap_}, ffs {ffs_}, len {len_}
    {
        period_tt = new double[demand_period_num];
        period_vol = new double[demand_period_num];

        for (unsigned i = 0; i != demand_period_num; ++i)
            period_tt[i] = period_vol[i] = 0;
    }

    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;

    Link(Link&&) = delete;
    Link&& operator=(Link&&) = delete;

    ~Link()
    {
        delete[] period_tt;
        delete[] period_vol;
    }

    const std::string& get_id() const
    {
        return id;
    }

    std::size_t get_no() const
    {
        return no;
    }

    const std::string get_head_node_id() const
    {
        return head_node_id;
    }

    const std::string get_tail_node_id() const
    {
        return tail_node_id;
    }

    const std::string get_geometry() const
    {
        return geo;
    }

    double get_length() const
    {
        return len;
    }

    double get_toll() const
    {
        return toll;
    }

    double get_fftt() const
    {
        return ffs > 0 ? static_cast<double>(len) / ffs * MINUTES_IN_HOUR : INT_MAX;
    }

    double get_generalized_cost(unsigned i, double vot) const
    {
        if (vot <= 0)
            vot = std::numeric_limits<double>::epsilon();

        return period_tt[i] + choice_cost + static_cast<double>(toll) / vot * MINUTES_IN_HOUR;
    }

    double get_route_choice_cost() const
    {
        return choice_cost;
    }

    double get_period_travel_time(unsigned i) const
    {
        return period_tt[i];
    }

    double get_period_vol(unsigned i) const
    {
        return period_vol[i];
    }

    void increase_period_vol(unsigned i, double v)
    {
        period_vol[i] += v;
    }

    void reset_period_vol(unsigned i)
    {
        period_vol[i] = 0;
    }

    // to be defined outside
    double get_period_voc(unsigned i) const;
    double get_period_fftt(unsigned i) const;
    double get_period_avg_tt(unsigned i) const;

    void update_period_travel_time(unsigned i, int iter_no);

private:
    std::string id;
    std::size_t no;

    std::string head_node_id;
    std::size_t head_node_no;

    std::string tail_node_id;
    std::size_t tail_node_no;

    unsigned lane_num;

    double cap;
    double ffs;
    double len;

    double cost;
    double choice_cost;
    double toll;

    double vol;

    // use vector instead?
    // these two can be members of VDFPeriod, why separate them out?
    double* period_tt;
    double* period_vol;

    std::string geo;

    std::vector<VDFPeriod*> vdfps;
};

class Network {

};

class PhyNetwork : public Network {

};

class SPNetwork : public Network {

};

#endif