#ifndef GUARD_NETWORK_H
#define GUARD_NETWORK_H

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

// some constants
constexpr unsigned MINUTES_IN_HOUR = 60;

class VDFPeriod {
public:
    VDFPeriod() = delete;

    VDFPeriod(unsigned no_, double alpha_ = 0.15, double beta_ = 4,
              double mu_ = 1000, double cap_ = 1999, double fftt_ = INT_MAX)
        : no {no_}, alpha {alpha_}, beta {beta_}, mu {mu_}, cap {cap_}, fftt {fftt_}
    {
    }

    VDFPeriod(const VDFPeriod&) = delete;
    VDFPeriod& operator=(const VDFPeriod&) = delete;

    VDFPeriod(VDFPeriod&&) = default;
    VDFPeriod& operator=(VDFPeriod&&) = default;

    double get_travel_time() const
    {
        return tt;
    }

    double get_fftt() const
    {
        return fftt;
    }

    double get_voc() const
    {
        return voc;
    }

    double get_vol() const
    {
        return vol;
    }

    void increase_vol(double v)
    {
        vol += v;
    }

    void reset_vol()
    {
        vol = 0;
    }

    void run_bpr(double reduction_ratio = 1)
    {
        voc = cap > 0 ? static_cast<double>(vol) / cap * reduction_ratio : INT_MAX;
        tt = fftt * (1 + alpha * std::pow(voc, beta));
    }

private:
    unsigned no;

    double alpha;
    double beta;
    double mu;

    double cap;
    double fftt;

    double tt = INT_MAX;
    double voc = 0;
    double vol = 0;
};

class Link {
public:
    Link() = default;

    Link(std::string&& id_, std::size_t no_,
         std::string&& head_node_id_, std::size_t head_node_no_,
         std::string&& tail_node_id_, std::size_t tail_node_no_,
         unsigned lane_num_, double cap_, double ffs_, double len_)
         : id {id_}, no {no_},
           head_node_id {head_node_id_}, head_node_no {head_node_no_},
           tail_node_id {tail_node_id_}, tail_node_no {tail_node_no},
           lane_num {lane_num_}, cap {cap_}, ffs {ffs_}, len {len_}
    {
    }

    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;

    Link(Link&&) = delete;
    Link& operator=(Link&&) = delete;

    ~Link() = default;

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

        return vdfps[i].get_travel_time() + choice_cost + static_cast<double>(toll) / vot * MINUTES_IN_HOUR;
    }

    double get_route_choice_cost() const
    {
        return choice_cost;
    }

    double get_period_voc(unsigned i) const
    {
        return vdfps[i].get_voc();
    }

    // useless as it should be always the same as fftt from link itself?
    double get_period_fftt(unsigned i) const
    {
        return vdfps[i].get_fftt();
    }

    double get_period_travel_time(unsigned i) const
    {
        return vdfps[i].get_travel_time();
    }

    double get_period_vol(unsigned i) const
    {
        return vdfps[i].get_vol();
    }

    void increase_period_vol(unsigned i, double v)
    {
        vdfps[i].increase_vol(v);
    }

    void reset_period_vol(unsigned i)
    {
        vdfps[i].reset_vol();
    }

    void update_period_travel_time(unsigned iter_no);

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

    double choice_cost = 0;
    double toll = 0;

    std::string geo;
    // ditch pointers to take advantage of stack memory and avoid potential memory fragmentation
    // given instances of VDFPeriod are small objects
    std::vector<VDFPeriod> vdfps;
};

class Node {
public:
    Node() = default;

    Node(std::size_t no_, std::string&& id_,  double x_, double y_,
         std::string&& z_id, bool act_node_ = false)
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

class Column {
public:
    Column() = delete;

    Column(unsigned id_) : id {id_}
    {
    }

    Column(const Column&) = delete;
    Column& operator=(const Column&) = delete;

    Column(Column&&) = default;
    Column& operator=(Column&&) = delete;

    ~Column()
    {
    }

    // the following functions can have unified names. take get_dist for example,
    // double distance() const
    // double& distance()
    double get_dist() const
    {
        return dist;
    }

    double get_gradient_cost() const
    {
        return gc;
    }

    double get_gradient_cost_abs_diff() const
    {
        return gc_ad;
    }

    double get_gradient_cost_rel_diff() const
    {
        return gc_rd;
    }

    unsigned get_id() const
    {
        return id;
    }

    std::vector<size_t>::size_type get_link_num() const
    {
        return links.size();
    }

    std::vector<size_t>::size_type get_node_num() const
    {
        return nodes.size();
    }

    double get_travel_time() const
    {
        return tt;
    }

    double get_toll() const
    {
        return toll;
    }

    double get_volume() const
    {
        return vol;
    }

    const std::vector<size_t>& get_links() const
    {
        return links;
    }

    const std::vector<size_t>& get_nodes() const
    {
        return nodes;
    }

    void increase_toll(double t)
    {
        toll += t;
    }

    void increase_volume(double v)
    {
        vol += v;
    }

    void set_distance(double d)
    {
        dist = d;
    }

    void set_geometry(std::string&& s)
    {
        geo = s;
    }

    void set_gradient_cost(double c)
    {
        gc = c;
    }

    void set_gradient_cost_abs_diff(double ad)
    {
        gc_ad = ad;
    }

    void set_gradient_cost_rel_diff(double rd)
    {
        gc_rd = rd;
    }

    void set_travel_time(double t)
    {
        tt = t;
    }

private:
    unsigned id;

    double dist = 0;
    double gc = 0;
    double gc_ad = 0;
    double gc_rd = 0;
    double tt = 0;
    double toll = 0;
    double vol = 0;

    std::string geo;
    std::vector<size_t> links;
    std::vector<size_t> nodes;
};

struct ColumnHash {
    size_t operator()(const Column& c) const
    {
        return std::hash<int>()(c.get_link_num()) ^ std::hash<double>()(c.get_dist());
    }
};

class ColumnVec {
public:
    ColumnVec() : vol {0}, route_fixed {false}
    {
    }

    ColumnVec(const ColumnVec&) = delete;
    ColumnVec& operator=(const ColumnVec) = delete;

    ColumnVec(ColumnVec&&) = delete;
    ColumnVec& operator=(ColumnVec&&) = delete;

    ~ColumnVec() = default;

    bool is_route_fixed() const
    {
        return route_fixed;
    }

    int get_column_num() const
    {
        return cols.size();
    }

    std::unordered_multiset<Column, ColumnHash>& get_columns()
    {
        return cols;
    }

    // it might be useless according to Path4GMNS
    const std::unordered_multiset<Column, ColumnHash>& get_columns() const
    {
        return cols;
    }

    void add_new_column(Column&& c)
    {
        cols.insert(c);
    }

    void increase_volume(double v)
    {
        vol += v;
    }

    void set_volume(double v)
    {
        vol = v;
    }

private:
    double vol;
    bool route_fixed;

    // use Column* instead, which will requires heap memory?
    std::unordered_multiset<Column, ColumnHash> cols;
};

class Network {

};

class PhyNetwork : public Network {

};

class SPNetwork : public Network {

};

#endif