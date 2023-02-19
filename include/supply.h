#ifndef GUARD_SUPPLY_H
#define GUARD_SUPPLY_H

#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

// some constants
constexpr unsigned MINUTES_IN_HOUR = 60;

// forward declaration
class DemandPeriod;

// move ratio_reduction to each individual VDFPeriod?
class SpecialEvent {
public:
    SpecialEvent() = delete;

    SpecialEvent(unsigned beg_iter_no_, unsigned end_iter_no_, std::string&& name_)
        : beg_iter_no {beg_iter_no_}, end_iter_no {end_iter_no_}, name {name_}
    {
    }

    SpecialEvent(const SpecialEvent&) = delete;
    SpecialEvent& operator=(const SpecialEvent&) = delete;

    SpecialEvent(SpecialEvent&&) = default;
    SpecialEvent& operator=(SpecialEvent&&) = default;

    ~SpecialEvent() = default;

    unsigned get_beg_iter_no() const
    {
        return beg_iter_no;
    }

    unsigned get_end_iter_no() const
    {
        return end_iter_no;
    }

    double get_cap_reduction_ratio(size_t link_no) const
    {
        return ratios.at(link_no);
    }

private:
    unsigned beg_iter_no;
    unsigned end_iter_no;

    std::string name;
    std::map<size_t, double> ratios;
};

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
        voc = cap > 0 ? static_cast<double>(vol) / (cap * reduction_ratio) : INT_MAX;
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

    std::string allowed_modes;
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

    double x = 91;
    double y = 181;

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

    // the following functions can have unified names via traditional C++ practices.
    // take get_dist for example, double distance() const and double& distance()
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

    // useless?
    bool is_route_fixed() const
    {
        return route_fixed;
    }

    bool has_column(const Column& c) const
    {
        if (cols.find(c) == cols.end())
            return false;

        // a further link-by-link comparison
        auto er = cols.equal_range(c);
        for (auto it = er.first; it != er.second; ++it)
        {
            if (it->get_links() == c.get_links())
                return true;
        }

        return false;
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

    // use Column* instead (which will require heap memory)?
    std::unordered_multiset<Column, ColumnHash> cols;
};

class ColumnPool {
public:
    using Key = std::tuple<unsigned, unsigned, std::string, std::string>;

    ColumnPool() = default;

    ColumnPool(const ColumnPool&) = delete;
    ColumnPool& operator=(const ColumnPool) = delete;

    ColumnPool(ColumnPool&&) = default;
    ColumnPool& operator=(ColumnPool&&) = delete;

    ColumnVec& get_column_vec(const Key& k)
    {
        return cp.at(k);
    }

    const ColumnVec& get_column_vec(const Key& k) const
    {
        return cp.at(k);
    }

private:
    std::map<Key, ColumnVec> cp;
};

class Zone {
public:
    using Vertex = std::pair<double, double>;
    using Boundary = std::tuple<Vertex, Vertex, Vertex, Vertex>;

    Zone() = default;

    explicit Zone(size_t no_) : no {no_}
    {
    }

    Zone(size_t no_, unsigned bin_id_) : no {no_}, bin_id {bin_id_}
    {
    }

    Zone(const Zone&) = delete;
    Zone& operator=(const Zone&) = delete;

    Zone(Zone&&) = default;
    Zone& operator=(Zone&&) = default;

    ~Zone() = default;

    const std::vector<size_t>& get_activity_nodes() const
    {
        return act_nodes;
    }

    std::vector<size_t>::size_type get_activity_nodes_num() const
    {
        return act_nodes.size();
    }

    unsigned get_bin_index() const
    {
        return bin_id;
    }

    const Boundary& get_boundaries() const
    {
        return bd;
    }

    const Node& get_centroid() const
    {
        return *centroid;
    }

    const Vertex& get_coordinate() const
    {
        return std::make_pair(x, y);
    }

    // incomplete
    std::string get_geometry() const
    {
        try
        {
            auto U = std::get<0>(bd);
            auto D = std::get<1>(bd);
            auto L = std::get<2>(bd);
            auto R = std::get<3>(bd);
        }
        catch(const std::exception& e)
        {
            return "INESTRING ()";
        }
    }

    double get_production() const
    {
        return prod;
    }

    void add_activity_node(size_t node_no)
    {
        act_nodes.push_back(node_no);
    }

    void set_bin_index(unsigned bi)
    {
        bin_id = bi;
    }

    // incomplete
    void set_geometry(double x_, double y_)
    {
        x = x_;
        y = y_;
    }

    void set_production(double p)
    {
        prod = p;
    }

private:
    std::string id;
    size_t no = 0;

    std::vector<size_t> act_nodes;
    Node* centroid;

    // the following members are related to zone synthesis
    unsigned bin_id = 0;

    Boundary bd;
    double x = 91;
    double y = 181;

    double prod = 0;
};

// an abstract class
class Network {
public:
    using size_type = std::vector<const Node*>::size_type;

    virtual std::vector<const Node*>& get_nodes() = 0;
    virtual const std::vector<const Node*>& get_nodes() const = 0;

    virtual std::vector<const Link*>& get_links() = 0;
    virtual const std::vector<const Link*>& get_links() const = 0;

    virtual std::map<std::string, Zone>& get_zones() = 0;
    virtual const std::map<std::string, Zone>& get_zones() const = 0;

    virtual size_t* cost_labels() = 0;
    virtual const size_t* cost_labels() const = 0;

    // no need for node predecessors which can be easily inferred
    virtual size_t* link_preds() = 0;
    virtual const size_t* link_preds() const = 0;

    // deque
    virtual size_t* next_nodes() = 0;
    virtual const size_t* next_nodes() const = 0;

    virtual size_t get_last_thru_node_no() const = 0;

    virtual const std::vector<size_t>& get_orig_centroids() const = 0;
    virtual const std::vector<size_t> get_all_centroids() const = 0;

    virtual size_type get_link_num() const = 0;
    virtual size_type get_node_num() const = 0;

    virtual ~Network() {}
};

class PhyNetwork : public Network {
public:
    PhyNetwork() = default;

    PhyNetwork(const PhyNetwork&) = delete;
    PhyNetwork& operator=(const PhyNetwork&) = delete;

    PhyNetwork(PhyNetwork&&) = default;
    PhyNetwork& operator=(PhyNetwork&&) = default;

    ~PhyNetwork()
    {
        // release memory
        for (auto p : links)
            delete p;

        for (auto p : nodes)
            delete p;
    }

    std::vector<const Node*>& get_nodes() override
    {
        return nodes;
    }

    const std::vector<const Node*>& get_nodes() const override
    {
        return nodes;
    }

    std::vector<const Link*>& get_links() override
    {
        return links;
    }

    const std::vector<const Link*>& get_links() const override
    {
        return links;
    }

    std::map<std::string, Zone>& get_zones() override
    {
        return zones;
    }

    const std::map<std::string, Zone>& get_zones() const override
    {
        return zones;
    }

    // useless as we need centroid objects / pointers?
    // performance issue: we will use this extensively in column generation.
    const std::vector<size_t> get_all_centroids() const override
    {
        std::vector<size_t> vec;
        for (auto i = last_thru_node_no + 1; i != nodes.size(); ++i)
            vec.push_back(nodes[i]->get_no());

        // it will be moved outside the function body.
        return vec;
    }

private:
    std::size_t last_thru_node_no;

    std::vector<const Link*> links;
    // it can be vector<Node> if we store the centroid of a zone as node_no
    std::vector<const Node*> nodes;
    std::vector<Agent> agents;
    std::map<std::string, Zone> zones;

    // time-dependent agents for simulation
    std::map<unsigned, size_t> td_agents;
};

class SPNetwork : public Network {
public:
    SPNetwork() = delete;

    // do i really need to create an instance of SPNetwork for each agent type
    // given a demand period?
    SPNetwork(unsigned id_, DemandPeriod* dp_) : id {id_}, dp {dp_}
    {
    }

    SPNetwork(const SPNetwork&) = delete;
    SPNetwork& operator=(const SPNetwork&) = delete;

    SPNetwork(SPNetwork&) = delete;
    SPNetwork& operator=(SPNetwork&) = delete;

    ~SPNetwork()
    {
        delete[] costs;
        delete[] deque;
        delete[] preds;
    }

    void reset()
    {
        for (auto i = 0, n = get_node_num(); i != n; ++i)
        {
            costs[i] = INT_MAX;
            deque[i] = preds[i] = -1;
        }
    }

private:
    // use unsigned short instead?
    unsigned id;
    // Assignment is responsible to clean it up.
    DemandPeriod* dp;

    // inconsistent with the type of node no
    // but the network usually would not exceed 2,147,483,647 in terms of number of nodes.
    long* preds;
    long* deque;
    double* costs;

    std::vector<std::size_t> centroids;
};

#endif