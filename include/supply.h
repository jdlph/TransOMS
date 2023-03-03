#ifndef GUARD_SUPPLY_H
#define GUARD_SUPPLY_H

#include <cmath>
#include <cstddef>
// #include <forward_list>
#include <limits>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

namespace opendta
{
// to do: change size_t to size_type later
using size_type = unsigned long;

// origin zone id, destination zone id, demand period no, agent type no
using ColumnVecKey = std::tuple<size_type, size_type, unsigned short, unsigned short>;

// some constants
constexpr unsigned short MINUTES_IN_HOUR = 60;

// forward declarations
class Agent;
class AgentType;
class DemandPeriod;

// move ratio_reduction to each individual VDFPeriod?
class SpecialEvent {
public:
    SpecialEvent() = delete;

    SpecialEvent(unsigned short beg_iter_no_, unsigned short end_iter_no_, std::string&& name_)
        : beg_iter_no {beg_iter_no_}, end_iter_no {end_iter_no_}, name {name_}
    {
    }

    SpecialEvent(const SpecialEvent&) = delete;
    SpecialEvent& operator=(const SpecialEvent&) = delete;

    SpecialEvent(SpecialEvent&&) = default;
    SpecialEvent& operator=(SpecialEvent&&) = default;

    ~SpecialEvent() = default;

    unsigned short get_beg_iter_no() const
    {
        return beg_iter_no;
    }

    unsigned short get_end_iter_no() const
    {
        return end_iter_no;
    }

    double get_cap_reduction_ratio(size_type link_no) const
    {
        return ratios.at(link_no);
    }

private:
    unsigned short beg_iter_no;
    unsigned short end_iter_no;

    std::string name;
    std::map<size_type, double> ratios;
};

// PeriodVDF might be a better name
class VDFPeriod {
public:
    VDFPeriod() = delete;

    VDFPeriod(unsigned short no_, double alpha_ = 0.15, double beta_ = 4,
              double mu_ = 1000, double cap_ = 1999, double fftt_ = INT_MAX)
        : no {no_}, alpha {alpha_}, beta {beta_}, mu {mu_}, cap {cap_}, fftt {fftt_}
    {
    }

    VDFPeriod(const VDFPeriod&) = default;
    VDFPeriod& operator=(const VDFPeriod&) = delete;

    VDFPeriod(VDFPeriod&&) = default;
    VDFPeriod& operator=(VDFPeriod&&) = delete;

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
        voc = cap > 0 ? vol / (cap * reduction_ratio) : INT_MAX;
        tt = fftt * (1 + alpha * std::pow(voc, beta));
    }

private:
    unsigned short no;

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

    Link(std::string&& id_, size_type no_,
         std::string&& head_node_id_, size_type head_node_no_,
         std::string&& tail_node_id_, size_type tail_node_no_,
         unsigned short lane_num_, double cap_, double ffs_, double len_,
         std::string&& modes_, std::string&& geo_)
         : id {id_}, no {no_},
           head_node_id {head_node_id_}, head_node_no {head_node_no_},
           tail_node_id {tail_node_id_}, tail_node_no {tail_node_no_},
           lane_num {lane_num_}, cap {cap_}, ffs {ffs_}, len {len_},
           allowed_modes {modes_}, geo {geo_}
    {
    }

    // for connector only
    Link(std::string&& id_, size_type no_,
         const std::string& head_node_id_, size_type head_node_no_,
         const std::string& tail_node_id_, size_type tail_node_no_)
         : id {id_}, no {no_},
           head_node_id {head_node_id_}, head_node_no {head_node_no_},
           tail_node_id {tail_node_id_}, tail_node_no {tail_node_no_}
    {
    }

    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;

    Link(Link&&) = delete;
    Link& operator=(Link&&) = delete;

    ~Link() = default;

    const std::string& get_allowed_modes() const
    {
        return allowed_modes;
    }

    const std::string& get_id() const
    {
        return id;
    }

    auto get_cap() const
    {
        return cap * lane_num;
    }

    size_type get_no() const
    {
        return no;
    }

    const std::string get_head_node_id() const
    {
        return head_node_id;
    }

    size_type get_head_node_no() const
    {
        return head_node_no;
    }

    const std::string get_tail_node_id() const
    {
        return tail_node_id;
    }

    size_type get_tail_node_no() const
    {
        return tail_node_no;
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
        return ffs > 0 ? len / ffs * MINUTES_IN_HOUR : INT_MAX;
    }

    double get_generalized_cost(unsigned short i, double vot) const
    {
        if (vot <= 0)
            vot = std::numeric_limits<double>::epsilon();

        return vdfps[i].get_travel_time() + choice_cost + toll / vot * MINUTES_IN_HOUR;
    }

    double get_route_choice_cost() const
    {
        return choice_cost;
    }

    double get_period_voc(unsigned short i) const
    {
        return vdfps[i].get_voc();
    }

    // useless as it should be always the same as fftt from link itself?
    double get_period_fftt(unsigned short i) const
    {
        return vdfps[i].get_fftt();
    }

    double get_period_travel_time(unsigned short i) const
    {
        return vdfps[i].get_travel_time();
    }

    double get_period_vol(unsigned short i) const
    {
        return vdfps[i].get_vol();
    }

    void increase_period_vol(unsigned short i, double v)
    {
        vdfps[i].increase_vol(v);
    }

    // useless
    void reset_period_vol(unsigned short i)
    {
        vdfps[i].reset_vol();
    }

    void reset_period_vol()
    {
        for (auto& v : vdfps)
            v.reset_vol();
    }

    void add_vdfperiod(VDFPeriod&& vdf)
    {
        vdfps.push_back(vdf);
    }

    void update_period_travel_time(const std::vector<DemandPeriod>* dps, short iter_no);

private:
    std::string id;
    size_type no;

    std::string head_node_id;
    size_type head_node_no;

    std::string tail_node_id;
    size_type tail_node_no;

    // to do: use unsigned short?
    unsigned short lane_num = 1;

    double cap;
    double ffs;
    double len = 0;

    double choice_cost = 0;
    double toll = 0;

    std::string allowed_modes {"all"};
    std::string geo;
    std::vector<VDFPeriod> vdfps;
};

class Node {
public:
    Node() = default;

    Node(size_type no_, std::string&& id_,  double x_, double y_,
         const std::string& z_id_, bool act_node_ = false)
        : no {no_}, id {id_},  x {x_}, y {y_}, zone_id {z_id_}, act_node {act_node_}
    {
    }

    Node(size_type no_, std::string&& id_,  double x_, double y_,
         size_type z_no_, bool act_node_ = false)
        : no {no_}, id {id_},  x {x_}, y {y_}, zone_no {z_no_}, act_node {act_node_}
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

    size_type get_no() const
    {
        return no;
    }

    auto get_zone_no() const
    {
        return zone_no;
    }

    auto get_coordinate() const
    {
        return std::make_pair(x, y);
    }

    std::vector<const Link*>::size_type incoming_link_num() const
    {
        return incoming_links.size();
    }

    std::vector<const Link*>::size_type outgoing_link_num() const
    {
        return outgoing_links.size();
    }

    std::vector<const Link*>& get_incoming_links()
    {
        return incoming_links;
    }

    const std::vector<const Link*>& get_incoming_links() const
    {
        return incoming_links;
    }

    std::vector<const Link*>& get_outgoing_links()
    {
        return outgoing_links;
    }

    const std::vector<const Link*>& get_outgoing_links() const
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
    size_type no;

    double x = 91;
    double y = 181;

    std::string zone_id;
    size_type zone_no;
    bool act_node;

    std::vector<const Link*> incoming_links;
    std::vector<const Link*> outgoing_links;
    // for future performance comparison
    // std::forward_list<const Link*> outgoing_links;
};

class Column {
    friend bool operator==(const Column& c1, const Column& c2)
    {
        if (c1.get_dist() != c2.get_dist())
            return false;

        if (c1.get_links() != c2.get_links())
            return false;

        return true;
    }

public:
    Column() = delete;

    Column(size_type no_) : no {no_}
    {
    }

    Column(size_type no_, double od_vol_, double dist_,
          std::vector<size_type>& links_,
          std::vector<size_type>& nodes_,
          double tt_)
        : no {no_}, od_vol {od_vol_}, dist {dist_},
          links {std::move(links_)}, nodes {std::move(nodes_)},
          tt {tt_}
    {
    }

    Column(const Column&) = default;
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

    unsigned short get_no() const
    {
        return no;
    }

    std::vector<size_type>::size_type get_link_num() const
    {
        return links.size();
    }

    std::vector<size_type>::size_type get_node_num() const
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

    const std::vector<size_type>& get_links() const
    {
        return links;
    }

    const std::vector<size_type>& get_nodes() const
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

    // to do: a better name is needed to pair with shift_volume()
    void reduce_volume(unsigned short iter_no)
    {
        vol *= iter_no / (iter_no + 1.0);
    }

    void set_geometry(std::string&& s)
    {
        geo = s;
    }

    void set_gradient_cost(double c)
    {
        gc = c;
    }

    void set_travel_time(double t)
    {
        tt = t;
    }

    // optimized interfaces
    void update_gradient_cost_diffs(double least_gc)
    {
        gc_ad = gc - least_gc;
        gc_rd = least_gc > 0 ? gc_ad / least_gc : INT_MAX;
    }

    double get_gap() const
    {
        return gc_ad * vol;
    }

    double get_sys_travel_time() const
    {
        return gc * vol;
    }

    void set_toll(double t)
    {
        toll = t;
    }

    double shift_volume(unsigned short iter_no)
    {
        auto step_size = 1.0 / (iter_no + 2);
        auto new_vol = std::max(0.0, vol - step_size * gc_rd * od_vol);

        auto prev_vol = vol;
        vol = new_vol;

        return prev_vol - vol;
    }

    std::size_t get_hash() const
    {
        return std::hash<int>()(get_link_num()) ^ std::hash<double>()(get_dist());
    }

private:
    size_type no;
    double od_vol = 0;

    double dist = 0;
    double gc = 0;
    double gc_ad = 0;
    double gc_rd = 0;
    double tt = 0;
    double toll = 0;
    double vol = 0;

    std::string geo;
    std::vector<size_type> links;
    std::vector<size_type> nodes;
};

struct ColumnHash {
    size_t operator()(const Column& c) const
    {
        return c.get_hash();
    }
};

class ColumnVec {
public:
    ColumnVec() : vol {0}, route_fixed {false}
    {
    }

    ColumnVec(const ColumnVec&) = delete;
    ColumnVec& operator=(const ColumnVec&) = delete;

    ColumnVec(ColumnVec&&) = delete;
    ColumnVec& operator=(ColumnVec&&) = default;

    ~ColumnVec() = default;

    // useless?
    bool is_route_fixed() const
    {
        return route_fixed;
    }

    bool has_column(const Column& c) const
    {
        if (cols.find(c) != cols.end())
        {
            // a further link-by-link comparison
            auto er = cols.equal_range(c);
            for (auto it = er.first; it != er.second; ++it)
            {
                if (it->get_links() == c.get_links())
                    return true;
            }
        }

        return false;
    }

    size_type get_column_num() const
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

    double get_volume() const
    {
        return vol;
    }

    void add_new_column(Column& c)
    {
        cols.emplace(c);
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

    void update(Column& c, unsigned short iter_no)
    {
        // k_path_prob = 1 / (iter_no + 1)
        auto v = vol / (iter_no + 1);

        if (cols.find(c) != cols.end())
        {
            // a further link-by-link comparison
            auto er = cols.equal_range(c);
            for (auto it = er.first; it != er.second; ++it)
            {
                if (it->get_links() == c.get_links())
                {
                    v += it->get_volume();
                    // erase the existing one as it is a const iterator
                    cols.erase(it);
                    return;
                }
            }
        }

        c.increase_volume(v);
        add_new_column(c);
    }

    // move Column c
    void update(Column&& c, unsigned short iter_no)
    {
        // k_path_prob = 1 / (iter_no + 1)
        auto v = vol / (iter_no + 1);

        if (cols.find(c) != cols.end())
        {
            // a further link-by-link comparison
            auto er = cols.equal_range(c);
            for (auto it = er.first; it != er.second; ++it)
            {
                if (it->get_links() == c.get_links())
                {
                    const_cast<Column&>(*it).increase_volume(v);
                    return;
                }
            }
        }

        c.increase_volume(v);
        add_new_column(c);
    }

private:
    double vol;
    bool route_fixed;

    std::unordered_multiset<Column, ColumnHash> cols;
};

class ColumnPool {
public:
    ColumnPool() = default;

    ColumnPool(const ColumnPool&) = delete;
    ColumnPool& operator=(const ColumnPool) = delete;

    ColumnPool(ColumnPool&&) = default;
    ColumnPool& operator=(ColumnPool&&) = delete;

    ColumnVec& get_column_vec(const ColumnVecKey& k)
    {
        return cp.at(k);
    }

    const ColumnVec& get_column_vec(const ColumnVecKey& k) const
    {
        return cp.at(k);
    }

    std::map<ColumnVecKey, ColumnVec>& get_column_vecs()
    {
        return cp;
    }

    bool contains_key(const ColumnVecKey& k) const
    {
        return cp.find(k) != cp.end();
    }

    // useless?
    void create_columnvec(const ColumnVecKey& cvk)
    {
        cp[cvk] = ColumnVec();
    }

    void update(const ColumnVecKey& cvk, double vol)
    {
        if (!contains_key(cvk))
            cp[cvk] = ColumnVec();

        cp[cvk].increase_volume(vol);
    }

private:
    std::map<ColumnVecKey, ColumnVec> cp;
};

class Zone {
public:
    using Vertex = std::pair<double, double>;
    using Boundary = std::tuple<double, double, double, double>;

    Zone() = default;

    explicit Zone(size_type no_) : no {no_}
    {
    }

    Zone(size_type no_, const std::string& id_, unsigned short bin_id_) : no {no_}, id {id_}, bin_id {bin_id_}
    {
    }

    Zone(const Zone&) = delete;
    Zone& operator=(const Zone&) = delete;

    Zone(Zone&&) = default;
    Zone& operator=(Zone&&) = default;

    ~Zone() = default;

    const std::vector<size_type>& get_activity_nodes() const
    {
        return act_nodes;
    }

    size_type get_activity_nodes_num() const
    {
        return act_nodes.size();
    }

    unsigned short get_bin_index() const
    {
        return bin_id;
    }

    const Boundary& get_boundaries() const
    {
        return bd;
    }

    const Node* get_centroid() const
    {
        return centroid;
    }

    const Vertex get_coordinate() const
    {
        return std::make_pair(x, y);
    }

    std::string get_geometry() const
    {
        try
        {
            auto U = std::to_string(std::get<0>(bd));
            auto D = std::to_string(std::get<1>(bd));
            auto L = std::to_string(std::get<2>(bd));
            auto R = std::to_string(std::get<3>(bd));

            return "LINESTRING (" + L + U + ',' + R + U + ',' + R + D + ',' + L + D + ',' + L + U + ')';
        }
        catch(const std::exception& e)
        {
            return "INESTRING ()";
        }
    }

    const auto& get_id() const
    {
        return id;
    }

    size_type get_no() const
    {
        return no;
    }

    const std::vector<size_type>& get_nodes() const
    {
        return nodes;
    }

    double get_production() const
    {
        return prod;
    }

    void add_activity_node(size_type node_no)
    {
        act_nodes.push_back(node_no);
    }

    void add_node(size_type node_no)
    {
        nodes.push_back(node_no);
    }

    void set_bin_index(unsigned short bi)
    {
        bin_id = bi;
    }

    void set_geometry(double x_, double y_, const Boundary& bd_)
    {
        x = x_;
        y = y_;
        bd = bd_;
    }

    void set_production(double p)
    {
        prod = p;
    }

    void set_centroid(const Node* p)
    {
        centroid = p;
    }

private:
    std::string id;
    size_type no = 0;

    std::vector<size_type> act_nodes;
    std::vector<size_type> nodes;
    const Node* centroid;

    // the following members are related to zone synthesis
    unsigned short bin_id = 0;

    Boundary bd;
    double x = 91;
    double y = 181;

    double prod = 0;
};

// an abstract class
class Network {
public:
    virtual std::vector<Node*>& get_nodes() = 0;
    virtual const std::vector<Node*>& get_nodes() const = 0;

    virtual std::vector<Link*>& get_links() = 0;
    virtual const std::vector<Link*>& get_links() const = 0;

    virtual std::map<std::string, Zone*>& get_zones() = 0;
    virtual const std::map<std::string, Zone*>& get_zones() const = 0;

    virtual const std::vector<const Node*>& get_centroids() const = 0;

    virtual size_type get_last_thru_node_no() const = 0;

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

        // this releases memory for centroids as well
        for (auto p : nodes)
            delete p;
    }

    size_type get_last_thru_node_no() const override
    {
        return last_thru_node_no;
    }

    size_type get_link_num() const override
    {
        return links.size();
    }

    size_type get_node_num() const override
    {
        return nodes.size();
    }

    std::vector<Node*>& get_nodes() override
    {
        return nodes;
    }

    const std::vector<Node*>& get_nodes() const override
    {
        return nodes;
    }

    std::vector<Link*>& get_links() override
    {
        return links;
    }

    const std::vector<Link*>& get_links() const override
    {
        return links;
    }

    std::map<std::string, Zone*>& get_zones() override
    {
        return zones;
    }

    const std::map<std::string, Zone*>& get_zones() const override
    {
        return zones;
    }

    const std::vector<const Node*>& get_centroids() const override
    {
        return centroids;
    }

    void add_link(Link* link)
    {
        auto head_node = nodes[link->get_head_node_no()];
        auto tail_node = nodes[link->get_tail_node_no()];

        head_node->add_outgoing_link(link);
        tail_node->add_incoming_link(link);

        links.push_back(link);
    }

    // user is responsible for the uniqueness of node id
    void add_node(Node* n)
    {
        id_no_map[n->get_id()] = n->get_no();

        nodes.push_back(n);
    }

    // user is responsible for the uniqueness of zone id
    void add_zone(Zone* z)
    {
        zones[z->get_id()] = z;
    }

    void collect_centroids()
    {
        for (const auto& z : zones)
        {
            // make sure centroid is not nullptr
            if (z.second->get_centroid())
                centroids.push_back(z.second->get_centroid());
        }
    }

    bool contains_zone(const std::string& zone_id) const
    {
        return zones.find(zone_id) != zones.end();
    }

    size_type get_node_no(const std::string& node_id) const
    {
        return id_no_map.at(node_id);
    }

    size_type get_zone_no(const std::string& zone_id) const
    {
        return zones.at(zone_id)->get_no();
    }

    void set_last_thru_node_no(size_type no)
    {
        last_thru_node_no = no;
    }

private:
    size_type last_thru_node_no;

    std::vector<Link*> links;
    std::vector<Node*> nodes;
    std::map<std::string, size_type> id_no_map;

    std::vector<const Node*> centroids;
    std::map<std::string, Zone*> zones;

    std::vector<Agent> agents;
    // time-dependent agents for simulation
    std::map<unsigned, size_type> td_agents;
};

class SPNetwork : public Network {
public:
    SPNetwork() = delete;

    SPNetwork(unsigned short no_, PhyNetwork& pn_, ColumnPool& cp_, DemandPeriod& dp_, const AgentType* at_)
        : no {no_}, pn {&pn_}, cp {&cp_}, dp {&dp_}, at {at_}
    {
    }

    SPNetwork(const SPNetwork&) = delete;
    SPNetwork& operator=(const SPNetwork&) = delete;

    SPNetwork(SPNetwork&&) = delete;
    SPNetwork& operator=(SPNetwork&&) = delete;

    ~SPNetwork()
    {
        delete[] link_costs;
        delete[] node_costs;
        delete[] next_nodes;
        delete[] link_preds;
        delete[] node_preds;
    }

    size_type get_last_thru_node_no() const override
    {
        return pn->get_last_thru_node_no();
    }

    size_type get_link_num() const override
    {
        return pn->get_link_num();
    }

    size_type get_node_num() const override
    {
        return pn->get_node_num();
    }

    std::vector<Link*>& get_links() override
    {
        return pn->get_links();
    }

    const std::vector<Link*>& get_links() const override
    {
        return pn->get_links();
    }

    std::vector<Node*>& get_nodes() override
    {
        return pn->get_nodes();
    }

    const std::vector<Node*>& get_nodes() const override
    {
        return pn->get_nodes();
    }

    const std::vector<size_type>& get_orig_nodes() const
    {
        return orig_nodes;
    }

    std::map<std::string, Zone*>& get_zones() override
    {
        return pn->get_zones();
    }

    const std::map<std::string, Zone*>& get_zones() const override
    {
        return pn->get_zones();
    }

    const std::vector<const Node*>& get_centroids() const override
    {
        return pn->get_centroids();
    }

    void add_orig_nodes(const Zone* z)
    {
        orig_nodes.push_back(z->get_centroid()->get_no());
    }

    void generate_columns(unsigned short iter_no);

private:
    void initialize();
    void reset();
    void update_link_costs();

    // the most efficient deque implementation of the MLC algorithm adopted from Path4GMNS
    void single_source_shortest_path(size_type src_node_no);
    void backtrace_shortest_path_tree(size_type src_node_no, unsigned short iter_no);

    // static function?
    bool is_mode_compatible(const std::string& s1, const std::string& s2)
    {
        static const std::string all_modes {"all"};

        return s1.find(s2) != std::string::npos || s1.find(all_modes) != std::string::npos;
    }

private:
    unsigned short no;

    // NetworkHandle is responsible to clean them up
    const AgentType* at;
    DemandPeriod* dp;

    ColumnPool* cp;
    PhyNetwork* pn;

    // inconsistent with the type of node no
    // but a network usually would not have 2,147,483,647 nodes
    long* link_preds;
    long* node_preds;
    // deque
    long* next_nodes;
    double* link_costs;
    double* node_costs;

    std::vector<size_type> orig_nodes;

    static constexpr long nullnode = -1;
    static constexpr long pastnode = -3;
};

} // namespace opendta

#endif