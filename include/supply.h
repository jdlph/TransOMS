#ifndef GUARD_SUPPLY_H
#define GUARD_SUPPLY_H

#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

namespace opendta
{
// to do: change size_t to size_type later
using size_type = unsigned long;

// origin zone id, destination zone id, demand period no
using ColumnVecKey = std::tuple<std::string, std::string, unsigned>;

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

// PeriodVDF might be a better name
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

    const std::string& get_allowed_modes() const
    {
        return allowed_modes;
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

    std::size_t get_head_node_no() const
    {
        return head_node_no;
    }

    const std::string get_tail_node_id() const
    {
        return tail_node_id;
    }

    std::size_t get_tail_node_no() const
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

    // useless
    void increase_period_vol(unsigned i, double v)
    {
        vdfps[i].increase_vol(v);
    }

    void increase_period_vol(double v)
    {
        for (auto& vdf : vdfps)
            vdf.increase_vol(v);
    }

    // useless
    void reset_period_vol(unsigned i)
    {
        vdfps[i].reset_vol();
    }

    void reset_period_vol()
    {
        for (auto& v : vdfps)
            v.reset_vol();
    }

    void update_period_travel_time(unsigned iter_no);

private:
    std::string id;
    std::size_t no;

    std::string head_node_id;
    std::size_t head_node_no;

    std::string tail_node_id;
    std::size_t tail_node_no;

    // to do: use unsigned short?
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
    std::size_t no;

    double x = 91;
    double y = 181;

    std::string zone_id;
    bool act_node;

    std::vector<const Link*> incoming_links;
    std::vector<const Link*> outgoing_links;
};

class Column {
public:
    Column() = delete;

    // Column(size_type id_) : id {id_}
    // {
    // }

    Column(size_type id_, double dist_, std::vector<std::size_t>&& links_, std::vector<std::size_t>&& nodes_)
        : id {id_}, dist {dist_}, links {links_}, nodes {nodes_}
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

    // to do: a better name is needed to pair with shift_volume()
    void reduce_volume(unsigned short iter_no)
    {
        vol *= static_cast<double>(iter_no) / (iter_no + 1);
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
        gc_rd = least_gc > 0? gc_ad / least_gc : INT_MAX;
    }

    double get_gap() const
    {
        return gc_ad * vol;
    }

    double get_sys_travel_time() const
    {
        return gc * vol;
    }

    double shift_volume(unsigned short iter_no)
    {
        auto step_size = 1.0 / (iter_no + 2) * vol;
        auto new_vol = std::max(0.0, vol - step_size * gc_rd);

        auto prev_vol = vol;
        vol = new_vol;

        return prev_vol - vol;
    }

private:
    size_type id;

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

    void set_toll(double t)
    {
        toll = t;
    }

    void set_volume(double v)
    {
        vol = v;
    }

    void update(Column& c, unsigned short iter_no)
    {
        // k_path_prob = 1 / (iter_no + 1)
        auto v = vol / (iter_no + 1);

        if (cols.find(c) == cols.end())
        {
            c.increase_volume(v);
            add_new_column(c);
            return;
        }

        // a further link-by-link comparison
        auto er = cols.equal_range(c);
        for (auto it = er.first; it != er.second; ++it)
        {
            if (it->get_links() == c.get_links())
            {
                v += it->get_volume();
                // erase the existing one as it is a const iterator and the following operation is not allowed
                // it->increase_volume(v);
                // it can only be avoided by designing a customer hash table
                cols.erase(it);
                break;
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

        if (cols.find(c) == cols.end())
        {
            c.increase_volume(v);
            add_new_column(c);
            return;
        }

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

        c.increase_volume(v);
        add_new_column(c);
    }

private:
    double toll;
    double vol;
    bool route_fixed;

    std::unordered_multiset<Column, ColumnHash> cols;
};

// to do: no need to use at_id as part of the key
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

private:
    std::map<ColumnVecKey, ColumnVec> cp;
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

    const Node* get_centroid() const
    {
        return centroid;
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
    virtual std::vector<const Node*>& get_nodes() = 0;
    virtual const std::vector<const Node*>& get_nodes() const = 0;

    virtual std::vector<Link*>& get_links() = 0;
    virtual const std::vector<Link*>& get_links() const = 0;

    virtual std::map<std::string, Zone>& get_zones() = 0;
    virtual const std::map<std::string, Zone>& get_zones() const = 0;

    virtual const std::vector<const Node*>& get_centroids() const = 0;

    virtual size_t get_last_thru_node_no() const = 0;

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

    std::size_t get_last_thru_node_no() const override
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

    std::vector<const Node*>& get_nodes() override
    {
        return nodes;
    }

    const std::vector<const Node*>& get_nodes() const override
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

    std::map<std::string, Zone>& get_zones() override
    {
        return zones;
    }

    const std::map<std::string, Zone>& get_zones() const override
    {
        return zones;
    }

    const std::vector<const Node*>& get_centroids() const override
    {
        return centroids;
    }

    void collect_centroids()
    {
        for (const auto& z : zones)
        {
            // make sure centroid is not nullptr
            if (z.second.get_centroid())
                centroids.push_back(z.second.get_centroid());
        }
    }

private:
    std::size_t last_thru_node_no;

    std::vector<Link*> links;
    std::vector<const Node*> nodes;

    std::vector<const Node*> centroids;
    std::map<std::string, Zone> zones;

    std::vector<Agent> agents;
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
        delete[] node_costs;
        delete[] next_nodes;
        delete[] link_preds;
    }

    std::size_t get_last_thru_node_no() const override
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

    std::vector<const Node*>& get_nodes() override
    {
        return pn->get_nodes();
    }

    const std::vector<const Node*>& get_nodes() const override
    {
        return pn->get_nodes();
    }

    const std::vector<size_t>& get_orig_nodes() const
    {
        return orig_nodes;
    }

    std::map<std::string, Zone>& get_zones() override
    {
        return pn->get_zones();
    }

    const std::map<std::string, Zone>& get_zones() const override
    {
        return pn->get_zones();
    }

    void reset()
    {
        for (size_type i = 0, n = get_node_num(); i != n; ++i)
        {
            node_costs[i] = std::numeric_limits<double>::max();
            next_nodes[i] = link_preds[i] = node_preds[i] = nullnode;
        }
    }

    void generate_columns(unsigned short iter_no)
    {
        update_link_costs();

        for (auto s : get_orig_nodes())
        {
            single_source_shortest_path(s);
            backtrace_shortest_path_tree(s, iter_no);
            reset();
        }
    }

    void update_link_costs()
    {
        unsigned dp_id = dp->get_id();
        double vot = dp->get_agent_vot();

        for (auto p : get_links())
            link_costs[p->get_no()] = p->get_generalized_cost(dp_id, vot);
    }

private:
    void backtrace_shortest_path_tree(size_type src_node_no, unsigned short iter_no)
    {
        const auto p = get_nodes()[src_node_no];
        if (p->get_outgoing_links().empty())
            return;

        const auto oz_id = p->get_zone_id();

        for (const auto c : get_centroids())
        {
            auto dz_id = c->get_zone_id();
            if (oz_id == dz_id)
                continue;

            ColumnVecKey cvk {oz_id, dz_id, dp->get_id()};
            if (!cp->contains_key(cvk))
                continue;

            auto& cv = cp->get_column_vec(cvk);
            if (cv.is_route_fixed())
                continue;

            std::vector<std::size_t> link_path;
            std::vector<std::size_t> node_path;

            double dist = 0;
            // use long intensionally as node_preds is long*
            // otherwise, size_t (later size_type) will be automatically deduced via auto.
            long cur_node = c->get_no();
            while (cur_node >= 0)
            {
                node_path.push_back(cur_node);

                auto cur_link = link_preds[cur_node];
                if (cur_link >=0)
                {
                    link_path.push_back(cur_link);
                    dist += get_links()[cur_link]->get_length();
                }

                cur_node = node_preds[cur_node];
            }

            if (link_path.empty())
                continue;

            // move temporary Column
            cv.update(Column{cv.get_column_num(), dist, link_path, node_path}, iter_no);
        }
    }

    // static function?
    bool is_mode_compatible(const std::string& s1, const std::string& s2)
    {
        static const std::string all_modes {"all"};

        return s1.find(s2) == std::string::npos && s1.find(all_modes) == std::string::npos;
    }

    // the most efficient deque implementation of the MLC algorithm adopted from Path4GMNS
    void single_source_shortest_path(size_type src_node_no)
    {
        for (size_type cur_node = src_node_no, deq_head = nullnode, deq_tail = nullnode;;)
        {
            if (cur_node <= get_last_thru_node_no() || cur_node == src_node_no)
            {
                for (const auto link : get_nodes()[cur_node]->get_outgoing_links())
                {
                    if (is_mode_compatible(link->get_allowed_modes(), dp->get_agent_type_name()))
                        continue;

                    size_type new_node = link->get_tail_node_no();
                    double new_cost = node_costs[cur_node] + link_costs[link->get_no()];
                    if (new_cost < node_costs[new_node])
                    {
                        node_costs[new_node] = new_cost;
                        link_preds[new_node] = link->get_no();

                        if (next_nodes[new_node] == pastnode)
                        {
                            next_nodes[new_node] = deq_head;
                            deq_head = new_node;

                            if (deq_tail == nullnode)
                                deq_tail = new_node;
                        }
                        else if (next_nodes[new_node] == nullnode && new_node != deq_tail)
                        {
                            if (deq_tail == nullnode)
                            {
                                deq_head = deq_tail = new_node;
                                next_nodes[deq_tail] = nullnode;
                            }
                            else
                            {
                                next_nodes[deq_tail] = new_node;
                                deq_tail = new_node;
                            }
                        }
                    }
                }
            }

            if (deq_head < 0)
                break;

            cur_node = deq_head;
            deq_head = next_nodes[cur_node];
            next_nodes[cur_node] = pastnode;

            if (deq_tail == cur_node)
                deq_tail = nullnode;
        }
    }

private:
    // use unsigned short instead?
    unsigned id;

    ColumnPool* cp;
    // Assignment is responsible to clean it up.
    DemandPeriod* dp;
    PhyNetwork* pn;

    // inconsistent with the type of node no
    // but the network usually would not exceed 2,147,483,647 in terms of number of nodes.
    // no need for node predecessors which can be easily inferred
    // change it to link* link_preds for better performance in backtrace_shortest_path_tree()?
    long* link_preds;
    long* node_preds;
    // deque
    long* next_nodes;
    double* link_costs;
    double* node_costs;

    std::vector<std::size_t> orig_nodes;

    static constexpr long nullnode = -1;
    static constexpr long pastnode = -3;
};

} // namespace opendta

#endif