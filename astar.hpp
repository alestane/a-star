#pragma once

// Template A* pathfinder by Nevin Flanagan

/*
 Call route::plan(start, goal, adjacent, heuristic[, debugger]):
    start and goal should be of the same type (the state type)
    adjacent should be a callable that takes an argument of the state
        type, and returns an iterable of pairs where the first element
        is the state type (or if the state type is a reference type, a
        pointer or reference wrapper to the underlying type), and the
        second element describes the cost or weight of that link; this
        should be an ordinal type called the cost type
    heuristic should be a callable that takes two arguments of the state
        type and returns a value of the cost type
 
    The result type is the state type, or a reference_wrapper equivalent
        to the state type if the state type was a reference type. The
        actual return type of route::plan is a vector of the result type
        containing the states to be traversed, in order. If no path was
        found, route::plan returns an empty vector.
 
    route::plan accepts an optional 5th argument, which is a debug
        function. If provided, this should be a return-void callable
        that accepts as its single argument a vector of the result type
        containing the states in the path currently being examined. It
        will be called once per node being considered.
 */

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <limits>
#include <utility>
#include <functional>
#include <algorithm>

namespace route {
    template<typename cell>
    class location: public std::reference_wrapper<cell> {
    public:
        using std::reference_wrapper<cell>::reference_wrapper;
        location(): std::reference_wrapper<cell>{dead} {}
        location(cell* address): std::reference_wrapper<cell>{*address} {}
        operator bool() const { return &this->get() != &dead; }
        bool operator==(location<cell> const& other) const {
            return &this->get() == &other.get();
        }
    private:
        static cell dead;
    };
    template <typename cell>
    cell location<cell>::dead;
    
    template <typename condition, typename A, typename B>
    using If = typename std::conditional<condition::value, A, B>::type;
    template <typename T>
    using strip = typename std::remove_reference<T>::type;
    
    template <typename cell>
    using safe = If<std::is_reference<cell>, const strip<cell>&, If<std::is_pointer<cell>, const typename std::remove_pointer<strip<cell>>::type*, const cell>>;
    template <typename cell>
    using value = If<std::is_reference<cell>, location<safe<strip<cell>>>, safe<cell>>;
}

template<typename T>
class std::hash<route::location<T>>: public std::hash<T*> {
public:
    using hash<T*>::hash;
    size_t operator() (route::location<T> const& value) const { return hash<T*>::operator()(&static_cast<T&>(value)); }
};

namespace route {

    template <typename C, typename A, typename H, typename D>
    std::vector<value<C>> plan(
                               C start, C goal, A adjacent, H heuristic,
                               D debug) {
        using std::vector; using std::unordered_map;
        using N = decltype(adjacent(start));
        using cost = decltype(heuristic(start, goal));
        using node = value<C>;
        class waypoint: public std::pair<node, cost> {
            public:
            using std::pair<node, cost>::pair;
            static bool matches(waypoint const& w, node test) { return w.first == test; }
        };
        class cost_list: public unordered_map<node, waypoint> {
        public:
            using unordered_map<node, waypoint>::unordered_map;
            // This allows a cost_list to compare two nodes by adding its own discovered cost to the included heuristic
            bool operator() (waypoint const& a, waypoint const& b) const {
                return a.second + limit(a.first) > b.second + limit(b.first);
            }
            class branch: public std::iterator <std::input_iterator_tag, node> {
            public:
                using tree = unordered_map<node, waypoint>;
                branch (tree const& source): complete {true}, stop {}, current { stop }, domain { source } {}
                branch (node start, node root, tree const& source): domain { source }, stop {root}, current { start } {
                    if (domain.count(current) == 0)
                        throw std::out_of_range{"state not found in search history"};
                }
                branch (branch const& other): domain { other.domain }, stop { other.stop }, current { other.current } {}
                bool operator!= (branch const& other) const {
                    return &domain != &other.domain
                    || complete != other.complete
                    || !(complete || current == other.current);
                }
                node& operator* () { return current; }
                node const& operator* () const { return current; }
                branch& operator++() {
                    if (stop == current) { complete = true; }
                    if (!complete) {
                        current = domain.at(current).first;
                    }
                    return *this;
                }
            private:
                node stop;
                node current;
                tree const& domain;
                bool complete = false;
            };
            branch follow(node leaf, node root) { return branch { leaf, root, *this }; }
            branch root() { return branch{*this}; }
        private:
            cost limit(node n) const { typedef std::numeric_limits<cost> limits; return this->count(n) > 0? this->at(n).second: limits::is_bounded? limits::max(): limits::infinity(); }
        };
        // actual A* implementation begins here
        
        // create a spanning tree and a heap containing the sgart node only
        cost_list costs = { {node{start}, {node{}, 0}} };
        vector<waypoint> candidates { {node{start}, heuristic(goal, start)} };
        // if no candidates are available, we have explored all paths without reaching the goal
        while (candidates.size() > 0) {
            // select the candidate with the lowest combined reach cost and heuristic to the goal
            pop_heap(candidates.begin(), candidates.end(), costs);
            waypoint next = candidates.back();
            candidates.pop_back();
            node prospect = next.first;
            // if this candidate is the goal, the search is complete; return the discovered path
            if (prospect == goal) {
                auto path = vector<node> (costs.follow(prospect, start), costs.root());
                reverse(path.begin(), path.end());
                return move(path);
            }
            // provide debug output if requested
            debug(vector<node>(costs.follow(prospect, start), costs.root()));
            // search all neighbors of the candidate to see if they represent new territory or a better route
            N neighbors = std::forward<N>(adjacent(prospect));
            for (auto const& neighbor: neighbors) {
                // calculate cost to reach this point from the start
                cost burden = neighbor.second + costs[prospect].second;
                node candidate {neighbor.first};
                if (costs.count(candidate) == 0 || costs[candidate].second > burden) {
                    // store discovered cost to reach this point
                    costs[candidate] = waypoint {prospect, burden};
                    // fix heap if a new or better cost has been discovered
                    auto begin = candidates.begin();
                    auto end = find_if(begin, candidates.end(), std::bind(&waypoint::matches, std::placeholders::_1, candidate));
                    auto end_position = end - begin;
                    if (end == candidates.end()) candidates.emplace_back(candidate, heuristic(goal, candidate));
                    begin = candidates.begin(); //iterators invalidated by change to vector
                    push_heap(begin, begin + end_position + 1, costs);
                }
            }
        }
        // no paths left if this point reached; return empty path
        return vector<node>{};
    }
    
    template <typename C, typename A, typename H>
    std::vector<value<C>> plan(C start, C goal, A adjacent, H heuristic) {
        return plan<C>(start, goal, adjacent, heuristic, [](vector<value<C>> const&) {});
    }
}
