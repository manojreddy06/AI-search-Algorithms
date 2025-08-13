#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <functional>
#include <limits>
using namespace std;

using Graph = unordered_map<int, vector<int>>;

void print_path(const vector<int>& path) {
    for (int node : path) cout << node << " ";
    cout << endl;
}

bool british_museum_search(const Graph& g, int start, int goal, vector<int>& path, vector<int>& result) {
    path.push_back(start);
    if (start == goal) {
        result = path;
        return true;
    }
    for (int neighbor : g.at(start)) {
        if (find(path.begin(), path.end(), neighbor) == path.end()) {
            if (british_museum_search(g, neighbor, goal, path, result)) return true;
        }
    }
    path.pop_back();
    return false;
}

bool dfs(const Graph& g, int start, int goal, vector<int>& path) {
    stack<int> s;
    unordered_map<int, int> parent;
    s.push(start);
    parent[start] = -1;
    while (!s.empty()) {
        int node = s.top(); s.pop();
        if (node == goal) {
            while (node != -1) {
                path.push_back(node);
                node = parent[node];
            }
            reverse(path.begin(), path.end());
            return true;
        }
        for (int neighbor : g.at(node)) {
            if (parent.find(neighbor) == parent.end()) {
                s.push(neighbor);
                parent[neighbor] = node;
            }
        }
    }
    return false;
}

bool bfs(const Graph& g, int start, int goal, vector<int>& path) {
    queue<int> q;
    unordered_map<int, int> parent;
    q.push(start);
    parent[start] = -1;
    while (!q.empty()) {
        int node = q.front(); q.pop();
        if (node == goal) {
            while (node != -1) {
                path.push_back(node);
                node = parent[node];
            }
            reverse(path.begin(), path.end());
            return true;
        }
        for (int neighbor : g.at(node)) {
            if (parent.find(neighbor) == parent.end()) {
                q.push(neighbor);
                parent[neighbor] = node;
            }
        }
    }
    return false;
}

bool bfs_dfs_hybrid(const Graph& g, int start, int goal, vector<int>& path) {
    deque<int> dq;
    unordered_map<int, int> parent;
    dq.push_back(start);
    parent[start] = -1;
    bool use_bfs = true;
    while (!dq.empty()) {
        int node;
        if (use_bfs) { node = dq.front(); dq.pop_front(); }
        else { node = dq.back(); dq.pop_back(); }
        if (node == goal) {
            while (node != -1) {
                path.push_back(node);
                node = parent[node];
            }
            reverse(path.begin(), path.end());
            return true;
        }
        for (int neighbor : g.at(node)) {
            if (parent.find(neighbor) == parent.end()) {
                dq.push_back(neighbor);
                parent[neighbor] = node;
            }
        }
        use_bfs = !use_bfs;
    }
    return false;
}

bool bfs_with_history(const Graph& g, int start, int goal, vector<int>& path) {
    queue<int> q;
    unordered_map<int, int> parent;
    unordered_set<int> visited;
    q.push(start);
    parent[start] = -1;
    visited.insert(start);
    while (!q.empty()) {
        int node = q.front(); q.pop();
        if (node == goal) {
            while (node != -1) {
                path.push_back(node);
                node = parent[node];
            }
            reverse(path.begin(), path.end());
            return true;
        }
        for (int neighbor : g.at(node)) {
            if (!visited.count(neighbor)) {
                q.push(neighbor);
                parent[neighbor] = node;
                visited.insert(neighbor);
            }
        }
    }
    return false;
}

bool dfs_with_history(const Graph& g, int start, int goal, vector<int>& path) {
    stack<int> s;
    unordered_map<int, int> parent;
    unordered_set<int> visited;
    s.push(start);
    parent[start] = -1;
    visited.insert(start);
    while (!s.empty()) {
        int node = s.top(); s.pop();
        if (node == goal) {
            while (node != -1) {
                path.push_back(node);
                node = parent[node];
            }
            reverse(path.begin(), path.end());
            return true;
        }
        for (int neighbor : g.at(node)) {
            if (!visited.count(neighbor)) {
                s.push(neighbor);
                parent[neighbor] = node;
                visited.insert(neighbor);
            }
        }
    }
    return false;
}

bool hill_climbing(const Graph& g, int start, int goal, function<int(int)> heuristic, vector<int>& path) {
    int node = start;
    unordered_set<int> visited;
    path.push_back(node);
    while (node != goal) {
        visited.insert(node);
        vector<pair<int, int>> neighbors;
        for (int neighbor : g.at(node)) {
            if (!visited.count(neighbor))
                neighbors.push_back({heuristic(neighbor), neighbor});
        }
        if (neighbors.empty()) return false;
        sort(neighbors.begin(), neighbors.end());
        node = neighbors[0].second;
        path.push_back(node);
    }
    return true;
}

bool hill_climbing_with_history(const Graph& g, int start, int goal, function<int(int)> heuristic, vector<int>& path) {
    return hill_climbing(g, start, goal, heuristic, path);
}

bool beam_search(const Graph& g, int start, int goal, function<int(int)> heuristic, int beam_width, vector<int>& path) {
    vector<vector<int>> beams = {{start}};
    while (!beams.empty()) {
        vector<vector<int>> next_beams;
        for (auto& b : beams) {
            int node = b.back();
            if (node == goal) {
                path = b;
                return true;
            }
            for (int neighbor : g.at(node)) {
                if (find(b.begin(), b.end(), neighbor) == b.end()) {
                    auto new_path = b;
                    new_path.push_back(neighbor);
                    next_beams.push_back(new_path);
                }
            }
        }
        sort(next_beams.begin(), next_beams.end(), [&](const vector<int>& a, const vector<int>& b) {
            return heuristic(a.back()) < heuristic(b.back());
        });
        if (next_beams.size() > beam_width)
            next_beams.resize(beam_width);
        beams = next_beams;
    }
    return false;
}

bool beam_search_with_history(const Graph& g, int start, int goal, function<int(int)> heuristic, int beam_width, vector<int>& path) {
    vector<vector<int>> beams = {{start}};
    unordered_set<int> visited;
    visited.insert(start);
    while (!beams.empty()) {
        vector<vector<int>> next_beams;
        for (auto& b : beams) {
            int node = b.back();
            if (node == goal) {
                path = b;
                return true;
            }
            for (int neighbor : g.at(node)) {
                if (!visited.count(neighbor)) {
                    auto new_path = b;
                    new_path.push_back(neighbor);
                    next_beams.push_back(new_path);
                    visited.insert(neighbor);
                }
            }
        }
        sort(next_beams.begin(), next_beams.end(), [&](const vector<int>& a, const vector<int>& b) {
            return heuristic(a.back()) < heuristic(b.back());
        });
        if (next_beams.size() > beam_width)
            next_beams.resize(beam_width);
        beams = next_beams;
    }
    return false;
}

bool oracle_search(const vector<int>& oracle_path, int goal, vector<int>& path) {
    for (int node : oracle_path) {
        path.push_back(node);
        if (node == goal) return true;
    }
    return false;
}

bool branch_and_bound(const Graph& g, int start, int goal, unordered_map<pair<int,int>, int, hash<pair<int,int>>> edge_cost, vector<int>& path) {
    using State = pair<int, vector<int>>;
    auto cmp = [](const State& a, const State& b) { return a.first > b.first; };
    priority_queue<State, vector<State>, decltype(cmp)> pq(cmp);
    pq.push({0, {start}});
    unordered_set<int> visited;
    while (!pq.empty()) {
        auto [cost, p] = pq.top(); pq.pop();
        int node = p.back();
        if (node == goal) {
            path = p;
            return true;
        }
        if (visited.count(node)) continue;
        visited.insert(node);
        for (int neighbor : g.at(node)) {
            if (!visited.count(neighbor)) {
                auto new_path = p;
                new_path.push_back(neighbor);
                int new_cost = cost + edge_cost[{node, neighbor}];
                pq.push({new_cost, new_path});
            }
        }
    }
    return false;
}

bool branch_and_bound_heuristic(const Graph& g, int start, int goal, unordered_map<pair<int,int>, int, hash<pair<int,int>>> edge_cost, function<int(int)> heuristic, vector<int>& path) {
    using State = tuple<int, int, vector<int>>;
    auto cmp = [](const State& a, const State& b) { return get<0>(a) > get<0>(b); };
    priority_queue<State, vector<State>, decltype(cmp)> pq(cmp);
    pq.push({heuristic(start), 0, {start}});
    unordered_set<int> visited;
    while (!pq.empty()) {
        auto [est, cost, p] = pq.top(); pq.pop();
        int node = p.back();
        if (node == goal) {
            path = p;
            return true;
        }
        if (visited.count(node)) continue;
        visited.insert(node);
        for (int neighbor : g.at(node)) {
            if (!visited.count(neighbor)) {
                auto new_path = p;
                new_path.push_back(neighbor);
                int new_cost = cost + edge_cost[{node, neighbor}];
                pq.push({new_cost + heuristic(neighbor), new_cost, new_path});
            }
        }
    }
    return false;
}

bool a_star(const Graph& g, int start, int goal, unordered_map<pair<int,int>, int, hash<pair<int,int>>> edge_cost, function<int(int)> heuristic, vector<int>& path) {
    return branch_and_bound_heuristic(g, start, goal, edge_cost, heuristic, path);
}

int main() {
    Graph g = {
        {1, {2,3}},
        {2, {4,5}},
        {3, {6}},
        {4, {}},
        {5, {6}},
        {6, {}}
    };
    unordered_map<pair<int,int>, int, hash<pair<int,int>>> edge_cost;
    edge_cost[{1,2}] = 1; edge_cost[{1,3}] = 2;
    edge_cost[{2,4}] = 2; edge_cost[{2,5}] = 2;
    edge_cost[{3,6}] = 3; edge_cost[{5,6}] = 1;

    auto heuristic = [&](int node) { return abs(6 - node); };

    vector<int> path;
    if (bfs(g, 1, 6, path)) {
        cout << "BFS Path: "; print_path(path);
    }
    return 0;
}
