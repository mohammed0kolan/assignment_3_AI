#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <tuple>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <string>
#include <iomanip>
#include <cstdlib>

using namespace std;

struct State {
    int row, col, air;

    bool operator<(const State& other) const {
        return tie(row, col, air) < tie(other.row, other.col, other.air);
    }
};

struct Node {
    State state;
    int cost;
    double estimate;
    int parent;
};

struct Result {
    bool found = false;
    int cost = 0;
    long expanded = 0;
    long generated = 0;
    long biggestFrontier = 0;
    double timeMs = 0;
    vector<State> path;
};

class Maze {
public:
    int rows, cols, maxAir;
    vector<vector<char>> grid;
    State start{0, 0, 0};
    int goalRow = -1;
    int goalCol = -1;

    Maze(const vector<vector<char>>& input, int air) {
        grid = input;
        rows = grid.size();
        cols = grid[0].size();
        maxAir = air;

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                if (grid[r][c] == 'S') {
                    start = {r, c, maxAir};
                    grid[r][c] = 'W';
                }
                if (grid[r][c] == 'X') {
                    goalRow = r;
                    goalCol = c;
                    grid[r][c] = 'W';
                }
            }
        }
    }

    bool isGoal(const State& s) const {
        return s.row == goalRow && s.col == goalCol;
    }

    double heuristic(const State& s, int hType) const {
        double dr = abs(s.row - goalRow);
        double dc = abs(s.col - goalCol);
        if (hType == 1) return sqrt(dr * dr + dc * dc);
        return dr + dc;
    }

    vector<State> getNextStates(const State& s) const {
        int dr[] = {-1, 1, 0, 0, 0};
        int dc[] = {0, 0, -1, 1, 0};
        vector<State> next;

        for (int i = 0; i < 5; i++) {
            int r = s.row + dr[i];
            int c = s.col + dc[i];

            if (r < 0 || c < 0 || r >= rows || c >= cols) continue;
            if (grid[r][c] == 'B') continue;

            int air = s.air;
            if (grid[r][c] == 'G') {
                air--;
                if (air < 0) continue;
            } else {
                air = maxAir;
            }
            next.push_back({r, c, air});
        }
        return next;
    }
};

Result search(const Maze& maze, string algorithm, int hType) {
    auto startTime = chrono::steady_clock::now();
    Result result;
    vector<Node> nodes;
    using Item = tuple<double, long, int>;
    priority_queue<Item, vector<Item>, greater<Item>> open;
    set<State> visited;
    long order = 0;

    auto priority = [&](const Node& n) {
        if (algorithm == "greedy") return n.estimate;
        return n.cost + n.estimate;
    };

    nodes.push_back({maze.start, 0, maze.heuristic(maze.start, hType), -1});
    open.push({priority(nodes[0]), order++, 0});
    result.generated++;

    while (!open.empty()) {
        result.biggestFrontier = max<long>(result.biggestFrontier, open.size());
        int index = get<2>(open.top());
        open.pop();

        Node current = nodes[index];
        if (visited.count(current.state)) continue;

        if (maze.isGoal(current.state)) {
            result.found = true;
            result.cost = current.cost;
            for (int i = index; i != -1; i = nodes[i].parent) {
                result.path.push_back(nodes[i].state);
            }
            reverse(result.path.begin(), result.path.end());
            break;
        }

        visited.insert(current.state);
        result.expanded++;

        vector<State> next = maze.getNextStates(current.state);
        for (const State& state : next) {
            if (visited.count(state)) continue;
            nodes.push_back({state, current.cost + 1,
                             maze.heuristic(state, hType), index});
            open.push({priority(nodes.back()), order++, (int)nodes.size() - 1});
            result.generated++;
        }
    }

    result.timeMs = chrono::duration<double, milli>(
        chrono::steady_clock::now() - startTime).count();
    return result;
}

int main(int argc, char** argv) {
    vector<vector<char>> grid = {
        {'B', 'B', 'B', 'B', 'B', 'B', 'B'},
        {'B', 'W', 'W', 'W', 'W', 'X', 'B'},
        {'B', 'B', 'G', 'B', 'B', 'B', 'B'},
        {'B', 'W', 'G', 'W', 'W', 'W', 'B'},
        {'B', 'B', 'G', 'B', 'W', 'B', 'B'},
        {'B', 'S', 'G', 'W', 'W', 'W', 'B'},
        {'B', 'B', 'B', 'B', 'B', 'B', 'B'}
    };

    int maxAir = argc > 1 ? atoi(argv[1]) : 3;
    Maze maze(grid, maxAir);
    if (argc >= 4) {
        maze.start = {atoi(argv[2]), atoi(argv[3]), maxAir};
    }

    cout << "Start=(" << maze.start.row << "," << maze.start.col << ") Goal=("
         << maze.goalRow << "," << maze.goalCol << ") A=" << maze.maxAir << "\n\n";
    cout << left << setw(10) << "Algo" << setw(5) << "h" << setw(8) << "Found"
         << setw(6) << "Cost" << setw(10) << "Expanded" << setw(11) << "Generated"
         << setw(9) << "MaxFront" << "Time(ms)\n";

    for (string algorithm : {"greedy", "astar"}) {
        for (int hType : {1, 2}) {
            Result result = search(maze, algorithm, hType);
            cout << left << setw(10) << algorithm << setw(5) << ("h" + to_string(hType))
                 << setw(8) << (result.found ? "yes" : "NO")
                 << setw(6) << (result.found ? to_string(result.cost) : "-")
                 << setw(10) << result.expanded << setw(11) << result.generated
                 << setw(9) << result.biggestFrontier << fixed << setprecision(3)
                 << result.timeMs << "\n";

            if (result.found) {
                cout << "   path: ";
                for (const State& s : result.path) {
                    cout << "(" << s.row << "," << s.col << ",a" << s.air << ") ";
                }
                cout << "\n";
            }
        }
    }
}
