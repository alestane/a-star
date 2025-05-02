//
//  main.cpp
//  AStar
//
//  Created by Nevin Flanagan on 11/20/16.
//  Copyright © 2016 PlaySmith. All rights reserved.
//

#include <cstdlib>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <utility>
#include <map>
#include "astar.hpp"
using namespace std;

class Board: public vector <float> {
public:
    using routes = map<float const*, float>;
    unsigned int const width, height;
    size_type position, goal;
    Board (unsigned int w, unsigned int h): vector (w * h, 1.0f), width { w }, height { h } {}
    routes neighbors(float const* cell) const;
    bool contains(int h, int v) const { return h >= 0 && h < width && v >= 0 && v < height; }
    float manhattan(float const* target, float const* at) const;
private:
    size_type approve(unsigned int h, unsigned int v) const {
        if (contains(h, v)) return v * width + height;
        throw this;
    }
    pair<int, int> where(float const* what) const {
        ptrdiff_t position = what - data();
        if (position < 0 || position >= size()) throw this;
        return {position % width, position / width};
    }
};

Board::routes Board::neighbors(const float* cell) const {
    routes adjacent;
    auto position = where(cell);
    struct { int x, y; } offset = {1, 0};
    do {
        if (contains(offset.x + position.first, offset.y + position.second)) {
            float const* neighbor = cell + offset.y * static_cast<int>(width) + offset.x;
            adjacent[neighbor] = *neighbor;
        }
        offset.y = -offset.y; std::swap(offset.x, offset.y);
    } while (offset.x != 1);
    return adjacent;
}

float Board::manhattan(float const* target, float const* at) const {
    auto start = where(at), destination = where(target);
    return abs(start.first - destination.first) + abs(start.second - destination.second);
}

istream& operator>> (istream& source, Board& target) {
    auto row = target.begin();
    for (string line; getline(source, line) && line.size() > 0; row += target.width) {
        auto position = row; if (position >= target.end()) break;
        for (char cell: line.substr(0, target.width)) {
            switch (cell) {
                case '#':
                    *position = numeric_limits<float>::infinity();
                    break;
                case '.':
                    *position = 3.0f;
                    break;
                case '@':
                    target.position = position - target.begin();
                    break;
                case '$':
                    target.goal = position - target.begin();
                    break;
            }
            ++position;
        }
    }
    return source;
}

string display(Board const& source, vector<float const*> const& overlay) {
    ostringstream buffer;
    for (float const& cell: source) {
        ptrdiff_t position = &cell - source.data();
        if (find(overlay.begin(), overlay.end(), &cell) != overlay.end()) {
            buffer << (&cell == overlay.front()? '@': '*');
        } else if (position == source.position) {
            buffer << '@';
        } else if (position == source.goal) {
            buffer << '$';
        } else if (cell == numeric_limits<float>::infinity()) {
            buffer << '#';
        } else if (cell >= 2.0f) {
            buffer << '.';
        } else {
            buffer << ' ';
        }
        if ((position + 1) % source.width == 0) {
            buffer << '\n';
        }
    }
    return buffer.str();
}

int main(int argc, const char * argv[]) {
    Board world { 20, 10 };
    cin >> world;
    cin >> noskipws;
    {
        using namespace placeholders;
        auto path = route::plan(
                                &world[world.position], &world[world.goal],
                                // [&world](float const& cell) {return world.neighbors(cell);},
                                bind(&Board::neighbors, &world, _1),
                                // [&world](float const& to, float const& from){return world.manhattan(to, from);},
                                bind(&Board::manhattan, &world, _1, _2),
                                [&world](vector<float const*>const& path){cout << display(world, path); char input; cin >> input;}
        );
        cout << display(world, path);
    }
    return 0;
}
