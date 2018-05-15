#include <deque>
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <iostream>
#include <cmath>
#include "Entry.h"

std::vector<bool> map;
std::vector<int> visited;
std::vector<xyLoc> succ;
int width, height;
int expanded;

const char *GetName()
{
    return "TestProgram";
}

void PreprocessMap(std::vector<bool> &bits, int width, int height, const char *filename)
{
    printf("Not writing to file '%s'\n", filename);
}

void *PrepareForSearch(std::vector<bool> &bits, int w, int h, const char *filename)
{
    printf("Not reading from file '%s'\n", filename);
    map = bits;
    width = w;
    height = h;
    return (void *)13182;
}

std::vector<node*> cache;
bool GetPath_ASTAR(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path,
                   std::vector<xyWithFGH> &expanded_nodes, std::function<void(node &, std::vector<node> &, xyLoc)> get_successors) {
    expanded = 0;
    assert((long)data == 13182);
    std::priority_queue<state, std::vector<state>, std::greater<state> > q;
    std::unordered_map<int, node> table;
    std::vector<node> successors;
    cache.resize(0);
    if(path.size() > 0) {
        path.push_back(g);
        return true;
    }
    node start_node(s);
    table[s.hash()] = start_node;
    state start_state(s.hash(), start_node.minimum, 0);
    q.push(start_state);

    int goal_id = g.hash();
    state next;

    while(q.size() > 0) {
        next = q.top();
        if(goal_id == next.id) {
            break;
        }
        q.pop();

        node next_node = table[next.id];
        if(!next_node.isOpen) {
            continue;
        }
        table[next.id].isOpen = false;

        xyWithFGH n(next_node, g);
        expanded_nodes.push_back(n);
        expanded++;

        get_successors(table[next.id], successors, g);
        for(auto it = successors.begin(); it != successors.end(); it++) {
            if(table.find(it->hash()) != table.end()) {
                if(table[it->hash()].minimum > it->minimum) {
                    table[it->hash()] = *it;
                    next = state(it->hash(), it->g + it->octile(g),it->g);
                    q.push(next);
                }
            } else {
                table[it->hash()] = *it;
                next = state(it->hash(), it->g + it->octile(g), it->g);
                q.push(next);
            }
        }
    }
    node last = table[next.id];

    while(last.x != s.x || last.y != s.y) {
        xyLoc pos;
        pos.x = last.x;
        pos.y = last.y;
        path.push_back(pos);
        last = *last.parent;
    }
    path.push_back(s);
    std::reverse(path.begin(), path.end());

    for(auto it = cache.begin(); it != cache.end(); it++) {
        delete *it;
    }

    if(path.size() > 0) {
        path.pop_back();
        return false;
    }
    return true;
}

bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path, std::vector<xyWithFGH> &expanded_nodes)
{
    expanded = 0;
    assert((long)data == 13182);
    std::deque<xyLoc> q;

    if (path.size() > 0)
    {
        path.push_back(g);
        return true;
    }
    visited.assign(map.size(),0);
    visited[GetIndex(s)] = 1;
    q.push_back(s);

    //BFS
    while (q.size() > 0)
    {
        xyLoc next = q.front();
        xyWithFGH next_node = xyWithFGH();
        double dist = sqrt((next.x - g.x)*(next.x - g.x)+(next.y-g.y)*(next.y-g.y));
        next_node.x = next.x, next_node.y = next.y, next_node.f = 0, next_node.g = 0, next_node.h = dist;
        expanded_nodes.push_back(next_node);
        expanded++;

        q.pop_front();
        GetSuccessors(next, succ);
        for (unsigned int x = 0; x < succ.size(); x++)
        {
            if (visited[GetIndex(succ[x])])
                continue;
            visited[GetIndex(succ[x])] = visited[GetIndex(next)]+1;

            if (succ[x].x == g.x && succ[x].y == g.y) // goal found
            {
                ExtractPath(g, path);
                if (path.size() > 0)
                {
                    path.pop_back();
                    return false;
                }
                return true; // empty path
            }

            q.push_back(succ[x]);
        }
    }
    return true; // no path returned, but we're done
}

int GetIndex(xyLoc s)
{
    return s.y*width+s.x;
}

node create_next_node(node &orig, xyLoc goal, Direction d) {
    node next = orig;
    next.parent = &orig;
    if(d%2 == 0) {
        next.g = orig.g + 1;
    } else {
        next.g = orig.g + 1.4142;
    }
    switch(d) {
    case UP:
        next.y--;
        break;
    case UPPERLEFT:
        next.x--;
        next.y--;
        break;
    case LEFT:
        next.x--;
        break;
    case LOWERLEFT:
        next.x--;
        next.y++;
        break;
    case DOWN:
        next.y++;
        break;
    case LOWERRIGHT:
        next.x++;
        next.y++;
        break;
    case RIGHT:
        next.x++;
        break;
    case UPPERRIGHT:
        next.x++;
        next.y--;
        break;
    }
    next.isOpen = true;
    next.minimum = next.g + next.octile(goal);
    return next;
}

Direction calcDirection(node &s) {
    if(s.parent->x == s.x && s.parent->y == s.y + 1)
        return UP;
    else if(s.parent->x == s.x + 1 && s.parent->y == s.y + 1)
        return UPPERLEFT;
    else if(s.parent->x == s.x + 1 && s.parent->y == s.y)
        return LEFT;
    else if(s.parent->x == s.x + 1 && s.parent->y == s.y - 1)
        return LOWERLEFT;
    else if(s.parent->x == s.x && s.parent->y == s.y - 1)
        return DOWN;
    else if(s.parent->x == s.x - 1 && s.parent->y == s.y - 1)
        return LOWERRIGHT;
    else if(s.parent->x == s.x - 1 && s.parent->y == s.y)
        return RIGHT;
    else
        return UPPERRIGHT;
}

bool isValidNode(node n) {
    return 0<= n.x && n.x < width && 0<= n.y && n.y < height && map[GetIndex(n)];
}

bool isObstacle(node n) {
    return 0<= n.x && n.x < width && 0<= n.y && n.y < height && !map[GetIndex(n)];
}

void add_next_if_valid(node& n, xyLoc g, Direction d, std::vector<node>& v) {
    node next = create_next_node(n, g, d);
    if(isValidNode(next))
        v.push_back(next);
}

std::vector<node> prune(node &current, xyLoc g) {
    std::vector<node> neighbors;
    if(current.parent == NULL) {
        GetSuccessors_for_astar(current, neighbors, g);
        return neighbors;
    }

    Direction d = calcDirection(current);
    node n = node();
    switch(d) {
    case UP:
        add_next_if_valid(current, g, UP, neighbors);

        n = create_next_node(current,g,LOWERLEFT);
        if(isObstacle(n)) {
            add_next_if_valid(current, g, UPPERLEFT, neighbors);
            add_next_if_valid(current, g, LEFT, neighbors);
        }
        n = create_next_node(current,g,LOWERRIGHT);
        if(isObstacle(n)) {
            add_next_if_valid(current, g, UPPERRIGHT, neighbors);
            add_next_if_valid(current, g, RIGHT, neighbors);
        }
        break;
    case UPPERLEFT:
        add_next_if_valid(current, g, UP, neighbors);
        add_next_if_valid(current, g, LEFT, neighbors);
        add_next_if_valid(current, g, UPPERLEFT, neighbors);
        break;

    case LEFT:
        add_next_if_valid(current, g, LEFT, neighbors);

        n = create_next_node(current,g,UPPERRIGHT);
        if(isObstacle(n)) {
            add_next_if_valid(current, g, UPPERLEFT, neighbors);
            add_next_if_valid(current, g, UP, neighbors);
        }
        n = create_next_node(current,g,LOWERRIGHT);
        if(isObstacle(n)) {
            add_next_if_valid(current, g, LOWERLEFT, neighbors);
            add_next_if_valid(current, g, DOWN, neighbors);
        }
        break;
    case LOWERLEFT:
        add_next_if_valid(current, g, DOWN, neighbors);
        add_next_if_valid(current, g, LEFT, neighbors);
        add_next_if_valid(current, g, LOWERLEFT, neighbors);
        break;
    case DOWN:
        add_next_if_valid(current, g, DOWN, neighbors);

        n = create_next_node(current,g,UPPERLEFT);
        if(isObstacle(n)) {
            add_next_if_valid(current, g, LOWERLEFT, neighbors);
            add_next_if_valid(current, g, LEFT, neighbors);
        }
        n = create_next_node(current,g,UPPERRIGHT);
        if(isObstacle(n)) {
            add_next_if_valid(current, g, LOWERRIGHT, neighbors);
            add_next_if_valid(current, g, RIGHT, neighbors);
        }
        break;
    case LOWERRIGHT:
        add_next_if_valid(current, g, DOWN, neighbors);
        add_next_if_valid(current, g, RIGHT, neighbors);
        add_next_if_valid(current, g, LOWERRIGHT, neighbors);
        break;
    case RIGHT:
        add_next_if_valid(current, g, RIGHT, neighbors);

        n = create_next_node(current,g,UPPERLEFT);
        if(isObstacle(n)) {
            add_next_if_valid(current, g, UPPERRIGHT, neighbors);
            add_next_if_valid(current, g, UP, neighbors);
        }
        n = create_next_node(current,g,LOWERLEFT);
        if(isObstacle(n)) {
            add_next_if_valid(current, g, LOWERRIGHT, neighbors);
            add_next_if_valid(current, g, DOWN, neighbors);
        }
        break;
    case UPPERRIGHT:
        add_next_if_valid(current, g, UP, neighbors);
        add_next_if_valid(current, g, RIGHT, neighbors);
        add_next_if_valid(current, g, UPPERRIGHT, neighbors);
        break;
    }
    return neighbors;
}

node jump(node &current, Direction d, xyLoc g) {
    if(d%2 == 1) {
        Direction d1 = Direction(d-1);
        Direction d2 = (d+1==8) ? Direction(0) : Direction(d+1);
        if(!isValidNode(create_next_node(current,g,d1)) || !isValidNode(create_next_node(current,g,d2))) {
            return node(-1, -1);
        }
    }
    node *n = new node();
    *n = create_next_node(current, g, d);
    cache.push_back(n);

    if (!isValidNode(*n)) {
        return node(-1,-1);
    }
    if (n->x == g.x && n->y == g.y) {
        return *n;
    }
    if(are_successors_forced(current, *n, d, g)) {
        return *n;
    }
    if(d%2 == 1) {
        Direction d1 = Direction(d-1);
        Direction d2 = (d+1==8) ? Direction(0) : Direction(d+1);
        if(!jump(*n,d1,g).isNULL()) {
            return *n;
        }
        if(!jump(*n,d2,g).isNULL()) {
            return *n;
        }
    }
    return jump(*n, d, g);
}

bool are_successors_forced(node px, node x, Direction d, xyLoc g) {
    if (d%2 == 1)
        return false;
    int neighbor_size = prune(x,g).size();
    if (neighbor_size > 1 || (neighbor_size == 1 && !isValidNode(create_next_node(x,g,d))))
        return true;
    return false;
}

void GetSuccessors_for_jps(node &s, std::vector<node> &successors, xyLoc g) {
    successors.resize(0);
    std::vector<node> neighbors;
    neighbors = prune(s, g);
    for(auto it = neighbors.begin(); it != neighbors.end(); it++) {
        Direction d = calcDirection(*it);
        node n = jump(s, d, g);
        if(!n.isNULL()) {
            successors.push_back(n);
        }
    }
    return;
}

void GetSuccessors_for_astar(node &s, std::vector<node> &neighbors, xyLoc g) {
    bool up = false, down = false, left = false, right = false;
    neighbors.resize(0);

    node next = create_next_node(s,g,RIGHT);
    if (next.x < width && map[GetIndex(next)]) {
        neighbors.push_back(next);
        right = true;
    }
    next = create_next_node(s,g,LEFT);
    if (next.x >= 0 && map[GetIndex(next)]) {
        neighbors.push_back(next);
        left = true;
    }
    next = create_next_node(s,g,UP);
    if (next.y >= 0 && map[GetIndex(next)]) {
        neighbors.push_back(next);
        up = true;
    }
    next = create_next_node(s,g,DOWN);
    if (next.y < height && map[GetIndex(next)]) {
        neighbors.push_back(next);
        down = true;
    }
    next = create_next_node(s,g,LOWERRIGHT);
    if (next.y < height && next.x < width && map[GetIndex(next)] && right && down)
        neighbors.push_back(next);
    next = create_next_node(s,g,UPPERRIGHT);
    if (next.y >= 0 &&  next.x < width && map[GetIndex(next)] && right && up)
        neighbors.push_back(next);
    next = create_next_node(s,g,UPPERLEFT);
    if (next.y >= 0 && next.x >= 0 && map[GetIndex(next)] && left && up)
        neighbors.push_back(next);
    next = create_next_node(s,g,LOWERLEFT);
    if (next.y < height && next.x >= 0 && map[GetIndex(next)] && left && down)
        neighbors.push_back(next);
}

// generates 4-connected neighbors
// doesn't generate 8-connected neighbors (which ARE allowed)
// a diagonal move must have both cardinal neighbors free to be legal
void GetSuccessors(xyLoc s, std::vector<xyLoc> &neighbors)
{
    neighbors.resize(0);

    xyLoc next = s;
    next.x++;
    // map[index] is true only when index is "G"||"S"||"."
    if (next.x < width && map[GetIndex(next)])
        neighbors.push_back(next);

    next = s;
    next.x--;
    if (next.x >= 0 && map[GetIndex(next)])
        neighbors.push_back(next);

    next = s;
    next.y--;
    if (next.y >= 0 && map[GetIndex(next)])
        neighbors.push_back(next);

    next = s;
    next.y++;
    if (next.y < height && map[GetIndex(next)])
        neighbors.push_back(next);
}

// make plan
// trace path by visited
void ExtractPath(xyLoc end, std::vector<xyLoc> &finalPath)
{
    int currCost = visited[GetIndex(end)];

    finalPath.resize(0);
    finalPath.push_back(end);

    while (currCost != 1)
    {
        GetSuccessors(finalPath.back(), succ);
        for (unsigned int x = 0; x < succ.size(); x++)
        {
            if (visited[GetIndex(succ[x])] == currCost-1)
            {
                finalPath.push_back(succ[x]);
                currCost--;
                break;
            }
        }
    }
    std::reverse(finalPath.begin(), finalPath.end());
}
