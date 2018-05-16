#include <deque>
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <iostream>
#include <cmath>
#include <random>
#include "Entry.h"

#define N 4

std::vector<bool> map;
std::vector<int> visited;
std::vector<xyLoc> succ;
int width, height;

using namespace std;

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

void remove(state* table, BinaryHeap* pqs, state** S, xyLoc goal, state &m, int num, state* neighbors){
    if (pqs[num].empty()){
        return;
    }

    stateWithF min;
    do {
        min = pqs[num].remove();
    } while(!min.ptr->isOpen && !pqs[num].empty());
    if (!min.ptr->isOpen)
        return;

    if (min.ptr->node.x == goal.x && min.ptr->node.y == goal.y) {
        if (m.isNil() || min.ptr->f_value < m.f_value) {
            m = *(min.ptr);
        }
    }

    int numOfNeighbors = GetSuccessors_for_gastar(&table[min.ptr->hash()], neighbors, goal);
    for(int i= 0;i<numOfNeighbors; i++) {
        S[num][i] = neighbors[i];
    }
    return;
}

bool isAllQueueEmpty(BinaryHeap* pqs) {
    for(int i=0; i< N; i++){
        if(!pqs[i].empty()){
            return false;
        }
    }
    return true;
}

void duplicate_detection(state* table, BinaryHeap* pqs, state** S, int num){
    for(int i=0;i<8;i++) {
        state s = S[num][i];
        if(s.isNil()){
            return;
        }
        state old = table[s.hash()];
        if (!old.isNil() && old.g_value <= s.g_value) {
            continue;
        } else {
            std::random_device rd;
	        std::mt19937 mt(rd());
            int result = mt()%N;

            table[s.hash()] = s;
            stateWithF froms(&table[s.hash()]);
            pqs[result].add(froms);
        }
    }
    return;
}

bool GetPath_GASTAR(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path) {
    assert((long)data == 13182);
    BinaryHeap pqs[N];
    for(int i=0;i<N;i++){
        pqs[i].n = 0;
    }
    state initial(s,g);
    state nil(xyLoc(-1,-1), xyLoc(-1,-1));
    state m = nil;

    state* table = (state*)malloc(sizeof(state)*width*height);
    for(int i=0; i<width*height; i++){
        table[i] = nil;
    }
    table[initial.hash()] = initial;
    pqs[0].add(stateWithF(&table[initial.hash()]));

    if(path.size() > 0) {
        path.push_back(g);
        return true;
    }

    while(!isAllQueueEmpty(pqs)) {
        state**  S= (state**)malloc(sizeof(state*)*N);
        for (int i=0;i<N;i++) {
	        S[i] = (state*)malloc(sizeof(state)*8);
            for(int j=0;j<8;j++){
                S[i][j] = nil;
            }
        }
        state* neighbors = (state*)malloc(sizeof(state)*8);
        for(int j=0;j<8;j++){
            neighbors[j] = nil;
        }
        for (int i=0;i<N;i++){
            remove(table, pqs, S, g, m, i, neighbors);
            for(int j=0;j<8;j++){
                neighbors[j] = nil;
            }
        }
        bool pathFound = false;
        if (!m.isNil()) {
            for(int i=0; i< N; i++) {
                if (pqs[i].empty()){
                    continue;
                } else {
                    if(m.f_value > pqs[i].top().ptr->f_value)
                        break;
                }
            }
            pathFound = true;
        }
        if(pathFound){
            for(int i=0;i<N;i++){
                free(S[i]);
            }
            free(S);
            free(neighbors);
            break;
        }
        for (int i=0; i< N;i++){
            duplicate_detection(table, pqs, S, i);
        }

        for(int i=0;i<N;i++){
            free(S[i]);
        }
        free(S);
        free(neighbors);
    }

    state last = m;
    while(last.node.x != s.x || last.node.y != s.y) {
        xyLoc pos;
        pos.x = last.node.x;
        pos.y = last.node.y;
        path.push_back(pos);
        last = *last.parent;
    }
    path.push_back(s);
    reverse(path.begin(), path.end());
    free(table);

    // if(path.size() > 0) {
    //     path.pop_back();
    //     //XXX
    //     // return false;
    //     return true;
    // }
    return true;
}

int GetIndex(xyLoc s)
{
    return s.y*width+s.x;
}

state create_next_state(state &orig, xyLoc goal, Direction d) {
    state next = orig;
    next.parent = &orig;
    if(d%2 == 0) {
        next.g_value = orig.g_value + 1;
    } else {
        next.g_value = orig.g_value + 1.4142;
    }
    switch(d) {
    case UP:
        next.node.y--;
        break;
    case UPPERLEFT:
        next.node.x--;
        next.node.y--;
        break;
    case LEFT:
        next.node.x--;
        break;
    case LOWERLEFT:
        next.node.x--;
        next.node.y++;
        break;
    case DOWN:
        next.node.y++;
        break;
    case LOWERRIGHT:
        next.node.x++;
        next.node.y++;
        break;
    case RIGHT:
        next.node.x++;
        break;
    case UPPERRIGHT:
        next.node.x++;
        next.node.y--;
        break;
    }
    next.isOpen = true;
    next.f_value = next.g_value + next.octile(goal);
    return next;
}

// GetSuccessors_for_gastar is extarcting function
// @param s: the state will be extracted.
// @param neighbors: an array of pointers which points state*. It stores neighbors
// @param g: goal
// @return : the number of neighbors.
int GetSuccessors_for_gastar(state* s, state* neighbors, xyLoc g) {
    bool up = false, down = false, left = false, right = false;
    int i = 0;

    state next = create_next_state(*s,g,RIGHT);
    if (next.node.x < width && map[GetIndex(next.node)]) {
        neighbors[i] = next;
        i++;
        right = true;
    }
    next = create_next_state(*s,g,LEFT);
    if (next.node.x >= 0 && map[GetIndex(next.node)]) {
        neighbors[i] = next;
        i++;
        left = true;
    }
    next = create_next_state(*s,g,UP);
    if (next.node.y >= 0 && map[GetIndex(next.node)]) {
        neighbors[i] = next;
        i++;
        up = true;
    }
    next = create_next_state(*s,g,DOWN);
    if (next.node.y < height && map[GetIndex(next.node)]) {
        neighbors[i] = next;
        i++;
        down = true;
    }
    next = create_next_state(*s,g,LOWERRIGHT);
    if (next.node.y < height && next.node.x < width && map[GetIndex(next.node)] && right && down){
        neighbors[i] = next;
        i++;
    }
    next = create_next_state(*s,g,UPPERRIGHT);
    if (next.node.y >= 0 &&  next.node.x < width && map[GetIndex(next.node)] && right && up){
        neighbors[i] = next;
        i++;
    }
    next = create_next_state(*s,g,UPPERLEFT);
    if (next.node.y >= 0 && next.node.x >= 0 && map[GetIndex(next.node)] && left && up){
        neighbors[i] = next;
        i++;
    }
    next = create_next_state(*s,g,LOWERLEFT);
    if (next.node.y < height && next.node.x >= 0 && map[GetIndex(next.node)] && left && down){
        neighbors[i] = next;
        i++;
    }

    // for(int j=0;j<i;j++){
    //     cout<< neighbors[j].node.x << " " << neighbors[j].node.y << endl;
    // }
    return i;
}

