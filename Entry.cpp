#include <deque>
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <iostream>
#include <cmath>
#include "BinaryHeap.h"

#define N 512

std::vector<bool> map;
std::vector<int> visited;
std::vector<xyLoc> succ;
int width, height;

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

// TODO
void remove(BinaryHeap* pqs, std::vector<std::vector<state*> >S, /*lengthForS,*/ xyLoc g, state m);
// void remove(thrust::device_vector<state *> arrays[], int *lengths, thrust::device_vector<state *> S[], int *lengthForS, xyLoc goal, state** candidate){
//     int num = threadIdx.x;
//     state** array = RAW(arrays[num]);
//     /* state** array = arrays[num]; */
//     int len = lengths[num];
//
//     BinaryHeap pq = BinaryHeap(array, len);
//     if (pq.empty()) {
//         return;
//     }
//     state* min = pq.remove();
//     if (min->node.x == goal.x && min->node.y == goal.y) {
//         if (candidate[0]->isNil() || min->f_value < candidate[0]->f_value) {
//             candidate[0] = min;
//         }
//     }
//
//     lengthForS[num] = GetSuccessors_for_gastar(min, RAW(S[num]));
//     /* lengthForS[num] = GetSuccessors_for_gastar(min, S[num]); */
//     return;
// }

bool isAllQueueEmpty(BinaryHeap* pqs) {
    for(int i=0; i< N; i++){
        if(!pqs[i].empty())
            return false;
    }
    return true;
}

// TODO
// void duplicate_detection(state** table,thrust::device_vector<state *, thrust::device_malloc_allocator<state *>> pqs[], int *lengths, thrust::device_vector<state *,
//         thrust::device_malloc_allocator<state *>> S[], int *lengthForS){
//     int x = threadIdx.x;
//     for(int i=0; i< lengthForS[x]; i++) {
//         state* s = S[x][i];
//         state* old = table[s->hash()];
//         if (!old->isNil() && old->g_value < s->g_value) {
//             return;
//         } else {
//             /* s->f_value = s->f_value */
//             curandState_t state;
//             curand_init(0,0,0, &state);
//             int result = curand(&state) %N;
//             BinaryHeap pq = BinaryHeap(RAW(pqs[result]), lengths[result]);
//             pq.add(s);
//             lengths[result] += 1;
//         }
//     }
// }

bool GetPath_GASTAR(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path) {
    assert((long)data == 13182);
    BinaryHeap pqs[N];
    state initial(s,g);
    state m;
    pqs[0].add(stateWithF(&initial));

    // TODO: prepare enough size(not N)
    state* table[N];

    if(path.size() > 0) {
        path.push_back(g);
        return true;
    }

    while(!isAllQueueEmpty(pqs)) {
        std::vector<std::vector<state*> > S(N);
        for(int i=0;i<N;i++) {
            std::vector<state*> tmp;
            S[i] = tmp;
        }
        // 8個の要素を持つ配列、の配列をSとする
        // lengthの代わりに、illegalな値をセットしてチェックをするようにする

        //TODO: for loop
        remove(pqs, S, g, m);

        bool flag = true;
        if (m.isNil()) {
            for(int i=0; i< N; i++) {
                if (pqs[i].empty()){
                    continue;
                } else {
                    if(m.f_value > pqs[i].top().ptr->f_value)
                        flag = false;
                    break;
                }
            }
        }
        if(flag){
            break;
        }
        duplicate_detection(table, pqs, S);
        // duplicate_detection<<<1,N>>>(RAW(table), pqs, RAW(lengths), S, RAW(lengthForS));
    }

    // TODO: must change
    /* while(last.x != s.x || last.y != s.y) { */
    /*     xyLoc pos;                          */
    /*     pos.x = last.x;                     */
    /*     pos.y = last.y;                     */
    /*     path.push_back(pos);                */
    /*     last = *last.parent;                */
    /* }                                       */
    /* path.push_back(s); */

    if(path.size() > 0) {
        path.pop_back();
        return false;
    }
    return true;
}

int GetIndex(xyLoc s)
{
    return s.y*width+s.x;
}

int GetSuccessors_for_gastar(state* s, state** neighbors) {
    return 0;
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

