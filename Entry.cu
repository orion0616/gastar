#include <deque>
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <iostream>
#include <cmath>
#include <random>
#include <curand_kernel.h>
#include "Entry.h"

#define N 64
#define M 500
#define MAX 2000000

#define CHECK(call)                                                          \
{                                                                            \
    const cudaError_t error = call;                                          \
    if (error!=cudaSuccess){                                                 \
        printf("Error: %s:%d, ", __FILE__, __LINE__);                        \
        printf("code:%d, reason: %s\n", error, cudaGetErrorString(error));   \
        exit(1);                                                             \
    }                                                                        \
}

std::vector<bool> map;
std::vector<int> visited;
std::vector<xyLoc> succ;
int h_width, h_height;
__constant__ state d_initial;
__constant__ state d_nil;
__constant__ state d_m;
__device__ int id;
__device__ bool d_pathFound;

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
    h_width = w;
    h_height = h;
    CHECK(cudaMemcpyToSymbol(width, &h_width, sizeof(int)));
    CHECK(cudaMemcpyToSymbol(height, &h_height, sizeof(int)));

    state* d_table;
    CHECK(cudaMalloc((state**)&d_table, MAX*sizeof(state)));

    return (void *)d_table;

    /* return (void *)13182; */
}

__device__ void Rand(unsigned int* randx) {
    int num = threadIdx.x;
    randx[num] = (randx[num]*1103515245+12345)&2147483647;
    return;
}

__global__ void remove(state* table, int* lengths, stateWithF* array, state* S, xyLoc goal, state* neighbors, bool* d_map, state* kouho, int* closed_list){
    int num = threadIdx.x;
    stateWithF elem[M];
    for(int i=0;i<M;i++){
        elem[i] = array[num*M+i];
    }
    BinaryHeap pq = BinaryHeap(elem,lengths[num]);
    if (pq.empty()){
        return;
    }

    stateWithF min;
    do {
        min = pq.remove();
    } while(!min.ptr->isOpen && !pq.empty());
    lengths[num] = pq.size();
    for(int i=0;i<M;i++){
        array[num*M+i] = pq.a[i];
    }
    if (!min.ptr->isOpen)
        return;

    min.ptr->isOpen = false;
    if (min.ptr->node.x == goal.x && min.ptr->node.y == goal.y) {
        if (d_m.isNil() || min.ptr->f_value < d_m.f_value) {
            kouho[num] = *(min.ptr);
        }
    }

    int numOfNeighbors = GetSuccessors_for_gastar(&table[closed_list[min.ptr->hash()]], neighbors, num, goal, d_map);
    for(int i= 0;i<numOfNeighbors; i++) {
        S[num*8+i]= neighbors[num*8+i];
    }
    return;
}

bool isAllQueueEmpty(int* lengths) {
    for(int i=0; i< N; i++){
        if(lengths[i] != 0){
            return false;
        }
    }
    return true;
}

__global__ void duplicate_detection(state* table, int *lengths, stateWithF* array, state* S, int* closed_list){
    int num = threadIdx.x;
    int next = (num + 1)%N;

    for(int i=0;i<8;i++) {
        state s = S[num*8+i];
        if(s.isNil()){
            return;
        }
        int myid = atomicAdd(&id, 1);
        table[myid] = s;

        int old = closed_list[s.hash()];

        while(old == -1 || table[old].f_value > s.f_value){
            atomicCAS(&closed_list[s.hash()], old, myid);
            old = closed_list[s.hash()];
        }

        if (closed_list[s.hash()] == myid){
            stateWithF froms(&table[myid]);
            stateWithF tmp[M];
            for(int k=0;k<M;k++){
                tmp[k] = array[k+M*next];
            }
            BinaryHeap pq = BinaryHeap(tmp, lengths[next]);

            pq.add(froms);
            lengths[next] = pq.n;

            for(int k=0;k<M;k++){
                array[k+M*next] = pq.a[k];
            }
        }

    }
    return;
}


__global__ void init(state* table, int* lengths, stateWithF* array) {
    table[d_initial.hash()] = d_initial;
    stateWithF swf = stateWithF(&table[d_initial.hash()]);
    stateWithF tmp[M];
    for(int i=0;i<M;i++){
        tmp[i] = array[i];
    }
    BinaryHeap pq = BinaryHeap(tmp, lengths[0]);
    pq.add(swf);
    lengths[0]++;
    for(int i=0;i<M;i++){
        array[i] = tmp[i];
    }
}

__global__ void path_create(xyLoc* d_path, xyLoc s){
    for(int i=0;i<1000;i++){
        d_path[i] = xyLoc(-1,-1);
    }
    state last = d_m;
    int i = 0;
    while(last.node.x != s.x || last.node.y != s.y) {
        xyLoc pos;
        pos.x = last.node.x;
        pos.y = last.node.y;
        d_path[i] = pos;
        i++;
        last = *last.parent;
    }
}

__global__ void tmp_memory_reset(state *s, state *neighbors) {
    int num = threadIdx.x;
    s[num] = d_nil;
    neighbors[num] = d_nil;
    return;
}

__global__ void reset_kouho(state* kouho){
    int num = threadIdx.x;
    kouho[num] = d_nil;
}

__global__ void init_dtable(state * d_table){
    int num = threadIdx.x;
    int chunk = MAX/N;
    for(int i=num*chunk; i< num*chunk+num;i++){
        d_table[i] = d_nil;
    }
    if(num == 0){
        d_table[0] = d_initial;
    }
}

__global__ void checkIfPathIsFound(stateWithF* d_array){
    int num = threadIdx.x;
    d_pathFound = true;
    if(d_m.f_value >= d_array[M*num].fWhenInserted){
        d_pathFound = false;
    }
}


bool GetPath_GASTAR(void *d_table, xyLoc s, xyLoc g, std::vector<xyLoc> &path) {
    int h_id = 1;
    CHECK(cudaMemcpyToSymbol(id, &h_id, sizeof(int)));

    int mapsize = map.size();
    bool h_map[mapsize];
    bool *d_map;
    CHECK(cudaMalloc((bool**)&d_map, mapsize*sizeof(bool)));
    for(int i=0;i<mapsize;i++){
    h_map[i] = map[i];
    }
    CHECK(cudaMemcpy(d_map, h_map, mapsize*sizeof(bool), cudaMemcpyHostToDevice));


    BinaryHeap pqs[N];
    for(int i=0;i<N;i++){
        pqs[i].n = 0;
        pqs[i].a = (stateWithF*)malloc(sizeof(stateWithF)*M);
    }
    state initial(s,g);
    CHECK(cudaMemcpyToSymbol(d_initial, &initial, sizeof(state)));
    state nil(xyLoc(-1,-1), xyLoc(-1,-1));
    state m = nil;
    CHECK(cudaMemcpyToSymbol(d_m, &m, sizeof(state)));
    CHECK(cudaMemcpyToSymbol(d_nil, &m, sizeof(state)));


    init_dtable<<<1,N>>>((state*)d_table);

    int* h_closed_list = (int*)malloc(sizeof(int)*h_width*h_height);
    int* d_closed_list;
    for(int i=0; i<h_width*h_height; i++){
        h_closed_list[i] = -1;
    }
    h_closed_list[initial.h_hash()] = 0;
    CHECK(cudaMalloc((int**)&d_closed_list, sizeof(int)*h_width*h_height));
    CHECK(cudaMemcpy(d_closed_list, h_closed_list, h_width*h_height*sizeof(int), cudaMemcpyHostToDevice));


    int h_lengths[N];
    int* d_lengths;
    for(int i=0;i<N;i++){
        h_lengths[i] = pqs[i].n;
    }
    CHECK(cudaMalloc((int**)&d_lengths, N*sizeof(int)));
    CHECK(cudaMemcpy(d_lengths, h_lengths, N*sizeof(int), cudaMemcpyHostToDevice));

    stateWithF* d_array;
    CHECK(cudaMalloc((stateWithF**)&d_array, M*N*sizeof(stateWithF)));
    init<<<1,1>>>((state *)d_table, d_lengths, d_array);
    CHECK(cudaMemcpy(h_lengths, d_lengths, N*sizeof(int), cudaMemcpyDeviceToHost));


    if(path.size() > 0) {
        path.push_back(g);
        return true;
    }
    bool pathFound;

    state* d_S;
    CHECK(cudaMalloc((state**)&d_S, N*8*sizeof(state)));
    state* d_neighbors;
    CHECK(cudaMalloc((state**)&d_neighbors, 8*N*sizeof(state)));
    state* d_kouho;
    state h_kouho[N];
    CHECK(cudaMalloc((state**)&d_kouho, N*sizeof(state)));

    while(!isAllQueueEmpty(h_lengths)) {
        tmp_memory_reset<<<1, N*8>>>(d_S, d_neighbors);
        reset_kouho<<<1, N>>>(d_kouho);

        remove<<<1,N>>>((state *)d_table, d_lengths, d_array, d_S, g, d_neighbors, d_map, d_kouho, d_closed_list);
        CHECK(cudaMemcpy(h_kouho, d_kouho, N*sizeof(state), cudaMemcpyDeviceToHost));
        for(int i=0;i<N;i++){
            if(h_kouho[i].isNil()){
                /* cout << "kouho is nil" << endl; */
            } else{
                if(m.isNil() || m.f_value > h_kouho[i].f_value){
                    m = h_kouho[i];
                    CHECK(cudaMemcpyToSymbol(d_m, &m, sizeof(state)));
                }
            }
        }
        duplicate_detection<<<1,N>>>((state*)d_table, d_lengths, d_array, d_S, d_closed_list);
        pathFound = false;
        CHECK(cudaMemcpy(h_lengths, d_lengths, N*sizeof(int), cudaMemcpyDeviceToHost));
        if (!m.isNil()) {
            checkIfPathIsFound<<<1,N>>>(d_array);
            CHECK(cudaMemcpyFromSymbol(&pathFound, d_pathFound, sizeof(bool)));
        }
        if(pathFound){
            break;
        }
    }
    CHECK(cudaMemcpyToSymbol(d_m, &m, sizeof(state)));
    pathFound = true;

    cudaFree(d_S);
    cudaFree(d_neighbors);
    cudaFree(d_kouho);
    cudaFree(d_closed_list);
    cudaFree(d_array);
    cudaFree(d_lengths);
    free(h_closed_list);
    for(int i=0;i<N;i++){
        free(pqs[i].a);
    }

    if (pathFound){
        xyLoc h_path[1000];
        for(int i=0;i<1000;i++){
            h_path[i] = xyLoc(-1,-1);
        }
        xyLoc* d_path;
        CHECK(cudaMalloc((xyLoc**)&d_path, 1000*sizeof(xyLoc)));
        path_create<<<1,1>>>(d_path, s);
        CHECK(cudaMemcpy(h_path, d_path, 1000*sizeof(xyLoc), cudaMemcpyDeviceToHost));
        for(int i=0;h_path[i].x != -1; i++){
            path.push_back(h_path[i]);
        }
        path.push_back(s);
        reverse(path.begin(), path.end());
        CHECK(cudaFree(d_path));
    }

	CHECK(cudaFree(d_map));
    return true;
}

void device_reset(void *data){
    cudaFree(data);
    CHECK(cudaDeviceReset());
}

__device__ int GetIndex(xyLoc s)
{
    return s.y*width+s.x;
}

__host__ __device__ state create_next_state(state &orig, xyLoc goal, Direction d) {
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

__device__ int GetSuccessors_for_gastar(state* s, state* neighbors, int num,xyLoc g, bool* d_map) {
    bool up = false, down = false, left = false, right = false;
    int i = 0;

    state next = create_next_state(*s,g,RIGHT);
    if (next.node.x < width && d_map[GetIndex(next.node)]) {
        neighbors[num*8+i] = next;
        i++;
        right = true;
    }
    next = create_next_state(*s,g,LEFT);
    if (next.node.x >= 0 && d_map[GetIndex(next.node)]) {
        neighbors[num*8+i] = next;
        i++;
        left = true;
    }
    next = create_next_state(*s,g,UP);
    if (next.node.y >= 0 && d_map[GetIndex(next.node)]) {
        neighbors[num*8+i] = next;
        i++;
        up = true;
    }
    next = create_next_state(*s,g,DOWN);
    if (next.node.y < height && d_map[GetIndex(next.node)]) {
        neighbors[num*8+i] = next;
        i++;
        down = true;
    }
    next = create_next_state(*s,g,LOWERRIGHT);
    if (next.node.y < height && next.node.x < width && d_map[GetIndex(next.node)] && right && down){
        neighbors[num*8+i] = next;
        i++;
    }
    next = create_next_state(*s,g,UPPERRIGHT);
    if (next.node.y >= 0 &&  next.node.x < width && d_map[GetIndex(next.node)] && right && up){
        neighbors[num*8+i] = next;
        i++;
    }
    next = create_next_state(*s,g,UPPERLEFT);
    if (next.node.y >= 0 && next.node.x >= 0 && d_map[GetIndex(next.node)] && left && up){
        neighbors[num*8+i] = next;
        i++;
    }
    next = create_next_state(*s,g,LOWERLEFT);
    if (next.node.y < height && next.node.x >= 0 && d_map[GetIndex(next.node)] && left && down){
        neighbors[num*8+i] = next;
        i++;
    }
    return i;
}

