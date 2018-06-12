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

#define N 128
#define M 200

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
__device__ state d_initial;
__device__ state d_nil;
__device__ state d_m;
__device__ int id;

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
    return (void *)13182;
}

__device__ void Rand(unsigned int* randx) {
    int num = threadIdx.x;
    randx[num] = (randx[num]*1103515245+12345)&2147483647;
    return;
}

__global__ void remove(state* table, int* lengths, stateWithF* array, state* S, xyLoc goal, state* neighbors, bool* d_map, state* kouho, int* closed_list){
    /* printf("Start removing\n"); */
    int num = threadIdx.x;
    stateWithF elem[M];
    for(int i=0;i<M;i++){
        elem[i] = array[num*M+i];
    }
    BinaryHeap pq = BinaryHeap(elem,lengths[num]);
    if (pq.empty()){
        return;
    }
    /* printf("Pq %dth. Start choosing. The number of elements is %d. \n", num, pq.size()); */

    stateWithF min;
    do {
        min = pq.remove();
        /* printf("choose %d %d from pq%d-> open?:%d,    goal is %d,%d\n", min.ptr->node.x, min.ptr->node.y, num, min.ptr->isOpen, goal.x, goal.y); */
    } while(!min.ptr->isOpen && !pq.empty());
    lengths[num] = pq.size();
    for(int i=0;i<M;i++){
        array[num*M+i] = pq.a[i];
    }
    if (!min.ptr->isOpen)
        return;

    min.ptr->isOpen = false;
    if (min.ptr->node.x == goal.x && min.ptr->node.y == goal.y) {
        /* printf("Found! %d %d -> open?:%d\n", min.ptr->node.x, min.ptr->node.y, min.ptr->isOpen); */
        if (d_m.isNil() || min.ptr->f_value < d_m.f_value) {
            kouho[num] = *(min.ptr);
            /* printf("d_m change!!\n"); */
            /* d_m = *(min.ptr);         */
        }
    }
    /* printf("Start extracting\n"); */

    int numOfNeighbors = GetSuccessors_for_gastar(&table[closed_list[min.ptr->hash()]], neighbors, num, goal, d_map);
    /* printf("created neighbors\n"); */
    /* int numOfNeighbors = GetSuccessors_for_gastar(&table[min.ptr->hash()], neighbors, num, goal, d_map); */
    for(int i= 0;i<numOfNeighbors; i++) {
        S[num*8+i]= neighbors[num*8+i];
        /* printf("%d %d\n", S[num*8+i].node.x, S[num*8+i].node.y); */
    }
    /* printf("End extracting\n"); */
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

__global__ void duplicate_detection(state* table, int *lengths, stateWithF* array, state* S, unsigned int* random, int* closed_list){
    int num = threadIdx.x;
    /* printf("start duplidate in pq %d\n", num); */

    // XXX
    int j = num;
    for(int i=0;i<8;i++) {
        state s = S[num*8+i];
        if(s.isNil()){
            return;
        }
        int myid = atomicAdd(&id, 1);
        table[myid] = s;

        int old = closed_list[s.hash()];
        __threadfence();

        while(closed_list[s.hash()] == -1 || table[closed_list[s.hash()]].g_value > s.g_value){
            atomicCAS(&closed_list[s.hash()], old, myid);
            closed_list[s.hash()] = myid;
            old = closed_list[s.hash()];
            __threadfence();
        }
        __threadfence();
        /* while(table[closed_list[s.hash()]].g_value > s.g_value){ */
        /*     closed_list[s.hash()] = myid;                        */
        /* }                                                        */
        /* __threadfence();                                         */
        if (closed_list[s.hash()] == myid){
            Rand(random);

            //XXX
            /* int result = random[num]%N; */
            /* int result = j%N; */
            int result = (num + 1)%N;
            j++;


            stateWithF froms(&table[myid]);
            stateWithF tmp[M];
            for(int i=0;i<M;i++){
                tmp[i] = array[i+M*result];
            }
            BinaryHeap pq = BinaryHeap(tmp, lengths[result]);

            // These methods may include some bugs
            //XXX
            pq.add(froms);
            lengths[result] = pq.n;

            for(int i=0;i<M;i++){
                array[i+M*result] = pq.a[i];
            }
            /* printf("add (%d,%d) to pq %d from pq %d when f-value is %f\n", table[myid].node.x, table[myid].node.y, result, num, froms.fWhenInserted); */
        }
        __threadfence();



        /* while(table[s.hash()].isNil() || table[s.hash()].g_value > s.g_value){ */
        /*     // this must be atomic                                             */
        /*     //XXX                                                              */
        /*     table[s.hash()] = s;                                               */

        /*     if (s.g_value != table[s.hash()].g_value)                          */
        /*         continue;                                                      */
        /*     Rand(random);                                                      */
        /*     int result = random[num]%N;                                        */
        /*     stateWithF froms(&table[s.hash()]);                                */
        /*     stateWithF tmp[M];                                                 */
        /*     for(int i=0;i<M;i++){                                              */
        /*         tmp[i] = array[i+M*result];                                    */
        /*     }                                                                  */
        /*     BinaryHeap pq = BinaryHeap(tmp, lengths[result]);                  */
        /*     pq.add(froms);                                                     */
        /*     lengths[result] = pq.n;                                            */
        /*     for(int i=0;i<M;i++){                                              */
        /*         array[i+M*result] = pq.a[i];                                   */
        /*     }                                                                  */

        /* }                                                                      */
        /* if (old.isNil() || old.g_value > s.g_value) {         */
        /*     table[s.hash()] = s;                              */
        /*     if(s != table[s.hash()])                          */


        /*     Rand(random);                                     */
        /*     int result = random[num]%N;                       */
        /*     stateWithF froms(&table[s.hash()]);               */
        /*     stateWithF tmp[M];                                */
        /*     for(int i=0;i<M;i++){                             */
        /*         tmp[i] = array[i+M*result];                   */
        /*     }                                                 */
        /*     BinaryHeap pq = BinaryHeap(tmp, lengths[result]); */
        /*     pq.add(froms);                                    */
        /*     lengths[result] = pq.n;                           */
        /*     for(int i=0;i<M;i++){                             */
        /*         array[i+M*result] = pq.a[i];                  */
        /*     }                                                 */
        /* }                                                     */
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
    /* printf("Start to create path!\n"); */
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
    /* printf("Finish creating path!\n"); */
}


bool GetPath_GASTAR(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path) {
    int h_id = 1;
    CHECK(cudaMemcpyToSymbol(id, &h_id, sizeof(int)));

    CHECK(cudaMemcpyToSymbol(width, &h_width, sizeof(int)));
    CHECK(cudaMemcpyToSymbol(height, &h_height, sizeof(int)));

    int mapsize = map.size();
    bool h_map[mapsize];
    bool *d_map;
    CHECK(cudaMalloc((bool**)&d_map, mapsize*sizeof(bool)));
    for(int i=0;i<mapsize;i++){
        h_map[i] = map[i];
    }
    CHECK(cudaMemcpy(d_map, h_map, mapsize*sizeof(bool), cudaMemcpyHostToDevice));


    unsigned int h_random[N];
    unsigned int *d_random;
    CHECK(cudaMalloc((unsigned int**)&d_random, N*sizeof(unsigned int)));
    for(int i=0;i<N;i++){
        h_random[i] = i;
    }
    CHECK(cudaMemcpy(d_random, h_random, N*sizeof(unsigned int), cudaMemcpyHostToDevice));
    CHECK(cudaDeviceSynchronize());


    assert((long)data == 13182);
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


    // XXX:
    // must change length
    state* h_table = (state*)malloc(sizeof(state)*1000000);
    state* d_table;
    for(int i=0; i<1000000; i++){
        h_table[i] = nil;
    }
    h_table[0] = initial;
    CHECK(cudaMalloc((state**)&d_table, 1000000*sizeof(state)));
    CHECK(cudaMemcpy(d_table, h_table, 1000000*sizeof(state), cudaMemcpyHostToDevice));
    CHECK(cudaDeviceSynchronize());

    int* h_closed_list = (int*)malloc(sizeof(int)*h_width*h_height);
    int* d_closed_list;
    for(int i=0; i<h_width*h_height; i++){
        h_closed_list[i] = -1;
    }
    h_closed_list[initial.h_hash()] = 0;
    CHECK(cudaMalloc((int**)&d_closed_list, sizeof(int)*h_width*h_height));
    CHECK(cudaMemcpy(d_closed_list, h_closed_list, h_width*h_height*sizeof(int), cudaMemcpyHostToDevice));
    CHECK(cudaDeviceSynchronize());


    int h_lengths[N];
    int* d_lengths;
    for(int i=0;i<N;i++){
        h_lengths[i] = pqs[i].n;
    }
    CHECK(cudaMalloc((int**)&d_lengths, N*sizeof(int)));
    CHECK(cudaMemcpy(d_lengths, h_lengths, N*sizeof(int), cudaMemcpyHostToDevice));
    CHECK(cudaDeviceSynchronize());

    stateWithF* h_array = (stateWithF*)malloc(sizeof(stateWithF)*N*M);
    stateWithF* d_array;
    for(int i=0;i<N;i++){
        for(int j=0;j<M;j++){
            h_array[i*M+j] = pqs[i].a[j];
        }
    }
    CHECK(cudaMalloc((stateWithF**)&d_array, M*N*sizeof(stateWithF)));
    CHECK(cudaMemcpy(d_array, h_array, M*N*sizeof(stateWithF), cudaMemcpyHostToDevice));
    init<<<1,1>>>(d_table, d_lengths, d_array);
    CHECK(cudaMemcpy(h_lengths, d_lengths, N*sizeof(int), cudaMemcpyDeviceToHost));
    CHECK(cudaMemcpy(h_array, d_array, M*N*sizeof(stateWithF), cudaMemcpyDeviceToHost));
    CHECK(cudaDeviceSynchronize());


    if(path.size() > 0) {
        path.push_back(g);
        return true;
    }
    CHECK(cudaDeviceSynchronize());
    bool pathFound;
    while(!isAllQueueEmpty(h_lengths)) {
        state*  S= (state*)malloc(sizeof(state)*N*8);
        for(int i=0;i<8*N;i++){
            S[i] = nil;
        }
        state* d_S;
        CHECK(cudaMalloc((state**)&d_S, N*8*sizeof(state)));
        CHECK(cudaMemcpy(d_S, S, 8*N*sizeof(state), cudaMemcpyHostToDevice));

        state* neighbors = (state*)malloc(sizeof(state)*N*8);
        for(int i=0;i<8*N;i++){
            neighbors[i] = nil;
        }
        state* d_neighbors;
        CHECK(cudaMalloc((state**)&d_neighbors, 8*N*sizeof(state)));
        CHECK(cudaMemcpy(d_neighbors, neighbors, 8*N*sizeof(state), cudaMemcpyHostToDevice));

        CHECK(cudaMemcpy(d_lengths, h_lengths, N*sizeof(int), cudaMemcpyHostToDevice));

        state* d_kouho;
        state h_kouho[N];
        for(int i=0;i<N;i++){
            h_kouho[i] = nil;
        }
        CHECK(cudaMalloc((state**)&d_kouho, N*sizeof(state)));
        CHECK(cudaMemcpy(d_kouho, h_kouho, N*sizeof(state), cudaMemcpyHostToDevice));



        /* cout << "start removing\n" << endl; */
        remove<<<1,N>>>(d_table, d_lengths, d_array, d_S, g, d_neighbors, d_map, d_kouho, d_closed_list);
        CHECK(cudaDeviceSynchronize());
        CHECK(cudaMemcpy(h_lengths, d_lengths, N*sizeof(int), cudaMemcpyDeviceToHost));
        CHECK(cudaMemcpy(h_array, d_array, M*N*sizeof(stateWithF), cudaMemcpyDeviceToHost));
        CHECK(cudaMemcpy(h_kouho, d_kouho, N*sizeof(state), cudaMemcpyDeviceToHost));
        for(int i=0;i<N;i++){
            if(h_kouho[i].isNil()){
                /* cout << "kouho is nil" << endl; */
            } else{
                if(m.isNil() || m.f_value > h_kouho[i].f_value)
                    m = h_kouho[i];
            }
        }
        /* cout << "For end" << endl; */


        /* CHECK(cudaMemcpyFromSymbol(&m, d_m, sizeof(state))); */

        pathFound = false;
        if (!m.isNil()) {
            /* cout <<"これでいいかチェックします！" << endl; */
            pathFound = true;
            for(int i=0; i< N; i++) {
                if (h_lengths[i] == 0){
                    continue;
                } else {
                    /* cout << m.f_value << " " << h_array[M*i].fWhenInserted << endl; */
                    if(m.f_value > h_array[M*i].fWhenInserted){
                        pathFound = false;
                        break;
                    }
                }
            }
            CHECK(cudaMemcpyToSymbol(d_m, &m, sizeof(state)));
        }
        if(pathFound){
            free(S);
            free(neighbors);
            break;
        }
        /* cout << "start duplicate detection\n" << endl; */
        duplicate_detection<<<1,N>>>(d_table, d_lengths, d_array, d_S, d_random, d_closed_list);
        CHECK(cudaMemcpy(h_lengths, d_lengths, N*sizeof(int), cudaMemcpyDeviceToHost));
        CHECK(cudaDeviceSynchronize());

        free(S);
        free(neighbors);
        cudaFree(d_S);
        cudaFree(d_neighbors);
        cudaFree(d_kouho);
    }
    /* cout << "HERE?" << endl; */
    free(h_array);
    for(int i=0;i<N;i++){
        free(pqs[i].a);
    }

    if (pathFound){
        //XXX
        xyLoc h_path[1000];
        for(int i=0;i<1000;i++){
            h_path[i] = xyLoc(-1,-1);
        }
        xyLoc* d_path;
        CHECK(cudaMalloc((xyLoc**)&d_path, 1000*sizeof(xyLoc)));
        path_create<<<1,1>>>(d_path, s);
        CHECK(cudaDeviceSynchronize());
        CHECK(cudaMemcpy(h_path, d_path, 1000*sizeof(xyLoc), cudaMemcpyDeviceToHost));
        for(int i=0;h_path[i].x != -1; i++){
            path.push_back(h_path[i]);
        }
        path.push_back(s);
        reverse(path.begin(), path.end());
        CHECK(cudaFree(d_path));
    }

    CHECK(cudaFree(d_table));
    free(h_table);

	CHECK(cudaFree(d_map));

    CHECK(cudaFree(d_random));

    CHECK(cudaDeviceReset());
    return true;
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

