#ifndef BINARYHEAP_H_
#define BINARYHEAP_H_

#include <vector>
#include "Entry.h"


class BinaryHeap {
public:
    int n;
    state* a[1000];
    __host__ __device__ void bubbleUp(int i){
        int p = parent(i);
        while (i > 0 && (*a[i] < *a[p])) {
            state* tmp = a[i];
            a[i] = a[p];
            a[p] = tmp;
            i = p;
            p = parent(i);
        }
    }
    __host__ __device__ void trickleDown(int i){
        do {
            int j = -1;
            int r = right(i);
            if (r < n && (*a[r] < *a[i])) {
                int l = left(i);
                if (*a[l] < *a[r]) {
                    j = l;
                } else {
                    j = r;
                }
            } else {
                int l = left(i);
                if (l < n && (*a[l] < *a[i])) {
                    j = l;
                }
            }
            if (j >= 0) {
                state* tmp = a[i];
                a[i] = a[j];
                a[j] = tmp;
            }
            i = j;
        } while (i >= 0);
    }
    __host__ __device__ bool empty() {
        return n==0;
    }
    __host__ __device__ static int left(int i) {
        return 2*i + 1;
    }
    __host__ __device__ static int right(int i) {
        return 2*i + 2;
    }
    __host__ __device__ static int parent(int i) {
        return (i-1)/2;
    }
    __host__ __device__ BinaryHeap(state** array, int length) {
        for(int i=0; i<1000;i++){
            a[i] = array[i];
        }
        n=length;
    }
    __host__ __device__ ~BinaryHeap() {;}
    __host__ __device__ bool add(state* x){
        a[n] = x;
        n++;
        bubbleUp(n-1);
        return true;
    };
    __host__ __device__ state* remove(){
        state* x = a[0];
        a[0] = a[--n];
        // a.pop_back();
        trickleDown(0);
        return x;
    };
    __host__ __device__ state* top() {
        return a[0];
    }
    __host__ __device__ int size() {
        return n;
    }
};

#endif /* BINARYHEAP_H_ */
