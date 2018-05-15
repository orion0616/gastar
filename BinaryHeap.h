#ifndef BINARYHEAP_H_
#define BINARYHEAP_H_

#include "Entry.h"

class BinaryHeap {
public:
    int n;
    stateWithF a[1000];

    void bubbleUp(int i){
        int p = parent(i);
        while (i > 0 && (a[i] < a[p])) {
            stateWithF tmp = a[i];
            a[i] = a[p];
            a[p] = tmp;
            i = p;
            p = parent(i);
        }
    }
    void trickleDown(int i){
        do {
            int j = -1;
            int r = right(i);
            if (r < n && (a[r] < a[i])) {
                int l = left(i);
                if (a[l] < a[r]) {
                    j = l;
                } else {
                    j = r;
                }
            } else {
                int l = left(i);
                if (l < n && (a[l] < a[i])) {
                    j = l;
                }
            }
            if (j >= 0) {
                stateWithF tmp = a[i];
                a[i] = a[j];
                a[j] = tmp;
            }
            i = j;
        } while (i >= 0);
    }
    bool empty() {
        return n==0;
    }
    static int left(int i) {
        return 2*i + 1;
    }
    static int right(int i) {
        return 2*i + 2;
    }
    static int parent(int i) {
        return (i-1)/2;
    }
    BinaryHeap(){};
    ~BinaryHeap(){};
    bool add(stateWithF x){
        a[n] = x;
        n++;
        bubbleUp(n-1);
        return true;
    };
    stateWithF remove(){
        stateWithF x = a[0];
        a[0] = a[--n];
        // a.pop_back();
        trickleDown(0);
        return x;
    };
    stateWithF top() {
        return a[0];
    }
    int size() {
        return n;
    }
};

#endif /* BINARYHEAP_H_ */
