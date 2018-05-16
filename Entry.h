#include <stdint.h>
#include <vector>
#include <cmath>
#include <functional>
#include "BinaryHeap.h"

// int GetSuccessors_for_gastar(state* s, state* neighbors, int num, xyLoc g);
__device__ int GetSuccessors_for_gastar(state* s, state* neighbors, int num,xyLoc g, bool* d_map);
void PreprocessMap(std::vector<bool> &bits, int width, int height, const char *filename);
void *PrepareForSearch(std::vector<bool> &bits, int width, int height, const char *filename);
const char *GetName();
void printPath(std::vector<xyLoc> path);
// void GetSuccessors(xyLoc s, std::vector<xyLoc> & neighbors);
__device__ int GetIndex(xyLoc s);
bool GetPath_GASTAR(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path);
void duplicate_detection(state** table, BinaryHeap* pqs, std::vector<std::vector<state*> >S);
