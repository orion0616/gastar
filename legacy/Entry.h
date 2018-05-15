#include <stdint.h>
#include <vector>
#include <cmath>
#include <functional>
#include <cuda_runtime.h>

enum Direction
{
    UP,
    UPPERLEFT,
    LEFT,
    LOWERLEFT,
    DOWN,
    LOWERRIGHT,
    RIGHT,
    UPPERRIGHT
};

struct xyLoc {
    int16_t x;
    int16_t y;
    __host__ __device__ int hash() {
        return (x+1)*10000+y;
    };
    __host__ __device__ xyLoc();
    // xyLoc();
    xyLoc(int16_t x, int16_t y) {
        this->x = x;
        this->y = y;
    }
};

struct state {
    xyLoc node;
    double f_value;
    double g_value;
    __host__ __device__ bool operator>(const state& rhs) const {
        if(f_value == rhs.f_value)
            return g_value < rhs.g_value;
        else
            return f_value > rhs.f_value;
    }
    __host__ __device__ bool operator<(const state& rhs) const {
        if(f_value == rhs.f_value)
            return g_value > rhs.g_value;
        else
            return f_value < rhs.f_value;
    }
    state(xyLoc s, xyLoc g) {
        this->node = s;
        this->g_value = 0;
        this->f_value = this->g_value + octile(g);
    };
    double octile(xyLoc g) {
        double dx = abs(node.x-g.x);
        double dy = abs(node.y-g.y);
        return (dx+dy) + (1.4142-2)*fmin(dx,dy);
    }
    __host__ __device__ state(){};
    __host__ __device__ bool isNil() {
        return this->node.x == -1 && this->node.y == -1;
    }
    __host__ __device__ int hash() {
        return node.hash();
    }

};
__host__ __device__ int GetSuccessors_for_gastar(state* s, state** neighbors);
void PreprocessMap(std::vector<bool> &bits, int width, int height, const char *filename);
void *PrepareForSearch(std::vector<bool> &bits, int width, int height, const char *filename);
const char *GetName();
void printPath(std::vector<xyLoc> path);
void GetSuccessors(xyLoc s, std::vector<xyLoc> & neighbors);
int GetIndex(xyLoc s);
void ExtractPath(xyLoc end, std::vector<xyLoc> & finalPath);
bool GetPath_GASTAR(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path);
