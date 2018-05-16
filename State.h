#include <stdint.h>
#include <vector>
#include <cmath>
#include <functional>

extern int width, height;

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
    xyLoc(){};
    xyLoc(int16_t x, int16_t y) {
        this->x = x;
        this->y = y;
    }
};

struct state {
    xyLoc node;
    double f_value;
    double g_value;
    bool isOpen;
    state* parent;
    state(xyLoc s, xyLoc g) {
        this->node = s;
        this->g_value = 0;
        this->f_value = this->g_value + octile(g);
        this->isOpen = true;
    };
    double octile(xyLoc g) {
        double dx = abs(node.x-g.x);
        double dy = abs(node.y-g.y);
        return (dx+dy) + (1.4142-2)*fmin(dx,dy);
    }
    state(){};
    bool isNil() {
        return this->node.x == -1 && this->node.y == -1;
    }
    int hash() {
        // return (node.x+1)*10000+node.y;
        return (node.x)*height + node.y;
    }
};

struct stateWithF {
    state* ptr;
    double fWhenInserted;
    stateWithF() {};
    stateWithF(state* ptr) {
        this->ptr = ptr;
        this->fWhenInserted = ptr->f_value;
    }
    bool operator<(const stateWithF& rhs) const {
        return fWhenInserted < rhs.fWhenInserted;
    }
};
