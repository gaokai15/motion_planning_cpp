#ifndef VEC2_H
#define VEC2_H
#include <cmath>

using namespace std;
struct Vec2 {
    float x, y;
    Vec2(float x, float y) : x(x), y(y) {}

    // Subtract two vectors
    Vec2 operator-(const Vec2& rhs) const {
        return {x - rhs.x, y - rhs.y};
    }

    // Dot product
    float dot(const Vec2& rhs) const {
        return x * rhs.x + y * rhs.y;
    }

    // Get a perpendicular vector
    Vec2 perp() const {
        return {-y, x};
    }

    // Normalize the vector
    void normalize() {
        float len = sqrt(x * x + y * y);
        x /= len;
        y /= len;
    }
};

#endif // VEC2_H
