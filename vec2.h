#ifndef VEC2_H
#define VEC2_H
#include <cmath>

using namespace std;
struct Vec2 {
    float x, y;
    Vec2() : x(0), y(0) {}
    Vec2(float x, float y) : x(x), y(y) {}

    Vec2 operator+(const Vec2& rhs) const {
        return {x + rhs.x, y + rhs.y};
    }

    Vec2 operator*(const float& rhs) const {
        return {x * rhs, y * rhs};
    }

    // Subtract two vectors
    Vec2 operator-(const Vec2& rhs) const {
        return {x - rhs.x, y - rhs.y};
    }

    // Dot product
    float dot(const Vec2& rhs) const {
        return x * rhs.x + y * rhs.y;
    }

    float cross(const Vec2& rhs) const {
        return x * rhs.y - y * rhs.x;
    }

    float magnitude() const {
        return std::sqrt(x * x + y * y);
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

    float distance( const Vec2& rhs) const {
        return sqrt((x - rhs.x) * (x - rhs.x) + (y - rhs.y) * (y - rhs.y));
    }
};

#endif // VEC2_H
