#include "parser.h"
#include <cmath>

namespace parser{
    Vec3f add(Vec3f v, Vec3f u){
        Vec3f result;
        result.x = v.x + u.x;
        result.y = v.y + u.y;
        result.z = v.z + u.z;
        return result;
    }

    // Subtract vector "u" from vector "v"
    Vec3f substract(Vec3f v, Vec3f u){
        Vec3f result;
        result.x = v.x - u.x;
        result.y = v.y - u.y;
        result.z = v.z - u.z;
        return result;
    }

    float dotProduct(Vec3f v, Vec3f u){
        float result = (v.x * u.x) + (v.y * u.y) + (v.z * u.z);
        return result;
    }

    float length(Vec3f v) {
        return sqrt(dotProduct(v,v));
    }

    Vec3f normalize(Vec3f v) {
        Vec3f result;
        float lengthOfV = length(v);
        result.x = v.x / lengthOfV;
        result.y = v.y / lengthOfV;
        result.z = v.z / lengthOfV;
        return result;
    }

    float distance(Vec3f v, Vec3f u){
        float xDiff = v.x - u.x;
        float yDiff = v.y - u.y;
        float zDiff = v.z - u.z;
        return sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff);
    }

    Vec3f crossProduct(Vec3f v, Vec3f u){
        Vec3f result;
        result.x = v.y*u.z - v.z*u.y;
        result.y = v.z*u.x - v.x*u.z;
        result.z = v.x*u.y - v.y*u.x;
        return result;
    }

    float det(Vec3f v, Vec3f u, Vec3f w){
        return v.x * (u.y * w.z - u.z * w.y)
            - u.x * (v.y * w.z - v.z * w.y)
            + w.x * (v.y * u.z - v.z * u.y); 
    }
}
