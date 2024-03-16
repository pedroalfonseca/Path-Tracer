#ifndef MATH_HPP
#define MATH_HPP

#include <math.h>
#include <float.h>

// --------------------------------------------------------------------------------

#define INFINITY32 FLT_MAX
#define EPSILON32  (1e-6)
#define PI32       3.14159265359f
#define BIAS       (1e-2)

// --------------------------------------------------------------------------------

inline float randf() {
    return rand() / (RAND_MAX + 1.0f);
}

inline float randf(float min, float max) {
    return min + ((max - min) * randf());
}

inline size_t randzu(size_t min, size_t max) {
    return static_cast<size_t>(randf(min, max + 1));
}

// --------------------------------------------------------------------------------

// Adapted from https://github.com/id-Software/Quake-III-Arena/blob/master/code/game/q_math.c#L552
inline float rsqrtf(float num) {
    float res = num;
    long i = *reinterpret_cast<long *>(&res);
    i = 0x5f3759df - (i >> 1);
    res = *reinterpret_cast<float *>(&i);

    return res * (1.5f - 0.5f * num * res * res);
}

// Adapted from https://github.com/id-Software/Quake-III-Arena/blob/master/code/game/q_math.c#L574
inline float absf(float num) {
    float res = num;
    long tmp = *reinterpret_cast<long *>(&res);
    tmp &= 0x7fffffff;

    return *reinterpret_cast<float *>(&tmp);
}

// --------------------------------------------------------------------------------

inline int clampi(int num, int min, int max) {
    return num < min ? min : (num > max ? max : num);
}

inline float clampf(float num, float min, float max) {
    return num < min ? min : (num > max ? max : num);
}

// --------------------------------------------------------------------------------

inline float radians(float degrees) {
    return (degrees / 180.0f) * PI32;
}

// --------------------------------------------------------------------------------

union Vector3 {
    struct { float x, y, z; };
    struct { float r, g, b; };

    float data[3];
};

using Point3 = Vector3;
using Color3 = Vector3;

inline Vector3 operator+(Vector3 lhs, Vector3 rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

inline Vector3 &operator+=(Vector3 &lhs, Vector3 rhs) {
    lhs = lhs + rhs;
    return lhs;
}

inline Vector3 operator-(Vector3 vec) {
    return {-vec.x, -vec.y, -vec.z};
}

inline Vector3 operator-(Vector3 lhs, Vector3 rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

inline Vector3 &operator-=(Vector3 &lhs, Vector3 rhs) {
    lhs = lhs - rhs;
    return lhs;
}

inline Vector3 operator*(Vector3 lhs, float rhs) {
    return {lhs.x * rhs, lhs.y * rhs, lhs.z * rhs};
}

inline Vector3 operator*(float lhs, Vector3 rhs) {
    return rhs * lhs;
}

inline Vector3 &operator*=(Vector3 &lhs, float rhs) {
    lhs = lhs * rhs;
    return lhs;
}

inline Vector3 operator/(Vector3 lhs, float rhs) {
    return lhs * (1.0f / rhs);
}

inline Vector3 &operator/=(Vector3 &lhs, float rhs) {
    lhs = lhs / rhs;
    return lhs;
}

inline float dot(Vector3 lhs, Vector3 rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

inline Vector3 hadamard(Vector3 lhs, Vector3 rhs) {
    return {lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z};
}

inline Vector3 cross(Vector3 lhs, Vector3 rhs) {
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
}

inline float length2(Vector3 vec) {
    return dot(vec, vec);
}

inline float length(Vector3 vec) {
    return sqrtf(length2(vec));
}

inline Vector3 normalize(Vector3 vec) {
    return vec * rsqrtf(length2(vec));
}

inline bool is_near_zero(Vector3 vec) {
    return absf(vec.x) < EPSILON32 && absf(vec.y) < EPSILON32 && absf(vec.z) < EPSILON32;
}

inline bool is_zero(Vector3 vec) {
    return vec.x == 0.0f && vec.y == 0.0f && vec.z == 0.0f;
}

inline Vector3 reflect(Vector3 incident, Vector3 normal) {
    return incident - 2.0f * dot(incident, normal) * normal;
}

inline Vector3 refract(Vector3 incident, Vector3 normal, float etai_over_etat) {
    float cos_theta = fmin(dot(-incident, normal), 1.0f);
    float sin2_theta = (etai_over_etat * etai_over_etat) * (1.0f - (cos_theta * cos_theta));

    if (sin2_theta > 1.0f) {
        return {};
    }

    return etai_over_etat * incident + (etai_over_etat * cos_theta - sqrtf(1.0f - sin2_theta)) * normal;
}

inline Vector3 randv3() {
    return {randf(), randf(), randf()};
}

inline Vector3 randv3(float min, float max) {
    return {randf(min, max), randf(min, max), randf(min, max)};
}

inline Vector3 randv3_in_unit_sphere() {
    while (true) {
        Vector3 vec = randv3(-1.0f, 1.0f);

        if (length2(vec) < 1.0f) {
            return vec;
        }
    }
}

inline Vector3 randv3_normalized() {
    return normalize(randv3_in_unit_sphere());
}

inline Vector3 randv3_in_hemisphere(Vector3 normal) {
    Vector3 in_unit_sphere = randv3_normalized();

    return dot(in_unit_sphere, normal) > 0.0f ? in_unit_sphere : -in_unit_sphere;
}

// --------------------------------------------------------------------------------

union Quaternion {
    struct {
        union {
            Vector3 xyz;
            struct { float x, y, z; };
        };

        float w;
    };

    float data[4];
};

inline Quaternion rotation(float degrees, Vector3 vec) {
    float half_angle = radians(degrees / 2.0f);

    float sin_half_angle = sinf(half_angle);
    float cos_half_angle = cosf(half_angle);

    return {sin_half_angle * vec, cos_half_angle};
}

// --------------------------------------------------------------------------------

// Adapted from https://blog.molecular-matters.com/2013/05/24/a-faster-Quaternion-vector-multiplication/
inline Vector3 rotate(Vector3 vec, Quaternion quat) {
    Vector3 t = 2.0f * cross(quat.xyz, vec);
    return vec + quat.w * t + cross(quat.xyz, t);
}

inline Vector3 x_rotate(Vector3 vec, float degrees) {
    Quaternion quat = rotation(degrees, {1.0f, 0.0f, 0.0f});
    return rotate(vec, quat);
}

inline Vector3 y_rotate(Vector3 vec, float degrees) {
    Quaternion quat = rotation(degrees, {0.0f, 1.0f, 0.0f});
    return rotate(vec, quat);
}

// --------------------------------------------------------------------------------

namespace perlin {
static constexpr size_t n = 256;
static Vector3 random_vectors[n] = {};
static size_t x_permutation[n] = {}, y_permutation[n] = {}, z_permutation[n] = {};

static void permute(size_t *perm) {
    for (size_t i = 0; i < n; ++i) {
        perm[i] = i;
    }

    for (size_t i = n - 1; i > 0; --i) {
        size_t target = randzu(0, i);
        size_t tmp = perm[i];

        perm[i] = perm[target];
        perm[target] = tmp;
    }
}

static void init() {
    for (size_t i = 0; i < n; ++i) {
        random_vectors[i] = randv3(-1.0f, 1.0f);
    }

    permute(x_permutation);
    permute(y_permutation);
    permute(z_permutation);
}

static float trilerp(Vector3 c[2][2][2], float u, float v, float w) {
    float uu = u * u * (3 - 2 * u);
    float vv = v * v * (3 - 2 * v);
    float ww = w * w * (3 - 2 * w);

    float accum = 0.0f;

    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            for (size_t k = 0; k < 2; ++k) {
                Vector3 v_weight{u - i, v - j, w - k};
                accum += (i * uu + (1 - i) * (1 - uu)) *
                         (j * vv + (1 - j) * (1 - vv)) *
                         (k * ww + (1 - k) * (1 - ww)) *
                         dot(c[i][j][k], v_weight);
            }
        }
    }

    return accum;
}

static float get_noise(Point3 point) {
    float u = point.x - floorf(point.x);
    float v = point.y - floorf(point.y);
    float w = point.z - floorf(point.z);

    size_t i = static_cast<size_t>(floorf(point.x));
    size_t j = static_cast<size_t>(floorf(point.y));
    size_t k = static_cast<size_t>(floorf(point.z));

    Vector3 c[2][2][2];

    for (size_t di = 0; di < 2; ++di) {
        for (size_t dj = 0; dj < 2; ++dj) {
            for (size_t dk = 0; dk < 2; ++dk) {
                c[di][dj][dk] = random_vectors[
                    x_permutation[(i + di) & 255] ^
                    y_permutation[(j + dj) & 255] ^
                    z_permutation[(k + dk) & 255]
                ];
            }
        }
    }

    return trilerp(c, u, v, w);
}

static float turb(Point3 point, size_t depth = 7) {
    float accum = 0.0f;
    float weight = 1.0f;
    Point3 tmp = point;

    for (size_t i = 0; i < depth; ++i) {
        accum += weight * get_noise(tmp);
        weight *= 0.5f;
        tmp = 2.0f * tmp;
    }

    return absf(accum);
}
} // namespace perlin

// --------------------------------------------------------------------------------

#endif // MATH_HPP