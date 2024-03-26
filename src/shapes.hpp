#ifndef SHAPES_HPP
#define SHAPES_HPP

#include "materials.hpp"

// --------------------------------------------------------------------------------

struct AABB {
    Point3 min, max;

    static AABB make(AABB box0, AABB box1) {
        Point3 small{
            fmin(box0.min.x, box1.min.x),
            fmin(box0.min.y, box1.min.y),
            fmin(box0.min.z, box1.min.z)
        };

        Point3 big{
            fmax(box0.max.x, box1.max.x),
            fmax(box0.max.y, box1.max.y),
            fmax(box0.max.z, box1.max.z)
        };

        return {small, big};
    }

    float get_surface_area() const {
        float dx = max.x - min.x;
        float dy = max.y - min.y;
        float dz = max.z - min.z;

        return 2 * (dx * dy + dy * dz + dz * dx);
    }

    bool intersect(Ray ray, float t_min, float t_max) const {
        for (size_t i = 0; i < 3; ++i) {
            float t0 = fmin((min.data[i] - ray.origin.data[i]) / ray.direction.data[i],
                            (max.data[i] - ray.origin.data[i]) / ray.direction.data[i]);

            float t1 = fmax((min.data[i] - ray.origin.data[i]) / ray.direction.data[i],
                            (max.data[i] - ray.origin.data[i]) / ray.direction.data[i]);

            t_min = fmax(t0, t_min);
            t_max = fmin(t1, t_max);

            if (t_max <= t_min) {
                return false;
            }
        }

        return true;
    }
};

// --------------------------------------------------------------------------------

struct Plane {
    Point3 point;
    Vector3 normal;

    Material material;

    static Plane make(Point3 point, Vector3 normal, Material material) {
        return {point, normal, material};
    }

    Intersection intersect(Ray ray, float t_min, float t_max) const {
        Intersection ret{};

        float a = dot(ray.direction, normal);
        if (a > -EPSILON32 && a < EPSILON32) {
            return ret;
        }

        float t = dot((point - ray.origin), normal) / a;
        if (t > t_min && t < t_max) {
            ret.intersected = true;
            ret.t = t;
            ret.point = ray.at(ret.t);
            ret.material = material;

            ret.set_normal(ray, normal);
        }

        return ret;
    }

    void move(float x, float y, float z) {
        point = point + Vector3{x, y, z};
    }
};

// --------------------------------------------------------------------------------

struct Triangle {
    Point3 v0, v1, v2;
    Vector3 normal;

    Material material;

    static Triangle make(Point3 v0, Point3 v1, Point3 v2, Material material) {
        return {
            v0, v1, v2,
            normalize(cross(v1 - v0, v2 - v0)),
            material
        };
    }

    Intersection intersect(Ray ray, float t_min, float t_max) const {
        Intersection ret{};

        Vector3 edge0 = v1 - v0;
        Vector3 edge1 = v2 - v0;
        Vector3 h = cross(ray.direction, edge1);

        float a = dot(edge0, h);
        if (a == 0.0f) {
            return ret;
        }

        Vector3 s = ray.origin - v0;
        float beta = dot(s, h) / a;
        if (beta < 0.0f || beta > 1.0f) {
            return ret;
        }

        Vector3 q = cross(s, edge0);
        float gamma = dot(ray.direction, q) / a;
        if (gamma < 0.0f || beta + gamma > 1.0f) {
            return ret;
        }

        float t = dot(edge1, q) / a;
        if (t > t_min && t < t_max) {
            ret.intersected = true;
            ret.t = t;
            ret.point = ray.at(ret.t);
            ret.material = material;

            ret.set_normal(ray, normal);
        }

        return ret;
    }

    void move(float x, float y, float z) {
        v0 = v0 + Vector3{x, y, z};
        v1 = v1 + Vector3{x, y, z};
        v2 = v2 + Vector3{x, y, z};
    }
};

// --------------------------------------------------------------------------------

struct Mesh {
    // TODO(paalf): test if vertices and indices arrays are removable
    Array<Point3>   vertices;
    Array<size_t>   indices;
    Array<Triangle> triangles;

    Point3 centroid;

    Material material;

    void set_triangles() {
        for (size_t i = 0; i < indices.length; i += 3) {
            triangles.push(
                Triangle::make(
                    vertices[indices[i]],
                    vertices[indices[i + 1]],
                    vertices[indices[i + 2]],
                    material
                )
            );
        }
    }

    void set_torus_indices(Array<float> angles, float period) {
        size_t n = angles.length;

        for (size_t i = 0; i < n - 2; ++i) {
            for (size_t j = i + 1; j < n - 1; ++j) {
                if (absf(angles[i] - angles[j]) > period) {
                    continue;
                }

                for (size_t k = j + 1; k < n; ++k) {
                    if (absf(angles[i] - angles[k]) <= period &&
                        absf(angles[j] - angles[k]) <= period) {
                        indices.push(i);
                        indices.push(j);
                        indices.push(k);
                    }
                }
            }
        }
    }

    static Mesh make(Array<Point3> verts, Array<size_t> idxs, Material material) {
        Mesh ret{Array<Point3>::copy(verts), Array<size_t>::copy(idxs)};

        ret.material = material;
        ret.set_triangles();

        // TODO(paalf): make sure this works for more complex meshes
        size_t num_verts = verts.length;

        for (const auto &vert : verts) {
            ret.centroid += vert;
        }

        if (num_verts) {
            ret.centroid /= static_cast<float>(num_verts);
        }

        return ret;
    }

    static Mesh make(Point3 center, float major_radius, float minor_radius,
                     size_t num_circles, size_t verts_per_circle, Material material) {
        Mesh ret{};
        ret.material = material;

        float major_period = 360.0f / num_circles;
        float minor_period = 360.0f / verts_per_circle;

        auto angles = Array<float>::make(num_circles * verts_per_circle);

        for (float theta = 0.0f; theta <= 360.0f; theta += major_period) {
            float theta_rad = radians(theta);
            float cos_theta = cosf(theta_rad);
            float sin_theta = sinf(theta_rad);

            Point3 p{major_radius * cos_theta, major_radius * sin_theta, 0.0f};

            Vector3 v1{cos_theta, sin_theta, 0.0f};

            Vector3 v2{0.0f, 0.0f, 1.0f};

            for (float phi = 0.0f; phi <= 360.0f; phi += minor_period) {
                float phi_rad = radians(phi);

                Point3 vert = center + p + minor_radius * (cosf(phi_rad) * v1 + sinf(phi_rad) * v2);

                ret.vertices.push(vert);
                angles.push(theta);
            }
        }

        ret.set_torus_indices(angles, major_period);
        ret.set_triangles();

        ret.centroid = center;

        return ret;
    }

    void destroy() {
        vertices.destroy();
        indices.destroy();
        triangles.destroy();
    }

    Intersection intersect(Ray ray, float t_min, float t_max) const {
        Intersection ret{}, cur_info{};

        float t_closest = t_max;

        for (const auto &triangle : triangles) {
            cur_info = triangle.intersect(ray, t_min, t_closest);

            if (cur_info.intersected) {
                t_closest = cur_info.t;
                ret = cur_info;
            }
        }

        return ret;
    }

    void move(float x, float y, float z) {
        for (auto &triangle : triangles) {
            triangle.move(x, y, z);
        }
    }
};

// --------------------------------------------------------------------------------

struct Quadric {
    float a, b, c, d, e;
    float f, g, h, j, k;

    Material material;

    static Quadric make(float a, float b, float c, float d, float e,
                        float f, float g, float h, float j, float k,
                        Material material) {
        return {
            a, b, c, d, e,
            f, g, h, j, k,
            material
        };
    }

    Intersection intersect(Ray ray, float t_min, float t_max) const {
        Intersection ret{};

        Point3 orig = ray.origin;
        Vector3 dir = ray.direction;

        float acoef = a * dir.x * dir.x +
                      b * dir.y * dir.y +
                      c * dir.z * dir.z +
                      2.0f * d * dir.x * dir.y + 
                      2.0f * e * dir.y * dir.z +
                      2.0f * f * dir.x * dir.z;

        float bcoef = 2.0f * (a * orig.x * dir.x +
                              b * orig.y * dir.y +
                              c * orig.z * dir.z +
                              d * orig.x * dir.y +
                              d * orig.y * dir.x +
                              e * orig.y * dir.z +
                              e * orig.z * dir.y +
                              f * orig.x * dir.z +
                              f * orig.z * dir.x +
                              g * dir.x +
                              h * dir.y +
                              j * dir.z);

        float ccoef = a * orig.x * orig.x +
                      b * orig.y * orig.y +
                      c * orig.z * orig.z +
                      2.0f * d * orig.x * orig.y +
                      2.0f * e * orig.y * orig.z +
                      2.0f * f * orig.x * orig.z +
                      2.0f * g * orig.x +
                      2.0f * h * orig.y +
                      2.0f * j * orig.z +
                      k;

        float t;
        if (absf(acoef) < EPSILON32) {
            if (bcoef < EPSILON32) {
                return ret;
            }

            t = -ccoef / bcoef;
        } else {
            float delta = bcoef * bcoef - 4 * acoef * ccoef;
            if (delta < 0.0f) {
                return ret;
            }

            float sqrt_delta = sqrtf(delta);

            t = (-bcoef - sqrt_delta) / (2 * acoef);
            if (t < t_min || t > t_max) {
                t = (-bcoef + sqrt_delta) / (2 * acoef);

                if (t < t_min || t > t_max) {
                    return ret;
                }
            }
        }

        ret.intersected = true;
        ret.t = t;
        ret.point = ray.at(ret.t);
        ret.material = material;

        Vector3 outward_normal = normalize({
            2.0f * (a * ret.point.x + d * ret.point.y + f * ret.point.z + g),
            2.0f * (b * ret.point.y + d * ret.point.x + e * ret.point.z + h),
            2.0f * (c * ret.point.z + e * ret.point.y + f * ret.point.x + j)
        });
        ret.set_normal(ray, outward_normal);

        return ret;
    }
};

// --------------------------------------------------------------------------------

struct Sphere {
    Point3 center;
    float radius;

    Material material;

    static Sphere make(Point3 center, float radius, Material material) {
        return {center, radius, material};
    }

    Intersection intersect(Ray ray, float t_min, float t_max) const {
        Intersection ret{};

        Vector3 oc = ray.origin - center;
        float a = length2(ray.direction);
        float half_b = dot(oc, ray.direction);
        float c = length2(oc) - radius * radius;
        float delta = half_b * half_b - a * c;

        if (delta < 0.0f) {
            return ret;
        }

        float sqrt_delta = sqrtf(delta);

        float t = (-half_b - sqrt_delta) / a;
        if (t < t_min || t > t_max) {
            t = (-half_b + sqrt_delta) / a;

            if (t < t_min || t > t_max) {
                return ret;
            }
        }

        ret.intersected = true;
        ret.t = t;
        ret.point = ray.at(ret.t);
        ret.material = material;

        Vector3 outward_normal = (ret.point - center) / radius;
        ret.set_normal(ray, outward_normal);

        float theta = acosf(-outward_normal.y);
        float phi = atan2f(-outward_normal.z, outward_normal.x) + PI32;

        ret.u = phi / (2.0f * PI32);
        ret.v = theta / PI32;

        return ret;
    }

    void move(float x, float y, float z) {
        center = center + Vector3{x, y, z};
    }
};

// --------------------------------------------------------------------------------

struct Light {
    enum class Type : byte {
        MESH,
        SPHERE
    };

    Type type;

    union {
        Mesh   mesh;
        Sphere sphere;
    };

    Point3 centroid;
    float intensity;

    static Light make(Array<Point3> verts, Array<size_t> idxs, Material material, float intensity) {
        Light ret{Type::MESH};
        ret.mesh = Mesh::make(verts, idxs, material);
        ret.centroid = ret.mesh.centroid;
        ret.intensity = intensity;

        return ret;
    }

    static Light make(Point3 center, float radius, Material material, float intensity) {
        Light ret{Type::SPHERE};
        ret.sphere = Sphere::make(center, radius, material);
        ret.centroid = ret.sphere.center;
        ret.intensity = intensity;

        return ret;
    }

    Intersection intersect(Ray ray, float t_min, float t_max) const {
        switch (type) {
        case Type::MESH:   return mesh.intersect(ray, t_min, t_max);
        case Type::SPHERE: return sphere.intersect(ray, t_min, t_max);
        default:           return {};
        }
    }

    Material get_material() const {
        switch (type) {
        case Type::MESH:   return mesh.material;
        case Type::SPHERE: return sphere.material;
        default:           return {};
        }
    }
};

// --------------------------------------------------------------------------------

namespace scene {
static Array<Plane>   planes{};
static Array<Mesh>    meshes{};
static Array<Quadric> quadrics{};
static Array<Sphere>  spheres{};

static Array<Light>   lights{};

static Color3 background_color;

static float ambient_factor;

static void destroy() {
    spheres.destroy();
    planes.destroy();

    for (auto &mesh : meshes) {
        mesh.destroy();
    }
    meshes.destroy();
}

static Intersection closest_intersection(Ray ray, float t_min, float t_max) {
    Intersection ret{}, cur_info{};

    float t_closest = t_max;

    for (const auto &plane : planes) {
        cur_info = plane.intersect(ray, t_min, t_closest);

        if (cur_info.intersected) {
            t_closest = cur_info.t;
            ret = cur_info;
        }
    }

    for (const auto &mesh : meshes) {
        cur_info = mesh.intersect(ray, t_min, t_closest);

        if (cur_info.intersected) {
            t_closest = cur_info.t;
            ret = cur_info;
        }
    }

    for (const auto &quadric : quadrics) {
        cur_info = quadric.intersect(ray, t_min, t_closest);

        if (cur_info.intersected) {
            t_closest = cur_info.t;
            ret = cur_info;
        }
    }

    for (const auto &sphere : spheres) {
        cur_info = sphere.intersect(ray, t_min, t_closest);

        if (cur_info.intersected) {
            t_closest = cur_info.t;
            ret = cur_info;
        }
    }

    for (const auto &light : lights) {
        cur_info = light.intersect(ray, t_min, t_closest);

        if (cur_info.intersected) {
            t_closest = cur_info.t;
            ret = cur_info;
        }
    }

    return ret;
}

static bool shoot_shadow_ray(Ray ray) {
    Intersection info = scene::closest_intersection(ray, EPSILON32, INFINITY32);
    if (info.material.emissive) {
        return false;
    } else if (info.intersected) {
        return true;
    }

    return false;
}

static Color3 phong_shade(Ray ray, Intersection info) {
    Vector3 view_dir = normalize(ray.origin - info.point);

    Color3 closest_color = info.material.albedo.get_value(info.u, info.v, info.point);
    Color3 I{};

    for (const auto &light : lights) {
        Vector3 light_dir = normalize(light.centroid - info.point);

#define BARYCENTRIC
#ifdef BARYCENTRIC
        size_t face_idx = rand() % 2;
        Triangle face = light.mesh.triangles[face_idx];

        float alpha = rand() % 100;
        float beta = rand() % 100;
        float gamma = rand() % 100;

        float sum = alpha + beta + gamma;

        alpha /= sum;
        beta /= sum;
        gamma /= sum;

        Point3 v0 = face.v0;
        Point3 v1 = face.v1;
        Point3 v2 = face.v2;

        Point3 light_rand{
            alpha * v0.x + beta * v1.x + gamma * v2.x,
            v0.y,
            alpha * v0.z + beta * v1.z + gamma * v2.z
        };

        Vector3 rand_dir = normalize(light_rand - info.point);
#else
        //Vector3 rand_dir = randv3_in_hemisphere(light_dir);
        Vector3 rand_dir = light_dir;
#endif

        Color3 light_color = light.intensity * light.get_material().albedo.get_value(info.u, info.v, info.point);
        Ray light_ray{info.point + info.normal * BIAS, rand_dir};

        bool is_shadow = shoot_shadow_ray(light_ray);

        Vector3 reflected_dir = normalize(reflect(-light_dir, info.normal));

        Color3 ambient_color = closest_color * info.material.ka * fmaxf(scene::ambient_factor, 0.0f);

        float diffuse_factor = dot(light_dir, info.normal);
        Color3 tmp = hadamard(light_color, closest_color);
        Color3 diffuse_color = tmp * info.material.kd * fmaxf(diffuse_factor, 0.0f);

        float specular_factor = powf(dot(reflected_dir, view_dir), info.material.n);
        Color3 specular_color = light_color * info.material.ks * fmaxf(specular_factor, 0.0f);

        if (!is_shadow) {
            I += ambient_color + diffuse_color + specular_color;
        }
    }

    return I;
}
} // namespace scene

// --------------------------------------------------------------------------------

#endif // SHAPES_HPP
