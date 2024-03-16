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

namespace scene {
static Array<Plane>   planes{};
static Array<Mesh>    meshes{};
static Array<Quadric> quadrics{};
static Array<Sphere>  spheres{};

static Array<Mesh>    lights{};

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

static float get_k_shadow(Ray ray) {
    Intersection info{};

    for (const auto &plane : planes) {
        info = plane.intersect(ray, EPSILON32, INFINITY32);

        if (info.intersected) {
            return info.material.kt;
        }
    }

    for (const auto &mesh : meshes) {
        info = mesh.intersect(ray, EPSILON32, INFINITY32);

        if (info.intersected) {
            return info.material.kt;
        }
    }

    for (const auto &sphere : spheres) {
        info = sphere.intersect(ray, EPSILON32, INFINITY32);

        if (info.intersected) {
            return info.material.kt;
        }
    }

    for (const auto &light : lights) {
        info = light.intersect(ray, EPSILON32, INFINITY32);

        if (info.intersected) {
            return 0.0f;
        }
    }

    return 0.0f;
}

static Color3 phong_shade(Ray ray, Intersection info) {
    Vector3 view_dir = normalize(ray.origin - info.point);

    Color3 closest_color = info.material.albedo.get_value(info.u, info.v, info.point);
    Color3 I = closest_color * info.material.ka * fmaxf(scene::ambient_factor, 0.0f);

    constexpr float light_intensity = 1.0f;

    for (const auto &light : lights) {
        Vector3 light_dir = normalize(light.centroid - info.point);
        Color3 light_color = light_intensity * light.material.albedo.get_value(info.u, info.v, info.point);
        Ray light_ray{info.point + info.normal * BIAS, light_dir};

        float k_shadow = get_k_shadow(light_ray);

        Vector3 reflected_dir = normalize(reflect(-light_dir, info.normal));

        float diffuse_factor = dot(light_dir, info.normal);
        Color3 tmp = hadamard(light_color, closest_color);
        Color3 diffuse_color = tmp * info.material.kd * fmaxf(diffuse_factor, 0.0f);

        float specular_factor = powf(dot(reflected_dir, view_dir), info.material.n);
        Color3 specular_color = light_color * info.material.ks * fmaxf(specular_factor, 0.0f);

        I += (diffuse_color + specular_color) * k_shadow;
    }

    return I;
}
} // namespace scene

// --------------------------------------------------------------------------------

#endif // SHAPES_HPP
