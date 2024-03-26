#ifndef MISC_HPP
#define MISC_HPP

#include "shapes.hpp"

// --------------------------------------------------------------------------------

namespace camera {
static Point3 position, lower_left;
static Vector3 horizontal, vertical;

static Vector3 w, u, v;
static Vector3 up;

static float width, height;

static void init(Point3 look_from, Point3 look_at, Vector3 world_up, float vfov, float aspect_ratio) {
    float theta = radians(vfov);
    float height = tanf(theta / 2.0f);

    height = 2.0f * height;
    width = aspect_ratio * height;

    up = world_up;

    w = normalize(look_from - look_at);
    u = normalize(cross(up, w));
    v = cross(w, u);

    position = look_from;
    horizontal = width * u;
    vertical = height * v;
    lower_left = position - (horizontal / 2.0f) - (vertical / 2.0f) - w;
}

static Ray get_ray(float x, float y) {
    return {position, lower_left + (x * horizontal) + (y * vertical) - position};
}

static void update_basis() {
    u = normalize(cross(up, w));
    v = cross(w, u);

    horizontal = width * u;
    vertical = height * v;
    lower_left = position - (horizontal / 2.0f) - (vertical / 2.0f) - w;
}

static void v_tilt(float degrees) {
    w = x_rotate(w, degrees);
    update_basis();
}

static void h_tilt(float degrees) {
    w = y_rotate(w, degrees);
    update_basis();
}

static void move(float x, float y, float z) {
    position = position + Vector3{x, y, z};
    lower_left = position - (horizontal / 2.0f) - (vertical / 2.0f) - w;
}
} // namespace camera

// --------------------------------------------------------------------------------

namespace parser {
static constexpr size_t max_power = 20;

static const double powers_of_10_pos[max_power] = {
    1.0e0,  1.0e1,  1.0e2,  1.0e3,  1.0e4,  1.0e5,  1.0e6,  1.0e7,  1.0e8,  1.0e9,
    1.0e10, 1.0e11, 1.0e12, 1.0e13, 1.0e14, 1.0e15, 1.0e16, 1.0e17, 1.0e18, 1.0e19,
};

static const double powers_of_10_neg[max_power] = {
    1.0e0,   1.0e-1,  1.0e-2,  1.0e-3,  1.0e-4,  1.0e-5,  1.0e-6,  1.0e-7,  1.0e-8,  1.0e-9,
    1.0e-10, 1.0e-11, 1.0e-12, 1.0e-13, 1.0e-14, 1.0e-15, 1.0e-16, 1.0e-17, 1.0e-18, 1.0e-19,
};

static char *get_file_content(const char *filepath) {
    FILE *file = fopen(filepath, "rb");
    if (!file) {
        panic("could not open file.");
    }
    defer { fclose(file); };

    fseek(file, 0, SEEK_END);
    size_t filesize = ftell(file);
    fseek(file, 0, SEEK_SET);

    char *ret = static_cast<char *>(malloc(filesize + 1));
    fread(ret, filesize, 1, file);
    ret[filesize] = '\0';

    return ret;
}

static constexpr bool is_whitespace(char c) {
    return c == ' ' || c == '\t' || c == '\r';
}

static constexpr bool is_newline(char c) {
    return c == '\n';
}

static constexpr bool is_digit(char c) {
    return c >= '0' && c <= '9';
}

static constexpr bool is_exponent(char c) {
    return c == 'e' || c == 'E';
}

static const char *skip_whitespace(const char *at) {
    while (is_whitespace(*at)) {
        ++at;
    }

    return at;
}

static const char *skip_line(const char *at) {
    while (!is_newline(*at++));

    return at;
}

static const char *parse_int(const char *at, int &dst) {
    int sign, num;

    at = skip_whitespace(at);

    switch (*at) {
    case '+': {
        sign = +1;
        ++at;
    } break;

    case '-': {
        sign = -1;
        ++at;
    } break;

    default: { sign = +1; } break;
    }

    num = 0;
    while (is_digit(*at)) {
        num = 10 * num + (*at++ - '0');
    }

    dst = sign * num;

    return at;
}

static const char *parse_float(const char *at, float &dst) {
    double sign, num, fra, div;
    size_t eval;
    const double *powers;

    at = skip_whitespace(at);

    switch (*at) {
    case '+': {
        sign = 1.0;
        ++at;
    } break;

    case '-': {
        sign = -1.0;
        ++at;
    } break;

    default: { sign = 1.0; } break;
    }

    num = 0.0;
    while (is_digit(*at)) {
        num = 10.0 * num + static_cast<double>(*at++ - '0');
    }

    if (*at == '.') {
        ++at;
    }

    fra = 0.0, div = 1.0;

    while (is_digit(*at)) {
        fra = 10.0 * fra + static_cast<double>(*at++ - '0');
        div *= 10.0;
    }

    num += fra / div;

    if (is_exponent(*at)) {
        ++at;

        switch (*at) {
        case '+': {
            powers = powers_of_10_pos;
            ++at;
        } break;

        case '-': {
            powers = powers_of_10_neg;
            ++at;
        } break;

        default: {
            powers = powers_of_10_pos;
        } break;
        }

        eval = 0;
        while (is_digit(*at)) {
            eval = 10 * eval + (*at++ - '0');
        }

        num *= (eval >= max_power) ? 0.0 : powers[eval];
    }

    dst = static_cast<float>(sign * num);

    return at;
}

static const char *parse_vertex(const char *at, Array<Point3> &verts) {
    Point3 vert;
    at = parse_float(at, vert.x);
    at = parse_float(at, vert.y);
    at = parse_float(at, vert.z);

    verts.push(vert);

    return skip_whitespace(at);
}

static const char *parse_face(const char *at, Array<size_t> &idxs) {
    int idx0, idx1, idx2;
    at = parse_int(at, idx0);
    at = parse_int(at, idx1);
    at = parse_int(at, idx2);

    idxs.push(static_cast<size_t>(idx0 - 1));
    idxs.push(static_cast<size_t>(idx1 - 1));
    idxs.push(static_cast<size_t>(idx2 - 1));

    return skip_whitespace(at);
}

static void parse_obj(const char *filepath, Material material) {
    const char *at = get_file_content(filepath);

    Array<Point3> verts{};
    defer { verts.destroy(); };

    Array<size_t> idxs{};
    defer { idxs.destroy(); };

    while (*at) {
        at = skip_whitespace(at);

        switch (*at) {
        case 'v': {
            ++at;

            switch (*at++) {
            case ' ':
            case '\t': { at = parse_vertex(at, verts); } break;

            default: --at; // Roll ++at back in case *at was a newline
            }
        } break;

        case 'f': {
            ++at;

            switch (*at++) {
            case ' ':
            case '\t': { at = parse_face(at, idxs); } break;

            default: --at; // Roll ++at back in case *at was a newline
            }
        } break;

        case '#': break;
        }

        at = skip_line(at);
    }

    if (material.emissive) {
        // TODO(paalf): remove the hardcoded 1.0f intensity
        scene::lights.push(Light::make(verts, idxs, material, 1.0f));
    } else {
        scene::meshes.push(Mesh::make(verts, idxs, material));
    }
}

struct Token {
    enum class Type : byte {
        EYE,
        ORTHO,
        SIZE,
        BACKGROUND,
        AMBIENT,
        LIGHT,
        NPATHS,
        TONEMAPPING,
        SEED,
        OBJECTQUADRIC,
        OBJECT,
        UNKNOWN
    };

    Type type;
    const char *lexeme;
};

static Token get_next_token(const char *&at) {
    at = skip_whitespace(at);

    if (!(*at)) {
        return {Token::Type::UNKNOWN, nullptr};
    }

    const char *start = at;
    at = skip_whitespace(at);

    size_t length = at - start;

    if (strncmp(start, "eye", length) == 0) {
        return {Token::Type::EYE, start};
    } else if (strncmp(start, "ortho", length) == 0) {
        return {Token::Type::ORTHO, start};
    } else if (strncmp(start, "size", length) == 0) {
        return {Token::Type::SIZE, start};
    } else if (strncmp(start, "background", length) == 0) {
        return {Token::Type::BACKGROUND, start};
    } else if (strncmp(start, "ambient", length) == 0) {
        return {Token::Type::AMBIENT, start};
    } else if (strncmp(start, "light", length) == 0) {
        return {Token::Type::LIGHT, start};
    } else if (strncmp(start, "npaths", length) == 0) {
        return {Token::Type::NPATHS, start};
    } else if (strncmp(start, "tonemapping", length) == 0) {
        return {Token::Type::TONEMAPPING, start};
    } else if (strncmp(start, "seed", length) == 0) {
        return {Token::Type::SEED, start};
    } else if (strncmp(start, "objectquadric", length) == 0) {
        return {Token::Type::OBJECTQUADRIC, start};
    } else if (strncmp(start, "object", length) == 0) {
        return {Token::Type::OBJECT, start};
    }

    return {Token::Type::UNKNOWN, start};
}

static const char *parse_vector3(const char *at, Vector3 &vec) {
    at = parse_float(at, vec.x);
    at = parse_float(at, vec.y);
    at = parse_float(at, vec.z);

    return skip_whitespace(at);
}

static void parse_sdl(const char *filepath) {
    const char *at = get_file_content(filepath);

    while (*at) {
        Token token = get_next_token(at);

        switch (token.type) {
        case Token::Type::EYE: {
            parse_vector3(at, camera::position);
        } break;

        case Token::Type::ORTHO: {

        } break;

        case Token::Type::SIZE: {
            
        } break;

        case Token::Type::BACKGROUND: {

        } break;

        case Token::Type::AMBIENT: {

        } break;

        case Token::Type::LIGHT: {

        } break;

        case Token::Type::NPATHS: {

        } break;

        case Token::Type::TONEMAPPING: {

        } break;

        case Token::Type::SEED: {

        } break;

        case Token::Type::OBJECTQUADRIC: {

        } break;

        case Token::Type::OBJECT: {

        } break;

        case Token::Type::UNKNOWN: {

        } break;
        }
    }
}
} // namespace parser

// --------------------------------------------------------------------------------

namespace renderer {
static size_t img_width, img_height;
static float aspect_ratio;

inline static void set_resolution(size_t hres, size_t vres) {
    img_width = hres, img_height = vres;
    aspect_ratio = static_cast<float>(hres) / vres;
}

struct Pixel {
    byte r, g, b;

    void paint(Color3 color, size_t num_samples) {
        r = static_cast<byte>(256 * clampf(sqrtf(color.r / num_samples), 0.0f, 0.999f));
        g = static_cast<byte>(256 * clampf(sqrtf(color.g / num_samples), 0.0f, 0.999f));
        b = static_cast<byte>(256 * clampf(sqrtf(color.b / num_samples), 0.0f, 0.999f));
    }
};

static constexpr size_t samples_per_pixel = 100, max_depth = 10;

static Color3 shade(Ray ray, size_t depth) {
    if (depth == max_depth) {
        return {};
    }

    Intersection info = scene::closest_intersection(ray, EPSILON32, INFINITY32);

    if (info.intersected) {
        if (info.material.emissive) {
            return info.material.albedo.get_value(info.u, info.v, info.point);
        }

        Color3 color = scene::phong_shade(ray, info);

        float kd = info.material.kd, ks = info.material.ks, kt = info.material.kt;
        float ktot = kd + ks + kt;
        float rand_val = randf(0.0f, ktot);

        Ray new_ray;
        if (rand_val < kd) {
            float r1 = randf(0.0f, 1.0f), r2 = randf(0.0f, 1.0f);
            float phi = acosf(sqrtf(r1)), theta = 2 * PI32 * r2;

            new_ray.origin = info.point;
            new_ray.direction = {
                sinf(phi) * cosf(theta),
                sinf(phi) * sinf(theta),
                cosf(phi)
            };
        } else if (rand_val < kd + ks) {
            new_ray = reflect(ray, info.point, info.normal);

            /*
            Vector3 light_dir = scene::lights[0].centroid - info.point;

            new_ray.origin = info.point;
            new_ray.direction = reflect(-light_dir, info.normal);
            */
        } else {
            bool total_reflection = false;

            new_ray = refract(ray, info.point, info.normal, info.material.refractive_index,
                              info.front_face, total_reflection);
        }

        float k = fmaxf(fmaxf(kd, ks), kt);

        return (color + shade(new_ray, depth + 1)) * k;
    }

    return scene::background_color;
}

static void render() {
    size_t img_res = img_width * img_height;

    Pixel pixels[img_res];
    size_t num_painted = 0;

    for (size_t y = 0; y < img_height; ++y) {
        for (size_t x = 0; x < img_width; ++x) {
            Color3 pixel_color{};

            for (size_t _ = 0; _ < samples_per_pixel; ++_) {
                float hcoef = (x + randf()) / (img_width - 1);
                float vcoef = (y + randf()) / (img_height - 1);

                Ray ray = camera::get_ray(hcoef, vcoef);

                pixel_color = pixel_color + shade(ray, 0);
            }

            pixels[num_painted++].paint(pixel_color, samples_per_pixel);

            fprintf(stderr, "\rProgress: %zu%%", 100 * num_painted / img_res);
            fflush(stderr);
        }
    }
    fprintf(stderr, "\n");

    stbi_flip_vertically_on_write(1);
    stbi_write_png("img/image.png", img_width, img_height, 3, pixels, img_width * sizeof(Pixel));
}
} // namespace renderer

// --------------------------------------------------------------------------------

#endif // MISC_HPP
