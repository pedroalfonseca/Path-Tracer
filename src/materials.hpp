#ifndef MATERIALS_HPP
#define MATERIALS_HPP

#include "core.hpp"
#include "math.hpp"

#define STB_FAILURE_USERMSG
#define STB_IMAGE_IMPLEMENTATION
#include "../vendor/stb_image/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../vendor/stb_image/stb_image_write.h"

// --------------------------------------------------------------------------------

struct Solid_Color {
    Color3 color;

    static Solid_Color make(Color3 color) {
        return {color};
    }

    Color3 get_value(float u, float v, Point3 point) const {
        return color;
    }
};

// --------------------------------------------------------------------------------

struct Checker {
    Color3 odd_color, even_color;

    static Checker make(Color3 odd_color, Color3 even_color) {
        return {odd_color, even_color};
    }

    Color3 get_value(float u, float v, Point3 point) const {
        float sin_product = sinf(10.0f * point.x) * sinf(10.0f * point.y) * sinf(10.0f * point.z);
        return sin_product < 0.0f ? odd_color : even_color;
    }
};

// --------------------------------------------------------------------------------

struct Noise {
    Color3 base_color;
    float perlin_scale;

    static Noise make(Color3 base_color, float perlin_scale) {
        perlin::init();
        return {base_color, perlin_scale};
    }

    Color3 get_value(float u, float v, Point3 point) const {
        Point3 scaled_point = perlin_scale * point;
        return (base_color / 2.0f) * (1.0f + sinf(scaled_point.z + 10.0f * perlin::turb(scaled_point)));
    }
};

// --------------------------------------------------------------------------------

struct Image {
    static constexpr int bytes_per_pixel = 3;
    int bytes_per_scanline;

    int width, height;
    byte *data;

    static Image make(const char *filepath) {
        Image ret{};

        int n = bytes_per_pixel;
        ret.data = stbi_load(filepath, &ret.width, &ret.height, &n, bytes_per_pixel);
        ret.bytes_per_scanline = ret.width * bytes_per_pixel;

        if (!ret.data) {
            panic("could not load image file '%s'.", filepath);
        }

        return ret;
    }

    void destroy() {
        if (data) {
            stbi_image_free(data);
            data = nullptr;
        }

        width = height = 0;
    }

    Color3 get_value(float u, float v, Point3 point) const {
        if (height <= 0) {
            return {0.0f, 1.0f, 1.0f};
        }

        u = clampf(u, 0.0f, 1.0f);
        v = 1.0f - clampf(v, 0.0f, 1.0f);

        int i = static_cast<int>(u * width);
        int j = static_cast<int>(v * height);

        i = clampi(i, 0, width - 1);
        j = clampi(j, 0, height - 1);

        const byte *pixel_data = data + j * bytes_per_scanline + i * bytes_per_pixel;

        float scale = 1.0f / 255.0f;

        return {scale * pixel_data[0], scale * pixel_data[1], scale * pixel_data[2]};
    }
};

// --------------------------------------------------------------------------------

struct Texture {
    enum class Type : byte {
        SOLID_COLOR,
        CHECKER,
        NOISE,
        IMAGE
    };

    Type type;

    union {
        Solid_Color solid_color;
        Checker     checker;
        Noise       noise;
        Image       image;
    };

    static Texture make(Color3 color) {
        Texture ret{Type::SOLID_COLOR};
        ret.solid_color = Solid_Color::make(color);

        return ret;
    }

    static Texture make(Color3 odd_color, Color3 even_color) {
        Texture ret{Type::CHECKER};
        ret.checker = Checker::make(odd_color, even_color);

        return ret;
    }

    static Texture make(Color3 base_color, float perlin_scale) {
        Texture ret{Type::NOISE};
        ret.noise = Noise::make(base_color, perlin_scale);

        return ret;
    }

    static Texture make(const char *filepath) {
        Texture ret{Type::IMAGE};
        ret.image = Image::make(filepath);

        return ret;
    }

    void destroy() {
        if (type == Texture::Type::IMAGE) {
            image.destroy();
        }
    }

    Color3 get_value(float u, float v, Point3 point) const {
        switch (type) {
        case Type::SOLID_COLOR: return solid_color.get_value(u, v, point);
        case Type::CHECKER:     return checker.get_value(u, v, point);
        case Type::NOISE:       return noise.get_value(u, v, point);
        case Type::IMAGE:       return image.get_value(u, v, point);
        default:                return {};
        }
    }
};

// --------------------------------------------------------------------------------

struct Ray {
    Point3 origin;
    Vector3 direction;

    Point3 at(float t) const {
        return origin + (t * direction);
    }
};

Ray reflect(Ray incident, Point3 point, Vector3 normal) {
    Point3 reflected_origin = point + normal * BIAS;
    Vector3 reflected_direction = reflect(incident.direction, normal);

    return {reflected_origin, reflected_direction};
}

Ray refract(Ray incident, Point3 point, Vector3 normal, float etai_over_etat,
            bool front_face, bool &total_reflection) {
    float refraction_ratio = front_face ? (1.0f / etai_over_etat) : etai_over_etat;

    Vector3 refracted_origin = point - normal * BIAS;
    Vector3 refracted_dir = refract(incident.direction, normal, refraction_ratio);

    if (!is_zero(refracted_dir)) {
        return {refracted_origin, refracted_dir};
    }

    total_reflection = true;

    return reflect(incident, point, normal);
}

// --------------------------------------------------------------------------------

struct Material {
    Texture albedo;
    bool emissive;
    float ka, kd, ks, kt, n;
    float refractive_index;
};

// --------------------------------------------------------------------------------

struct Intersection {
    bool intersected, front_face;

    Point3 point;
    Vector3 normal;

    float t, u, v;

    Material material;

    void set_normal(Ray ray, Vector3 outward_normal) {
        front_face = dot(ray.direction, outward_normal) < 0.0f;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

// --------------------------------------------------------------------------------

namespace color {
static constexpr Color3 black       {0.0f, 0.0f, 0.0f};
static constexpr Color3 light_gray  {0.8f, 0.8f, 0.8f};
static constexpr Color3 white       {1.0f, 1.0f, 1.0f};

static constexpr Color3 crimson     {0.5f, 0.1f, 0.1f};
static constexpr Color3 pink        {1.0f, 0.4f, 0.8f};
static constexpr Color3 salmon      {0.9f, 0.4f, 0.4f};
static constexpr Color3 amber       {0.8f, 0.6f, 0.2f};
static constexpr Color3 olive       {0.3f, 0.4f, 0.1f};
static constexpr Color3 turquoise   {0.0f, 0.5f, 0.3f};
static constexpr Color3 indigo      {0.1f, 0.0f, 0.5f};
static constexpr Color3 light_indigo{0.4f, 0.4f, 0.8f};
static constexpr Color3 light_blue  {0.5f, 0.7f, 1.0f};
} // namespace color

// --------------------------------------------------------------------------------

namespace material {
static const Material glowing {Texture::make(color::white),                true};
                                                                               //  ka    kd    ks    kt    n     ri
static const Material chalk   {Texture::make(color::white),                false, 0.3f, 0.7f, 0.0f, 0.0f, 5.0f, 0.0f};
static const Material rubber  {Texture::make(color::crimson),              false, 0.1f, 0.7f, 0.0f, 0.0f, 3.0f, 0.0f};
static const Material girly   {Texture::make(color::salmon),               false, 0.3f, 0.7f, 0.0f, 0.0f, 5.0f, 0.0f};
static const Material aqua    {Texture::make(color::turquoise),            false, 0.1f, 0.7f, 0.0f, 0.0f, 3.0f, 0.0f};
static const Material gamecube{Texture::make(color::indigo),               false, 0.1f, 0.7f, 0.0f, 0.0f, 3.0f, 0.0f};
static const Material jeans   {Texture::make(color::light_indigo),         false, 0.3f, 0.7f, 0.0f, 0.0f, 5.0f, 0.0f};

static const Material steel   {Texture::make(color::light_gray),           false, 0.1f, 0.6f, 0.6f, 0.0f, 3.0f, 0.0f};
static const Material mirror  {Texture::make(color::light_gray),           false, 0.1f, 0.3f, 0.6f, 0.0f, 3.0f, 0.0f};
static const Material gold    {Texture::make(color::amber),                false, 0.1f, 0.6f, 0.6f, 0.0f, 3.0f, 0.0f};

static const Material glass   {Texture::make(color::white),                false, 0.1f, 0.0f, 0.3f, 0.9f, 3.0f, 1.5f};

static const Material chess   {Texture::make(color::black, color::white),  false, 0.1f, 0.6f, 0.3f, 0.0f, 3.0f, 0.0f};

static const Material marble  {Texture::make(color::white, 4.0f),          false, 0.1f, 0.6f, 0.3f, 0.0f, 3.0f, 0.0f};
static const Material grass   {Texture::make(color::olive, 1.0f),          false, 0.1f, 0.6f, 0.0f, 0.0f, 3.0f, 0.0f};

static       Material earth   {Texture::make("res/textures/earthmap.jpg"), false, 0.1f, 0.6f, 0.0f, 0.0f, 3.0f, 0.0f};
} // namespace material

// --------------------------------------------------------------------------------

#endif // MATERIALS_HPP