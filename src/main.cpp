#include "misc.hpp"

#include <time.h>

int main(int argc, char *argv[]) {
    if (argc != 2) {
        return 1;
    }

    timespec begin;
    timespec_get(&begin, TIME_UTC);

    Point3 look_from, look_at;

    defer { scene::destroy(), material::earth.albedo.destroy(); };

    char scene_option;
    if (strlen(argv[1]) == 2) {
        scene_option = argv[1][1];
    } else {
        const char *last_dot = strrchr(argv[1], '.');
        if (!last_dot || strcmp(last_dot, ".sdl") != 0) {
            return 1;
        }

        scene_option = 'f'; // 'f'ile scene option
    }

    switch (scene_option) {
    case 's': {
        renderer::set_resolution(400, 225);

        look_from = {0.0f, 0.0f,  4.0f};
        look_at   = {0.0f, 0.0f, -1.0f};

        camera::init(look_from, look_at, {0.0f, 1.0f, 0.0f}, 20.0f, renderer::aspect_ratio);

        scene::background_color = color::light_blue;
        scene::ambient_factor = 0.5f;

        scene::planes.push(Plane::make({0.0f, -0.5f, 0.0f}, {0.0f, -0.5f, 0.0f}, material::grass));

        scene::spheres.push(Sphere::make({ 0.0f,  1.5f,  1.5f}, 1.0f, material::glowing));
        scene::spheres.push(Sphere::make({ 0.0f, -0.2f, -1.0f}, 0.3f, material::rubber));
        scene::spheres.push(Sphere::make({-0.6f, -0.1f, -1.3f}, 0.4f, material::steel));
        scene::spheres.push(Sphere::make({ 0.4f, -0.3f, -0.7f}, 0.2f, material::glass));

        auto tetrahedron_verts = Array<Point3>::make(4);
        defer { tetrahedron_verts.destroy(); };

        tetrahedron_verts.push({0.3f, -0.5f, -1.5f});
        tetrahedron_verts.push({1.3f, -0.5f, -1.5f});
        tetrahedron_verts.push({0.8f, -0.5f, -0.5f});
        tetrahedron_verts.push({0.8f,  0.5f, -1.0f});

        auto tetrahedron_idxs = Array<size_t>::make(4 * 3);
        defer { tetrahedron_idxs.destroy(); };

        tetrahedron_idxs.push(0), tetrahedron_idxs.push(1), tetrahedron_idxs.push(2);
        tetrahedron_idxs.push(0), tetrahedron_idxs.push(1), tetrahedron_idxs.push(3);
        tetrahedron_idxs.push(0), tetrahedron_idxs.push(2), tetrahedron_idxs.push(3);
        tetrahedron_idxs.push(1), tetrahedron_idxs.push(2), tetrahedron_idxs.push(3);

        scene::meshes.push(Mesh::make(tetrahedron_verts, tetrahedron_idxs, material::gold));

        //scene::meshes.push(Mesh::make({0.2f, 0.2f, -1.5f}, 0.3f, 0.1f, 8, 4, material::gamecube));

        // Translate plane
        //scene::planes[0].move(0.0f, -0.5f, 0.0f);

        // Translate sphere (rubber)
        //scene::spheres[0].move(1.0f, 1.0f, -2.0f);

        // Translate mesh (torus)
        //scene::meshes[1].move(-0.2f, -0.2f, 1.5f);

        // Left front view
        //camera::move(-4.0f, 0.0f, -3.0f);
        //camera::h_tilt(-60.0f);

        // Right back view
        //camera::move(4.0f, 0.0f, -7.0f);
        //camera::h_tilt(120.0f);

        // Top view
        //camera::move(0.0f, 8.0f, -5.0f);
        //camera::v_tilt(-90.0f);

        // Far view
        //camera::move(0.0f, 0.0f, 7.0f);
    } break;

    case 'c': {
        renderer::set_resolution(600, 600);

        look_from = {278.0f, 278.0f, -765.0f};
        look_at   = {278.0f, 278.0f,    0.0f};

        camera::init(look_from, look_at, {0.0f, 1.0f, 0.0f}, 40.0f, renderer::aspect_ratio);

        scene::background_color = color::black;
        scene::ambient_factor = 0.5f;

        auto quad_verts = Array<Point3>::make(4);
        defer { quad_verts.destroy(); };

        auto quad_idxs = Array<size_t>::make(2 * 3);
        defer { quad_idxs.destroy(); };

        quad_idxs.push(0), quad_idxs.push(1), quad_idxs.push(2);
        quad_idxs.push(2), quad_idxs.push(3), quad_idxs.push(0);

        // Left wall
        quad_verts.push({555.0f,   0.0f,   0.0f});
        quad_verts.push({555.0f, 555.0f,   0.0f});
        quad_verts.push({555.0f, 555.0f, 555.0f});
        quad_verts.push({555.0f,   0.0f, 555.0f});

        scene::meshes.push(Mesh::make(quad_verts, quad_idxs, material::girly));

        quad_verts.clear();

        // Right wall
        quad_verts.push({0.0f,   0.0f,   0.0f});
        quad_verts.push({0.0f, 555.0f,   0.0f});
        quad_verts.push({0.0f, 555.0f, 555.0f});
        quad_verts.push({0.0f,   0.0f, 555.0f});

        scene::meshes.push(Mesh::make(quad_verts, quad_idxs, material::jeans));

        quad_verts.clear();

        // Back wall
        quad_verts.push({  0.0f,   0.0f, 555.0f});
        quad_verts.push({555.0f,   0.0f, 555.0f});
        quad_verts.push({555.0f, 555.0f, 555.0f});
        quad_verts.push({  0.0f, 555.0f, 555.0f});

        scene::meshes.push(Mesh::make(quad_verts, quad_idxs, material::chalk));

        quad_verts.clear();

        // Floor
        quad_verts.push({  0.0f, 0.0f,   0.0f});
        quad_verts.push({555.0f, 0.0f,   0.0f});
        quad_verts.push({555.0f, 0.0f, 555.0f});
        quad_verts.push({  0.0f, 0.0f, 555.0f});

        scene::meshes.push(Mesh::make(quad_verts, quad_idxs, material::chalk));

        quad_verts.clear();

        // Ceiling
        quad_verts.push({555.0f, 555.0f, 555.0f});
        quad_verts.push({  0.0f, 555.0f, 555.0f});
        quad_verts.push({  0.0f, 555.0f,   0.0f});
        quad_verts.push({555.0f, 555.0f,   0.0f});

        scene::meshes.push(Mesh::make(quad_verts, quad_idxs, material::chalk));

        quad_verts.clear();

        // Light
        quad_verts.push({343.0f, 554.0f, 332.0f});
        quad_verts.push({213.0f, 554.0f, 332.0f});
        quad_verts.push({213.0f, 554.0f, 227.0f});
        quad_verts.push({343.0f, 554.0f, 227.0f});

        scene::lights.push(Light::make(quad_verts, quad_idxs, material::glowing, 1.0f));

        auto tetrahedron_verts = Array<Point3>::make(4);
        defer { tetrahedron_verts.destroy(); };

        tetrahedron_verts.push({180.0f,   0.0f, 350.0f});
        tetrahedron_verts.push({510.0f,   0.0f, 350.0f});
        tetrahedron_verts.push({330.0f,   0.0f, 200.0f});
        tetrahedron_verts.push({330.0f, 300.0f, 275.0f});

        auto tetrahedron_idxs = Array<size_t>::make(4 * 3);
        defer { tetrahedron_idxs.destroy(); };

        tetrahedron_idxs.push(0), tetrahedron_idxs.push(1), tetrahedron_idxs.push(2);
        tetrahedron_idxs.push(0), tetrahedron_idxs.push(1), tetrahedron_idxs.push(3);
        tetrahedron_idxs.push(0), tetrahedron_idxs.push(2), tetrahedron_idxs.push(3);
        tetrahedron_idxs.push(1), tetrahedron_idxs.push(2), tetrahedron_idxs.push(3);

        scene::meshes.push(Mesh::make(tetrahedron_verts, tetrahedron_idxs, material::mirror));

        scene::spheres.push(Sphere::make({130.0f, 100.0f, 190.0f}, 100.0f, material::glass));
        scene::spheres.push(Sphere::make({450.0f,  50.0f, 210.0f},  50.0f, material::earth));
    } break;

    case 'v': {
        renderer::set_resolution(400, 225);

        look_from = {-2.0f, 2.0f,  1.0f};
        look_at   = { 0.0f, 0.0f, -1.0f};

        camera::init(look_from, look_at, {0.0f, 1.0f, 0.0f}, 20.0f, renderer::aspect_ratio);

        scene::background_color = color::pink;
        scene::ambient_factor = 0.5f;

        scene::planes.push(Plane::make({0.0f, -0.5f, 0.0f}, {0.0f, -0.5f, 0.0f}, material::chess));

        scene::spheres.push(Sphere::make({ 0.0f, 0.0f, -1.0f},  0.5f, material::aqua));
        scene::spheres.push(Sphere::make({ 1.0f, 0.0f, -1.0f},  0.5f, material::mirror));
        scene::spheres.push(Sphere::make({-1.0f, 0.0f, -1.0f},  0.5f, material::glass));
        scene::spheres.push(Sphere::make({-1.0f, 0.0f, -1.0f}, -0.4f, material::glass));

        scene::lights.push(Light::make({0.0f, 1.5f,  1.5f},  1.0f, material::glowing, 1.0f));
    } break;

    //case 'f': { parser::parse_sdl(argv[1]); } break;

    // TMP
    case 'f': {
        renderer::set_resolution(200, 200);

        look_from = {0.0f, 0.0f,  5.7f};
        look_at   = {0.0f, 0.0f, -1.0f};

        camera::init(look_from, look_at, {0.0f, 1.0f, 0.0f}, 20.0f, renderer::aspect_ratio);

        scene::background_color = color::black;
        scene::ambient_factor = 0.5f;

        parser::parse_obj("res/models/leftwall.obj", material::girly);
        parser::parse_obj("res/models/rightwall.obj", material::jeans);
        parser::parse_obj("res/models/floor.obj", material::chalk);
        parser::parse_obj("res/models/back.obj", material::chalk);
        parser::parse_obj("res/models/ceiling.obj", material::chalk);
        parser::parse_obj("res/models/luzcornell.obj", material::glowing);
        parser::parse_obj("res/models/cube1.obj", material::chalk);
        parser::parse_obj("res/models/cube2.obj", material::chalk);

        /*
        auto glass_ball = Quadric::make(
            1.0f, 1.0f, 1.0f,  0.0f,  0.0f,
            0.0f, -1.0f, 1.0f, 22.0f, 484.0f,
            material::glass
        );
        scene::quadrics.push(glass_ball);
        */
    } break;

    default: return 1;
    }

    renderer::render();

    timespec end;
    timespec_get(&end, TIME_UTC);
    fprintf(stderr, "Execution time: %lf seconds.\n", (end.tv_sec - begin.tv_sec) + (end.tv_nsec - begin.tv_nsec) * 1e-9);

    return 0;
}