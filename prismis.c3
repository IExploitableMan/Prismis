module prismis_c3;

import std::io;
import std::math;
import std::time;
import std::thread;
import std::atomic;
import sdl;

struct Segment {
    usz x, y;
}

struct RenderJob {
    uint* buffer;
    Camera camera;
    Segment[] segments;
    int* next_segment;
    uint seed;
}

struct Camera {
    float[<3>] position;
    float[<3>] forward;
    float[<3>] right;
    float[<3>] up;
    float scale;
}
fn Camera camera_new(float[<3>] position, float yaw, float pitch) {
    float[<3>] forward = {
        math::cos(pitch) * math::sin(yaw),
        math::sin(pitch),
        math::cos(pitch) * math::cos(yaw)
    };
    float[<3>] temp = {0, 1, 0};
    float[<3>] right = temp.cross(forward);
    float[<3>] up = forward.cross(right);
    
    return {
        position,
        forward,
        right,
        up,
        math::tan(FOV / 2)
    };
}

enum MaterialType 
{
    LAMBERTIAN,
    SPECULAR,  // Metal
    DIELECTRIC // Glass
}

struct Material {
    MaterialType type;
    float[<3>] color;
    float roughness;
    float ior;
}

interface IObject 
{
    fn float intersect(float[<3>] ro, float[<3>] rd);
    fn Material* get_material();
    fn float[<3>] get_normal(float[<3>] t); 
}
struct Object (IObject)
{
    float[<3>] position;
    Material material;
}
fn float Object.intersect(Object* this, float[<3>] ro, float[<3>] rd) @dynamic
{
    return -1.;
}
fn Material* Object.get_material(Object* this) @dynamic
{
    return &this.material;
}
fn float[<3>] Object.get_normal(Object* this, float[<3>] t) @dynamic
{
    return {0, 1, 0};
}

struct Cube
{
    inline Object object;
    float[<3>] min;
    float[<3>] max;
}
fn float Cube.intersect(Cube* this, float[<3>] ro, float[<3>] rd) @dynamic
{
    float tmin = (this.min.x - ro.x) / rd.x;
    float tmax = (this.max.x - ro.x) / rd.x;
    if (tmin > tmax)
    {
        float tmp = tmin;
        tmin = tmax;
        tmax = tmp;
    }

    float tymin = (this.min.y - ro.y) / rd.y;
    float tymax = (this.max.y - ro.y) / rd.y;
    if (tymin > tymax)
    {
        float tmp = tymin;
        tymin = tymax;
        tymax = tmp;
    }

    if ((tmin > tymax) || (tymin > tmax)) return -1;

    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;

    float tzmin = (this.min.z - ro.z) / rd.z;
    float tzmax = (this.max.z - ro.z) / rd.z;
    if (tzmin > tzmax)
    {
        float tmp = tzmin;
        tzmin = tzmax;
        tzmax = tmp;
    }

    if ((tmin > tzmax) || (tzmin > tmax)) return -1.;

    if (tzmin > tmin) tmin = tzmin;
    if (tzmax < tmax) tmax = tzmax;

    return tmin > 0. ? tmin : tmax;
}
fn float[<3>] Cube.get_normal(Cube* this, float[<3>] t) @dynamic
{
    float[<3>] center = (this.min + this.max) * 0.5;
    float[<3>] half_size = (this.max - this.min) * 0.5;
    float[<3>] local = t - center;
    
    float[<3>] d = local / half_size;
    float[<3>] abs_d = { math::abs(d.x), math::abs(d.y), math::abs(d.z) };
    
    if (abs_d.x > abs_d.y && abs_d.x > abs_d.z) 
    {
        return { d.x > 0 ? 1 : -1, 0, 0 };
    }
    else if (abs_d.y > abs_d.z) 
    {
        return { 0, d.y > 0 ? 1 : -1, 0 };
    }
    else 
    {
        return { 0, 0, d.z > 0 ? 1 : -1 };
    }
}

struct Sphere
{
    inline Object object;
    float radius;
}
fn float Sphere.intersect(Sphere* this, float[<3>] ro, float[<3>] rd) @dynamic
{
    float[<3>] oc = ro - this.position;
    float b = oc.dot(rd);
    float c = oc.dot(oc) - this.radius * this.radius;
    float d = b * b - c;
    if (d < 0.) return -1.;
    return -b - math::sqrt(d);
}
fn float[<3>] Sphere.get_normal(Sphere* this, float[<3>] t) @dynamic
{
    return (t - this.position).normalize();
}

struct Plane
{
    inline Object object;
    float[<3>] normal;
}
fn float Plane.intersect(Plane* this, float[<3>] ro, float[<3>] rd) @dynamic
{
    float denom = rd.dot(this.normal);
    if (math::abs(denom) < math::FLOAT_EPSILON) return -1.;
    float t = (this.position - ro).dot(this.normal) / denom;
    return t > 0. ? t : -1.;
}
fn float[<3>] Plane.get_normal(Plane* this, float[<3>] t) @dynamic
{
    return this.normal;
}

// TODO: mesh

fn float[<3>] trace(float[<3>] ro, float[<3>] rd, ushort depth, uint* seed)
{
    if (depth > MAX_DEPTH) return {}; 
    
    float t_min = float.inf;
    IObject hit_obj = null;

    foreach (obj : scene)
    {
        float t = obj.intersect(ro, rd);
        if (t > 0 && t < t_min)
        {
            t_min = t;
            hit_obj = obj;
        }
    }

    if (!hit_obj) return SPACE;
    
    float[<3>] p = ro + rd * t_min;
    float[<3>] n = hit_obj.get_normal(p);
    Material* material = hit_obj.get_material();

    float[<3>] p_eps = p + n * EPS;
    float[<3>] color;
    float[<3>] light_position = {2, 5, -2};
    float[<3>] light_color = {1, 1, 1}; // TODO: light

    switch (material.type)
    {
    case LAMBERTIAN:
        float[<3>] new_dir = random_hemisphere(n, seed);
        float[<3>] indirect = material.color * trace(p_eps, new_dir, depth + 1, seed);

        float[<3>] to_light = (light_position - p).normalize();
        float dot_nl = math::max(0f, n.dot(to_light));

        bool in_shadow = false;
        foreach (obj : scene)
        {
            float t_shadow = obj.intersect(p_eps, to_light);
            if (t_shadow > 0)
            {
                in_shadow = true;
                break;
            }
        }

        float[<3>] direct = in_shadow ? {} : material.color * light_color * dot_nl;
        color = indirect + direct;
    case SPECULAR:
        float[<3>] reflection = rd.reflect(n);
        float[<3>] fuzz = random_in_unit_sphere(seed).normalize() * material.roughness;
        float[<3>] new_dir = (reflection + fuzz).normalize();
        color = material.color * trace(p_eps, new_dir, depth + 1, seed);
        break;
    case DIELECTRIC:
        float etai_over_etat;
        float[<3>] n_out = n;
        float cos_theta = math::min(-rd.dot(n), 1f);
        float sin_theta = math::sqrt(1 - cos_theta * cos_theta);

        if (rd.dot(n) > 0)
        {
            n_out *= -1;
            etai_over_etat = material.ior;
        }
        else
        {
            etai_over_etat = 1f / material.ior;
        }

        bool cannot_refract = etai_over_etat * sin_theta > 1;
        float reflect_prob = schlick(cos_theta, material.ior);

        float[<3>] direction;
        if (cannot_refract || fprand(seed) < reflect_prob)
        {
            direction = rd.reflect(n_out);
        }
        else
        {
            direction = rd.refract(n_out, etai_over_etat);
        }

        color = trace(p + direction * EPS, direction, depth + 1, seed);
        break;
    }

    return color;
}

fn float schlick(float cos_theta, float idx)
{
    float r0 = (1 - idx) / (1 + idx);
    r0 = r0 * r0;
    return r0 + (1 - r0) * math::pow(1 - cos_theta, 5);
}

fn uint prand(uint* seed)
{
    *seed = (*seed * 1103515245 + 12345) & 0x7fffffff;
    return *seed;
}

fn float fprand(uint* seed)
{
    return (float)prand(seed) / 0x7fffffff;
}

fn float[<3>] random_hemisphere(float[<3>] n, uint* seed)
{
    float u1 = fprand(seed);
    float u2 = fprand(seed);

    float r = math::sqrt(u1);
    float theta = 2.f * math::PI * u2;

    float x = r * math::cos(theta);
    float y = r * math::sin(theta);
    float z = math::sqrt(1 - u1);

    float[<3>] up = math::abs(n.z) < (1 - EPS) ? {0, 0, 1} : {1, 0, 0};
    float[<3>] tangent = up.cross(n).normalize();
    float[<3>] bitangent = n.cross(tangent);

    return (tangent * x + bitangent * y + n * z).normalize();
}

fn float[<3>] random_in_unit_sphere(uint* seed)
{
    while (1)
    {
        float[<3>] p = {
            fprand(seed) * 2 - 1, 
            fprand(seed) * 2 - 1, 
            fprand(seed) * 2 - 1
        };
        if (p.dot(p) < 1) return p;
    }
}

fn void worker(void* arg) {
    RenderJob* job = arg;
    
    while (true) {
        usz id = (usz)atomic::fetch_add(job.next_segment, 1);
        if (id >= job.segments.len) break;
        
        Segment segment = job.segments[id];
        usz y_end = math::min(segment.y + SEGMENT_SIZE, (usz)HEIGHT);
        usz x_end = math::min(segment.x + SEGMENT_SIZE, (usz)WIDTH);
        
        for (usz y = segment.y; y < y_end; y++) {
            for (usz x = segment.x; x < x_end; x++) {
                float[<3>] color = {0, 0, 0};
                
                for (usz s = 0; s < SAMPLES; s++) {
                    float nx = 2 * (x + fprand(&job.seed)) / WIDTH - 1;
                    float ny = 1 - 2 * (y + fprand(&job.seed)) / HEIGHT;
                    
                    float[<3>] rd = (
                        job.camera.right * (nx * ASPECT * job.camera.scale) +
                        job.camera.up * (ny * job.camera.scale) +
                        job.camera.forward
                    ).normalize();
                    
                    color += trace(job.camera.position, rd, 0, &job.seed);
                }
                
                color *= 1f / SAMPLES;
                
                const float INV_GAMMA = 1 / 2.2;
                uint r = (uint)math::min(255f, math::pow(color[0], INV_GAMMA) * 255);
                uint g = (uint)math::min(255f, math::pow(color[1], INV_GAMMA) * 255);
                uint b = (uint)math::min(255f, math::pow(color[2], INV_GAMMA) * 255);
                
                job.buffer[y * WIDTH + x] = (r << 16) | (g << 8) | b;
            }
        }
    }
}


const int WIDTH = 960;
const int HEIGHT = 540;
const usz MAX_DEPTH = 5; // Maximum number of bounces per ray
const usz SAMPLES = 4; // Number of samples per pixel
const usz THREADS = 8;
const usz SEGMENT_SIZE = 16; // Size of image segment (tile) for work distribution among threads
const float ASPECT = ((float)WIDTH / (float)HEIGHT);

const float SPEED = 0.1;
const float SENSITIVITY = 0.002;
const float FOV = math::PI / 2;

const float[<3>] SPACE = {0.87, 0.99, 1};
const float EPS = 0.001;

Sphere sphere1 = {
    .object = {
        .position = {0, 0, 0},
        .material = {
            .type = LAMBERTIAN,
            .color = {1, 0.2, 0.2},
            .roughness = 0,
            .ior = 1
        }
    },
    .radius = 1
};

Sphere sphere2 = {
    .object = {
        .position = {-2.2, 0, 0},
        .material = {
            .type = SPECULAR,
            .color = {0.95, 0.95, 0.95},
            .roughness = 0,
            .ior = 1
        }
    },
    .radius = 1
};

Sphere sphere3 = {
    .object = {
        .position = {2.2, 0, 0},
        .material = {
            .type = DIELECTRIC,
            .color = {0.9, 0.95, 1},
            .roughness = 0,
            .ior = 1.5
        }
    },
    .radius = 1
};

Plane plane1 = {
    .object = {
        .position = {0, -1, 0},
        .material = {
            .type = LAMBERTIAN,
            .color = {0.8, 0.8, 0.8},
            .roughness = 0.3,
            .ior = 1
        }

// --safe=no --optlevel=max --unroll-loops=yes --slp-vectorize=yes --loop-vectorize=yes --merge-functions=yes --bounds-check=off --single-module=yes
// 
    },
    .normal = {0, 1, 0}
};

Cube cube1 = {
    .object = {
        .position = {0, 1, 3},
        .material = {
            .type = LAMBERTIAN,
            .color = {0.2, 0.8, 0.2},
            .roughness = 0.5,
            .ior = 1
        }
    },
    .min = {-0.5, 0.5, 2.5},
    .max = {0.5, 1.5, 3.5}
};

IObject[] scene = {
    &sphere1,
    &sphere2,
    &sphere3,
    &plane1,
    &cube1
};

fn int main()
{
    if (sdl::init(sdl::INIT_VIDEO) != 0)
    {
        io::printfn("SDL_Init Error: %s", sdl::getError());
        return 1;
    }

    Window* window = sdl::createWindow(
        "[C3] Prismis™",
        sdl::WINDOWPOS_CENTERED_MASK,
        sdl::WINDOWPOS_CENTERED_MASK,
        WIDTH,
        HEIGHT,
        (uint)sdl::WINDOW_SHOWN // Bravo casting
    );
    sdl::setRelativeMouseMode(true);
    if (!window)
    {
        sdl::quit();
        io::printfn("SDL_CreateWindow Error: %s", sdl::getError());
        return 1;
    }

    Renderer* renderer = sdl::createRenderer(window, -1, (uint)sdl::RENDERER_SOFTWARE);
    if (!renderer)
    {
        sdl::destroyWindow(window);
        sdl::quit();
        io::printfn("SDL_CreateRenderer Error: %s", sdl::getError());
        return 1;
    }

    Texture* texture = sdl::createTexture(
        renderer,
        (uint)sdl::PIXELFORMAT_XRGB8888,
        sdl::TextureAccess.TEXTUREACCESS_STREAMING.ordinal, // what?...
        WIDTH,
        HEIGHT
    ); // Resizing for loosers
    if (!texture)
    {
        sdl::destroyRenderer(renderer);
        sdl::destroyWindow(window);
        sdl::quit();
        io::printfn("SDL_CreateTexture Error: %s", sdl::getError());
        return 1;
    }

    uint* buffer = malloc(WIDTH * HEIGHT * uint.sizeof);
    if (!buffer)
    {
        sdl::destroyRenderer(renderer);
        sdl::destroyWindow(window);
        sdl::quit();
        io::printfn("malloc failed!");
        return 1;
    }

    float[<3>] camera = {0, 0, -5};
    float yaw;
    float pitch;

    usz num_segments = ((WIDTH + SEGMENT_SIZE - 1) / SEGMENT_SIZE) * ((HEIGHT + SEGMENT_SIZE - 1) / SEGMENT_SIZE);
    Segment* segments = malloc(num_segments * Segment.sizeof);
    usz idx = 0;
    for (usz sy = 0; sy < HEIGHT; sy += SEGMENT_SIZE) {
        for (usz sx = 0; sx < WIDTH; sx += SEGMENT_SIZE) {
            segments[idx++] = { sx, sy };
        }
    }

    int segment_id;
    uint global_seed = (uint)std::time::now().to_seconds();
    RenderJob job = {
        buffer,
        {},
        segments[:num_segments],
        &segment_id,
        0
    };
    thread::Thread[THREADS] threads;

    sdl::Event event;
    bool[(usz)sdl::NUM_SCANCODES] key_states;
    Time t = std::time::now();

    while EXIT: (true)
    {
        while (sdl::pollEvent(&event))
        {
            switch (event.type)
            {
            case sdl::QUIT:
                break EXIT;
            case sdl::KEYDOWN:
                key_states[(usz)event.key.keysym.scancode] = true;
                break;
            case sdl::KEYUP:
                key_states[(usz)event.key.keysym.scancode] = false;
                break;
            }
        }

        float[<3>] forward = {math::sin(yaw), 0, math::cos(yaw)};
        float[<3>] right = {math::cos(yaw), 0, -math::sin(yaw)};

        int[<2>] mouse_movement;
        sdl::getRelativeMouseState(&mouse_movement.x, &mouse_movement.y);
        yaw += mouse_movement.x * SENSITIVITY;
        pitch -= mouse_movement.y * SENSITIVITY;

        if (pitch > math::PI_2 - math::FLOAT_EPSILON) pitch = math::PI_2 - math::FLOAT_EPSILON;
        if (pitch < -math::PI_2 + math::FLOAT_EPSILON) pitch = -math::PI_2 + math::FLOAT_EPSILON;

        if (key_states[(usz)sdl::SCANCODE_W]) camera = camera + forward * SPEED;
        if (key_states[(usz)sdl::SCANCODE_S]) camera = camera - forward * SPEED;
        if (key_states[(usz)sdl::SCANCODE_A]) camera = camera - right * SPEED;
        if (key_states[(usz)sdl::SCANCODE_D]) camera = camera + right * SPEED;

        segment_id = 0;
        job.seed = prand(&global_seed);
        job.camera = camera_new(camera, yaw, pitch);
        foreach (&thread : threads) 
        {
            thread.create((thread::ThreadFn)&worker, &job)!!;
        }
        foreach (thread : threads) thread.join()!!;   

        sdl::updateTexture(texture, null, buffer, WIDTH * uint.sizeof);
        sdl::renderCopy(renderer, texture, null, null);

        sdl::renderPresent(renderer);

        io::printfn("Frame time: %.2f ms", 1 / (std::time::now().to_seconds() - t.to_seconds()));
        t = std::time::now();
    }
    
    free(segments);
    free(buffer);
    sdl::destroyTexture(texture);
    sdl::destroyRenderer(renderer);
    sdl::destroyWindow(window);
    sdl::quit();
    return 0;
}