#include <stdatomic.h>
#include <pthread.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>
#define LA_IMPLEMENTATION
#include <la.h>

// Before invention of the constexpr
#define WIDTH 960
#define HEIGHT 540
#define MAX_DEPTH 8 // Maximum number of bounces per ray
#define SAMPLES 4 // Number of samples per pixel
#define THREADS 8
#define SEGMENT_SIZE 128 // Size of image segment (tile) for work distribution among threads
#define ASPECT ((float)WIDTH / (float)HEIGHT)

#define PI 3.14159265358979323846 // Literally idk why M_PI wont work
#define SCENE_SIZE 5

#define SPEED 0.1f
#define SENSITIVITY 0.002f
#define FOV PI / 2.f

typedef enum
{
    MAT_LAMBERTIAN,
    MAT_SPECULAR,  // Metal
    MAT_DIELECTRIC // Glass
} MaterialType;

typedef enum
{
    OBJ_SPHERE,
    OBJ_PLANE,
    OBJ_CUBE
    // Add OBJ_MESH later
} ObjectType;

typedef struct
{
    V3f color;
    MaterialType type;
    float roughness;
    float ior;
   SDL_Surface* texture;
} Material;

typedef struct
{
    ObjectType type;
    V3f position;
    union
    {
        float radius; // OBJ_SPHERE
        V3f normal;   // OBJ_PLANE
        struct
        {
            V3f min;
            V3f max;
        } cube; // OBJ_CUBE
    };
    Material material;
} Object;

typedef struct
{
    size_t x;
    size_t y;
} Segment;

typedef struct
{
    uint32_t *buffer;
    V3f camera;
    Segment *segments;
    size_t num_segments;
    atomic_int *segment_id;
    float yaw;
    float pitch;
    V3f right;
} Thread;

Object scene[SCENE_SIZE] = {
    {OBJ_SPHERE, {0, 0, 0}, .radius = 1.f, .material = {.color = {1, 0.2, 0.2}, .type = MAT_LAMBERTIAN, .roughness = 0.f, .ior = 1.f}},
    {OBJ_SPHERE, {-2.2f, 0, 0}, .radius = 1.f, .material = {.color = {0.95, 0.95, 0.95}, .type = MAT_SPECULAR, .roughness = 0.f, .ior = 1.f}},
    {OBJ_SPHERE, {2.2f, 0, 0}, .radius = 1.f, .material = {.color = {0.9, 0.95, 1}, .type = MAT_DIELECTRIC, .roughness = 0.f, .ior = 1.5f}},
    {OBJ_PLANE, {0, -1, 0}, .normal = {0, 1, 0}, .material = {.color = {0.8, 0.8, 0.8}, .type = MAT_LAMBERTIAN, .roughness = 0.3f, .ior = 1.f}},
    {OBJ_CUBE, {0, 1, 3}, .cube = {.min = {-0.5f, 0.5f, 2.5f}, .max = {0.5f, 1.5f, 3.5f}}, .material = {.color = {0.2, 0.8, 0.2}, .type = MAT_LAMBERTIAN, .roughness = 0.5f, .ior = 1.f}}};

static uint32_t prand(uint32_t *seed)
{
    *seed = (*seed * 1103515245 + 12345) & 0x7fffffff;
    return *seed;
}

float fprand(uint32_t *seed)
{
    return (float)prand(seed) / 0x7fffffff;
}

V3f v3f_safe_norm(V3f a)
{
    return v3f_norm(a, __FLT_EPSILON__, (V3f){0});
}

V3f v3f_scale(V3f a, float scale)
{
    V3f result;
    result.x = a.x * scale;
    result.y = a.y * scale;
    result.z = a.z * scale;
    return result;
}

static float schlick(float cos_theta, float idx)
{
    float r0 = (1.f - idx) / (1.f + idx);
    r0 = r0 * r0;
    return r0 + (1.f - r0) * powf(1.f - cos_theta, 5.f);
}

V3f v3f_refract(V3f uv, V3f n, float etai_over_etat)
{
    float cos_theta = fminf(-v3f_dot(uv, n), 1.0f);
    V3f r_out_perp = v3f_scale(v3f_add(uv, v3f_scale(n, cos_theta)), etai_over_etat);
    float k = 1.0f - v3f_dot(r_out_perp, r_out_perp);
    if (k < 0.0f)
        return (V3f){0, 0, 0};
    V3f r_out_parallel = v3f_scale(n, -sqrtf(k));
    return v3f_add(r_out_perp, r_out_parallel);
}

V3f random_in_unit_sphere(uint32_t *seed)
{
    while (1)
    {
        V3f p = {fprand(seed) * 2.f - 1.f, fprand(seed) * 2.f - 1.f, fprand(seed) * 2.f - 1.f};
        if (v3f_dot(p, p) < 1.f)
            return p;
    }
}

float sphere_intersect(V3f ro, V3f rd, V3f center, float radius)
{
    V3f oc = v3f_sub(ro, center);
    float b = v3f_dot(oc, rd);
    float c = v3f_dot(oc, oc) - radius * radius;
    float D = b * b - c;
    if (D < 0)
        return -1.f;
    return -b - sqrtf(D);
}

float plane_intersect(V3f ro, V3f rd, V3f p0, V3f n)
{
    float denom = v3f_dot(rd, n);
    if (fabsf(denom) < __FLT_EPSILON__)
        return -1.f;
    float t = v3f_dot(v3f_sub(p0, ro), n) / denom;
    return t > 0 ? t : -1.f;
}

float cube_intersect(V3f ro, V3f rd, V3f min, V3f max)
{
    float tmin = (min.x - ro.x) / rd.x;
    float tmax = (max.x - ro.x) / rd.x;
    if (tmin > tmax)
    {
        float tmp = tmin;
        tmin = tmax;
        tmax = tmp;
    }

    float tymin = (min.y - ro.y) / rd.y;
    float tymax = (max.y - ro.y) / rd.y;
    if (tymin > tymax)
    {
        float tmp = tymin;
        tymin = tymax;
        tymax = tmp;
    }

    if ((tmin > tymax) || (tymin > tmax))
        return -1.f;

    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;

    float tzmin = (min.z - ro.z) / rd.z;
    float tzmax = (max.z - ro.z) / rd.z;
    if (tzmin > tzmax)
    {
        float tmp = tzmin;
        tzmin = tzmax;
        tzmax = tmp;
    }

    if ((tmin > tzmax) || (tzmin > tmax))
        return -1.f;

    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;

    return tmin > 0.f ? tmin : tmax;
}

V3f sphere_uv(V3f t, V3f n, SDL_Surface *texture) 
{
    float u = atan2f(n.z, n.x) / (2.f * PI) + 0.5f;
    float v = 0.5f - asinf(n.y) / PI;
    uint32_t tex_x = (uint32_t)(u * texture->w) % texture->w;
    uint32_t tex_y = (uint32_t)(v * texture->h) % texture->h;
    
    SDL_PixelFormat *fmt = texture->format;
    uint32_t *pixels = (uint32_t *)texture->pixels;
    uint32_t pixel = pixels[tex_y * texture->w + tex_x];

    uint8_t r, g, b;
    SDL_GetRGB(pixel, fmt, &r, &g, &b);
    return (V3f){r / 255.f, g / 255.f, b / 255.f};
}

V3f plane_uv(V3f p, V3f n, SDL_Surface *texture)
{
    float u = p.x - floorf(p.x);
    float v = p.z - floorf(p.z);
    uint32_t tex_x = (uint32_t)(u * texture->w) % texture->w;
    uint32_t tex_y = (uint32_t)(v * texture->h) % texture->h;

    SDL_PixelFormat *fmt = texture->format;
    uint32_t *pixels = (uint32_t *)texture->pixels;
    uint32_t pixel = pixels[tex_y * texture->w + tex_x];

    uint8_t r, g, b;
    SDL_GetRGB(pixel, fmt, &r, &g, &b);
    return (V3f){r / 255.f, g / 255.f, b / 255.f};
}

V3f random_hemisphere(V3f n, uint32_t *seed)
{
    float u1 = fprand(seed);
    float u2 = fprand(seed);

    float r = sqrtf(u1);
    float theta = 2.f * PI * u2;

    float x = r * cosf(theta);
    float y = r * sinf(theta);
    float z = sqrtf(1.0f - u1);

    V3f up = fabsf(n.z) < 0.999f ? (V3f){0, 0, 1} : (V3f){1, 0, 0};
    V3f tangent = v3f_safe_norm(v3f_cross(up, n));
    V3f bitangent = v3f_cross(n, tangent);

    return v3f_safe_norm(
        v3f_add(
            v3f_add(v3f_scale(tangent, x),
                    v3f_scale(bitangent, y)),
            v3f_scale(n, z)));
}

V3f trace(V3f ro, V3f rd, size_t depth, uint32_t *seed)
{
    if (depth > MAX_DEPTH)
        return (V3f){0};

    float t_min = INFINITY;
    Object *hit_obj = NULL;

    for (size_t i = 0; i < SCENE_SIZE; ++i)
    {
        float t = -1.f;
        if (scene[i].type == OBJ_SPHERE)
            t = sphere_intersect(ro, rd, scene[i].position, scene[i].radius);
        else if (scene[i].type == OBJ_PLANE)
            t = plane_intersect(ro, rd, scene[i].position, scene[i].normal);
        else if (scene[i].type == OBJ_CUBE)
            t = cube_intersect(ro, rd,
                               scene[i].cube.max,
                               scene[i].cube.min);

        if (t > 0 && t < t_min)
        {
            t_min = t;
            hit_obj = &scene[i];
        }
    }

    if (!hit_obj)
        return (V3f){0.87f, 0.99f, 1.f}; // Space

    V3f p = v3f_add(ro, v3f_scale(rd, t_min));
    V3f n = (hit_obj->type == OBJ_SPHERE) ? v3f_safe_norm(v3f_sub(p, hit_obj->position)) : hit_obj->normal;
    Material material = hit_obj->material;

    V3f p_eps = v3f_add(p, v3f_scale(n, __FLT_EPSILON__ * 10.f));
    V3f color = {0};
    V3f light_position = {2, 5, -2};
    V3f light_color = {1, 1, 1};

    if (material.type == MAT_LAMBERTIAN)
    {
        V3f base_color = material.color;
        if (material.texture) 
        {
            if (hit_obj->type == OBJ_SPHERE) {
                base_color = sphere_uv(p, n, material.texture);
            } else if (hit_obj->type == OBJ_PLANE) {
                base_color = plane_uv(p, n, material.texture);
            }
        }

        V3f new_dir = random_hemisphere(n, seed);
        V3f indirect = v3f_mul(base_color, trace(p_eps, new_dir, depth + 1, seed));

        V3f to_light = v3f_safe_norm(v3f_sub(light_position, p));
        float dot_nl = fmaxf(0.f, v3f_dot(n, to_light));

        SDL_bool in_shadow = SDL_FALSE;
        for (size_t i = 0; i < SCENE_SIZE; i++)
        {
            float t_shadow = -1.f;
            if (scene[i].type == OBJ_SPHERE)
                t_shadow = sphere_intersect(p_eps, to_light, scene[i].position, scene[i].radius);
            else if (scene[i].type == OBJ_PLANE)
                t_shadow = plane_intersect(p_eps, to_light, scene[i].position, scene[i].normal);
            else if (scene[i].type == OBJ_CUBE)
                t_shadow = cube_intersect(p_eps, to_light, scene[i].cube.min, scene[i].cube.max);
            if (t_shadow > 0.f)
            {
                in_shadow = SDL_TRUE;
                break;
            }
        }

        V3f direct = in_shadow ? (V3f){0, 0, 0} : v3f_mul(base_color, v3f_scale(light_color, dot_nl));
        color = v3f_add(indirect, direct);
    }
    else if (material.type == MAT_SPECULAR)
    {
        V3f reflection = v3f_reflect(rd, n);
        V3f fuzz = v3f_scale(v3f_safe_norm(random_in_unit_sphere(seed)), material.roughness);
        V3f new_dir = v3f_safe_norm(v3f_add(reflection, fuzz));
        color = v3f_mul(material.color, trace(p_eps, new_dir, depth + 1, seed));
    }
    else if (material.type == MAT_DIELECTRIC)
    {
        float etai_over_etat;
        V3f n_out = n;
        float cos_theta = fminf(-v3f_dot(rd, n), 1.0f);
        float sin_theta = sqrtf(1.0f - cos_theta * cos_theta);

        if (v3f_dot(rd, n) > 0.0f)
        {
            n_out = v3f_scale(n, -1.0f);
            etai_over_etat = material.ior;
        }
        else
        {
            etai_over_etat = 1.0f / material.ior;
        }

        SDL_bool cannot_refract = etai_over_etat * sin_theta > 1.0f;
        float reflect_prob = schlick(cos_theta, material.ior);

        V3f direction;
        if (cannot_refract || fprand(seed) < reflect_prob)
            direction = v3f_reflect(rd, n_out);
        else
            direction = v3f_refract(rd, n_out, etai_over_etat);

        color = trace(v3f_add(p, v3f_scale(direction, __FLT_EPSILON__ * 10.f)), direction, depth + 1, seed);
    }

    return color;
}

void *worker(void *arg)
{
    Thread *data = (Thread *)arg;
    size_t total_pixels = WIDTH * HEIGHT;
    while (1)
    {
        size_t id = atomic_fetch_add(data->segment_id, 1);
        if (id >= data->num_segments)
            break;

        Segment segment = data->segments[id];
        uint32_t seed = (uint32_t)pthread_self() ^ (uint32_t)time(NULL) ^ id;

        for (size_t y = segment.y; y < segment.y + SEGMENT_SIZE && y < HEIGHT; y++)
            for (size_t x = segment.x; x < segment.x + SEGMENT_SIZE && x < WIDTH; x++)
            {

                V3f color = {0, 0, 0};
                for (size_t s = 0; s < SAMPLES; s++)
                {
                    float nx = 2.f * (x + fprand(&seed)) / WIDTH - 1.f;
                    float ny = 1.f - 2.f * (y + fprand(&seed)) / HEIGHT;
                    float scale = tanf(FOV / 2.f);

                    V3f forward = {
                        cosf(data->pitch) * sinf(data->yaw),
                        sinf(data->pitch),
                        cosf(data->pitch) * cosf(data->yaw)};
                    V3f right = v3f_safe_norm(v3f_cross((V3f){0, 1, 0}, forward));
                    V3f up = v3f_cross(forward, right);

                    V3f rd = v3f_add(
                        v3f_add(
                            v3f_scale(right, nx * ASPECT * scale),
                            v3f_scale(up, ny * scale)),
                        forward);
                    rd = v3f_safe_norm(rd);

                    color = v3f_add(color, v3f_scale(trace(data->camera, rd, 0, &seed), 1.f / SAMPLES));
                }

                float gamma = 2.2f;
                float inv_gamma = 1.f / gamma;
                uint8_t r = (uint8_t)fminf(255.f, powf(color.x, inv_gamma) * 255.f);
                uint8_t g = (uint8_t)fminf(255.f, powf(color.y, inv_gamma) * 255.f);
                uint8_t b = (uint8_t)fminf(255.f, powf(color.z, inv_gamma) * 255.f);

                data->buffer[y * WIDTH + x] = (r << 16) | (g << 8) | b;
            }
    }
    return NULL;
}

int main(void)
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        printf("SDL_Init Error: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow(
        "Prismis™",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WIDTH,
        HEIGHT,
        SDL_WINDOW_SHOWN
    );
    SDL_SetRelativeMouseMode(SDL_TRUE);
    if (!window)
    {
        SDL_Quit();
        printf("SDL_CreateWindow Error: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(
        window, 
        -1,
        SDL_RENDERER_SOFTWARE
    );
    if (!renderer)
    {
        SDL_DestroyWindow(window);
        SDL_Quit();
        printf("SDL_CreateRenderer Error: %s\n", SDL_GetError());
        return 1;
    }

    if (TTF_Init() != 0)
    {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        printf("TTF_Init Error: %s\n", TTF_GetError());
        return 1;
    }

    TTF_Font *font = TTF_OpenFont("assets/RobotoMono.ttf", 16);
    if (!font)
    {
        TTF_Quit();
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        printf("TTF_OpenFont Error: %s\n", TTF_GetError());
        return 1;
    }

    if (IMG_Init(IMG_INIT_PNG) == 0)
    {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        printf("IMG_Init Error: %s\n", IMG_GetError());
        return 1;
    }

    SDL_Surface *checkerboard = IMG_Load("assets/strange.png");
    if (!checkerboard)
    {
        TTF_CloseFont(font);
        TTF_Quit();
        IMG_Quit();
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        printf("IMG_Load Error: %s\n", IMG_GetError());
        return 1;
    }
    SDL_Surface *converted = SDL_ConvertSurfaceFormat(checkerboard, SDL_PIXELFORMAT_XRGB8888, 0);
    if (!converted)
    {
        TTF_CloseFont(font);
        TTF_Quit();
        SDL_FreeSurface(checkerboard);
        IMG_Quit();
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        printf("SDL_ConvertSurfaceFormat Error: %s\n", SDL_GetError());
        return 1;
    }
    SDL_FreeSurface(checkerboard);
    checkerboard = converted;

    SDL_Texture *texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_XRGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        WIDTH,
        HEIGHT); // Resizing for loosers
    if (!texture)
    {
        TTF_CloseFont(font);
        TTF_Quit();
        SDL_FreeSurface(checkerboard);
        IMG_Quit();
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        printf("SDL_CreateTexture Error: %s\n", SDL_GetError());
        return 1;
    }

    uint32_t *buffer = malloc(WIDTH * HEIGHT * sizeof(uint32_t));
    if (!buffer)
    {
        TTF_CloseFont(font);
        TTF_Quit();
        SDL_DestroyTexture(texture);
        SDL_FreeSurface(checkerboard);
        IMG_Quit();
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        printf("malloc failed!\n");
        return 1;
    }

    V3f camera = {0, 0, -5};
    float yaw = 0.f;
    float pitch = 0.f;
    size_t frame = 0;
    uint64_t last_time;
    uint64_t total_time = 0;
    SDL_Texture *fps_texture = NULL;
    SDL_Rect fps_rect;
    SDL_bool unk_fps = SDL_TRUE;

    // Assign texture to sphere material
    scene[0].material.texture = checkerboard;
    scene[3].material.texture = checkerboard;

    size_t num_segments = ((WIDTH + SEGMENT_SIZE - 1) / SEGMENT_SIZE) * ((HEIGHT + SEGMENT_SIZE - 1) / SEGMENT_SIZE);
    Segment *segments = malloc(sizeof(Segment) * num_segments);
    size_t i = 0;
    for (size_t y = 0; y < HEIGHT; y += SEGMENT_SIZE)
        for (size_t x = 0; x < WIDTH; x += SEGMENT_SIZE)
            segments[i++] = (Segment){x, y};
    pthread_t threads[THREADS];
    atomic_int segment_id = 0;
    atomic_bool render_complete = SDL_FALSE;

    SDL_Event event;
    SDL_bool running = SDL_TRUE;
    SDL_bool key_states[SDL_NUM_SCANCODES] = {SDL_FALSE};

    while (running)
    {
        last_time = SDL_GetTicks64();
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
                running = SDL_FALSE;
            else if (event.type == SDL_KEYDOWN)
                key_states[event.key.keysym.scancode] = SDL_TRUE;
            else if (event.type == SDL_KEYUP)
                key_states[event.key.keysym.scancode] = SDL_FALSE;
        }

        V3f forward = {sinf(yaw), 0, cosf(yaw)};
        V3f right = {cosf(yaw), 0, -sinf(yaw)};

        V2i mouse_movement;
        SDL_GetRelativeMouseState(&mouse_movement.x, &mouse_movement.y);
        yaw += mouse_movement.x * SENSITIVITY;
        pitch -= mouse_movement.y * SENSITIVITY;

        if (pitch > PI / 2.f - __FLT_EPSILON__)
            pitch = PI / 2.f - __FLT_EPSILON__;
        if (pitch < -PI / 2.f + __FLT_EPSILON__)
            pitch = -PI / 2.f + __FLT_EPSILON__;

        if (key_states[SDL_SCANCODE_W])
            camera = v3f_add(camera, v3f_scale(forward, SPEED));
        if (key_states[SDL_SCANCODE_S])
            camera = v3f_sub(camera, v3f_scale(forward, SPEED));
        if (key_states[SDL_SCANCODE_A])
            camera = v3f_sub(camera, v3f_scale(right, SPEED));
        if (key_states[SDL_SCANCODE_D])
            camera = v3f_add(camera, v3f_scale(right, SPEED));

        Thread data = {
            buffer,
            camera,
            segments,
            num_segments,
            &segment_id,
            yaw,
            pitch,
            right};
        atomic_store(&segment_id, 0);

        for (size_t i = 0; i < THREADS; i++)
            pthread_create(&threads[i], NULL, worker, &data);
        for (size_t i = 0; i < THREADS; i++)
            pthread_join(threads[i], NULL);

        total_time += SDL_GetTicks64() - last_time;
        if (++frame == 10 || unk_fps)
        {
            uint16_t fps = roundf((unk_fps ? 1.0f : 10.0f) * 1000.0f / total_time);
            if (unk_fps)
                unk_fps = SDL_FALSE;

            char fps_text[16];
            snprintf(fps_text, sizeof(fps_text), "FPS: %u", fps);

            SDL_Surface *fps_surface =
                TTF_RenderUTF8_Blended(font, fps_text,
                                    (SDL_Color){0,0,0,255});

            if (fps_texture)
                SDL_DestroyTexture(fps_texture);

            fps_texture = SDL_CreateTextureFromSurface(renderer, fps_surface);

            fps_rect.w = fps_surface->w;
            fps_rect.h = fps_surface->h;
            fps_rect.x = 10;
            fps_rect.y = 10;

            SDL_FreeSurface(fps_surface);

            frame = 0;
            total_time = 0;
        }

        SDL_UpdateTexture(texture, NULL, buffer, WIDTH * sizeof(uint32_t));
        SDL_RenderCopy(renderer, texture, NULL, NULL);

        if (fps_texture)
            SDL_RenderCopy(renderer, fps_texture, NULL, &fps_rect);

        SDL_RenderPresent(renderer);
    }

    free(buffer);
    free(segments);
    SDL_DestroyTexture(fps_texture);
    SDL_FreeSurface(checkerboard);
    IMG_Quit();
    TTF_CloseFont(font);
    TTF_Quit();
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
