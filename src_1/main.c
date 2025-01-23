#define SDL_MAIN_USE_CALLBACKS 1
#define STEP_SIZE 0.05
#define RES_X 640
#define RES_Y 400

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_render.h>
#include <stdio.h>

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;

// Physics space types
typedef struct position {
    double x_pos;
    double y_pos;
} pos_t;

typedef struct velocity {
    double x_vel;
    double y_vel;
} vel_t;

typedef struct particle {
    pos_t position;
    vel_t velocity;
    double x0_pos;
    double y0_pos;
    double x0_vel;
    double y0_vel;
} part_t;

// Coordinate system types
typedef struct point2d {
    double x;
    double y;
} point2d_t;

typedef struct viewport {
    double scale;      // World units to pixels
    point2d_t origin;  // World coordinate of screen center
    point2d_t size;    // Viewport size in world units
} viewport_t;

// Global state
static viewport_t viewport = {
    .scale = 100.0,
    .origin = {0.0, 0.0},
    .size = {RES_X / 100.0, RES_Y / 100.0}
};

static part_t particle;
static double time = 0.0;

// Physics functions
void init_particle(part_t *particle, double x_pos, double y_pos, double x_vel, double y_vel) {
    particle->position.x_pos = x_pos;
    particle->position.y_pos = y_pos;
    particle->velocity.x_vel = x_vel;
    particle->velocity.y_vel = y_vel;
    particle->x0_pos = x_pos;
    particle->y0_pos = y_pos;
    particle->x0_vel = x_vel;
    particle->y0_vel = y_vel;
}

void update_pos(part_t *particle) {
    particle->position.x_pos += STEP_SIZE * particle->velocity.x_vel;
    particle->position.y_pos += STEP_SIZE * particle->velocity.y_vel;
}

void update_vel(part_t *particle, double time) {
    particle->velocity.y_vel = -10 * time + particle->y0_vel;
}

void state_update(part_t *particle) {
    time += STEP_SIZE;
    update_vel(particle, time);
    update_pos(particle);
}

// Coordinate system functions
point2d_t world_to_screen(point2d_t world_pos) {
    point2d_t screen_pos = {
        .x = (world_pos.x - viewport.origin.x) * viewport.scale + RES_X / 2,
        .y = -(world_pos.y - viewport.origin.y) * viewport.scale + RES_Y / 2
    };
    return screen_pos;
}

point2d_t screen_to_world(point2d_t screen_pos) {
    point2d_t world_pos = {
        .x = (screen_pos.x - RES_X / 2) / viewport.scale + viewport.origin.x,
        .y = -(screen_pos.y - RES_Y / 2) / viewport.scale + viewport.origin.y
    };
    return world_pos;
}

// Check if a world position is within the viewport
int is_in_viewport(point2d_t world_pos) {
    double half_width = viewport.size.x / 2;
    double half_height = viewport.size.y / 2;
    return world_pos.x >= viewport.origin.x - half_width &&
           world_pos.x <= viewport.origin.x + half_width &&
           world_pos.y >= viewport.origin.y - half_height &&
           world_pos.y <= viewport.origin.y + half_height;
}

// SDL callback functions
SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]) {
    SDL_SetAppMetadata("Physics Simulation", "1.0", "com.example.physics-sim");

    init_particle(&particle, 0.0, 0.0, 2.0, 5.0);

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    if (!SDL_CreateWindowAndRenderer("Physics Simulation", RES_X, RES_Y, 0, &window, &renderer)) {
        SDL_Log("Couldn't create window/renderer: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
    if (event->type == SDL_EVENT_QUIT) {
        return SDL_APP_SUCCESS;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate) {
    // Clear screen
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    // Convert particle position to screen coordinates
    point2d_t world_pos = {particle.position.x_pos, particle.position.y_pos};
    point2d_t screen_pos = world_to_screen(world_pos);

    // Draw particle
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderPoint(renderer, (int)screen_pos.x, (int)screen_pos.y);

    // Present render
    SDL_RenderPresent(renderer);

    // Update physics
    state_update(&particle);

    // Reset if out of viewport
    if (!is_in_viewport(world_pos)) {
        init_particle(&particle, 0.0, 0.0, 2.0, 5.0);
        time = 0.0;
    }

    return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
