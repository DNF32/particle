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
static Uint64 start_time = 0;

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
  int radius;
} part_t;

// Coordinate system types
typedef struct point2d {
  double x;
  double y;
} point2d_t;

typedef struct viewport {
  double scale;     // World units to pixels
  point2d_t origin; // World coordinate of screen center
  point2d_t size;   // Viewport size in world units
} viewport_t;

// headers
void collision(part_t *particle);
point2d_t world_to_screen(point2d_t world_pos);

// Global state
static viewport_t viewport = {.scale = 100.0,
                              .origin = {0.0, 0.0},
                              .size = {RES_X / 100.0, RES_Y / 100.0}};

static part_t particle;
static double time_step = 0.0;

// Physics functions
void init_particle(part_t *particle, double x_pos, double y_pos, double x_vel,
                   double y_vel) {
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

void update_vel(part_t *particle, double time_step) {
  particle->velocity.y_vel = -10 * time_step + particle->y0_vel;
}

void state_update(part_t *particle) {
  time_step += STEP_SIZE;
  part_t current = *particle;

  //get next position
  update_vel(&current, time_step);
  update_pos(&current);

  point2d_t world_pos = {particle->position.x_pos, particle->position.y_pos};
  point2d_t screen_pos = world_to_screen(world_pos);

  // bottom colision or top
  if (screen_pos.y + particle->radius >= RES_Y ||
      screen_pos.y + particle->radius <= 0) {
    time_step = 0;
    particle->y0_vel = -particle->y0_vel;
    float t = 
  }
  // right colison or left
  if (screen_pos.x + particle->radius >= RES_X ||
      screen_pos.x + particle->radius <= 0) {
    time_step = 0;
    particle->x0_vel = -particle->x0_vel;
    update_pos(particle);
  }
}

void collision(part_t *particle) {
  point2d_t world_pos = {particle->position.x_pos, particle->position.y_pos};
  point2d_t screen_pos = world_to_screen(world_pos);

  // bottom colision or top
  if (screen_pos.y + particle->radius >= RES_Y ||
      screen_pos.y + particle->radius <= 0) {
    time_step = 0;
    particle->y0_vel = -particle->y0_vel;
    update_pos(particle);
    return true;
  }
  // right colison or left
  if (screen_pos.x + particle->radius >= RES_X ||
      screen_pos.x + particle->radius <= 0) {
    time_step = 0;
    particle->x0_vel = -particle->x0_vel;
    update_pos(particle);
    return true;
  }
  return false;
}

// Coordinate system functions
point2d_t world_to_screen(point2d_t world_pos) {
  point2d_t screen_pos = {
      .x = (world_pos.x - viewport.origin.x) * viewport.scale + RES_X / 2,
      .y = -(world_pos.y - viewport.origin.y) * viewport.scale + RES_Y / 2};
  return screen_pos;
}

point2d_t screen_to_world(point2d_t screen_pos) {
  point2d_t world_pos = {
      .x = (screen_pos.x - RES_X / 2) / viewport.scale + viewport.origin.x,
      .y = -(screen_pos.y - RES_Y / 2) / viewport.scale + viewport.origin.y};
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

void midPointCircleFill(SDL_Renderer *renderer, int x_centre, int y_centre,
                        int r) {
  int x = r, y = 0;

  // When radius is zero, only a single point will be printed
  if (r > 0) {
    // Draw horizontal lines for the initial points
    SDL_RenderLine(renderer, x + x_centre, y + y_centre, -x + x_centre,
                   y + y_centre);
    SDL_RenderLine(renderer, y + x_centre, x + y_centre, -y + x_centre,
                   x + y_centre);
  }

  // Initialising the value of P
  int P = 1 - r;
  while (x > y) {
    y++;

    // Mid-point is inside or on the perimeter
    if (P <= 0)
      P = P + 2 * y + 1;

    // Mid-point is outside the perimeter
    else {
      x--;
      P = P + 2 * y - 2 * x + 1;
    }

    // All the perimeter points have already been printed
    if (x < y)
      break;

    //  horizontal lines to fill the circle
    SDL_RenderLine(renderer, x + x_centre, y + y_centre, -x + x_centre,
                   y + y_centre);
    SDL_RenderLine(renderer, x + x_centre, -y + y_centre, -x + x_centre,
                   -y + y_centre);
    SDL_RenderLine(renderer, y + x_centre, x + y_centre, -y + x_centre,
                   x + y_centre);
    SDL_RenderLine(renderer, y + x_centre, -x + y_centre, -y + x_centre,
                   -x + y_centre);
  }
}
// SDL callback functions
SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]) {
  SDL_SetAppMetadata("Physics Simulation", "1.0", "com.example.physics-sim");

  init_particle(&particle, 0.0, 0.0, 2.0, 5.0);

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  if (!SDL_CreateWindowAndRenderer("Physics Simulation", RES_X, RES_Y, 0,
                                   &window, &renderer)) {
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

  Uint64 current = SDL_GetTicks();
  Uint64 delta = current - start_time;
  if (delta > 1000 / 45) {
    SDL_SetRenderDrawColor(renderer, 108, 122, 137, 0);
    SDL_RenderClear(renderer);
    // Convert particle position to screen coordinates
    point2d_t world_pos = {particle.position.x_pos, particle.position.y_pos};
    point2d_t screen_pos = world_to_screen(world_pos);

    // Present render
    SDL_SetRenderDrawColor(renderer, 236, 236, 236, 0);
    midPointCircleFill(renderer, (int)screen_pos.x, (int)screen_pos.y, 10);
    SDL_RenderPresent(renderer);

    // Update physics
    state_update(&particle);

    //  if (!is_in_viewport(world_pos)) {
    //    init_particle(&particle, 0.0, 0.0, 2.0, 5.0);
    //    time_step = 0.0;
    //  }
    collision(&particle);

    start_time = current;
  }
  printf("FPS %f", 1.0f / delta);
  return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) {
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}
