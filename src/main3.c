#define SDL_MAIN_USE_CALLBACKS 1
#include <time.h>

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_render.h>
#include <math.h>
#include <stdio.h>

// Screen and world dimensions
#define RES_X 800              // Screen width in pixels
#define RES_Y 800              // Screen height in pixels
#define BOX_LEFT -200.0        // Physics world left boundary
#define BOX_RIGHT 200.0        // Physics world right boundary
#define BOX_TOP 200.0          // Physics world top boundary
#define BOX_BOTTOM -200.0      // Physics world bottom boundary
#define TIME_STEP (1.0 / 60.0) // Physics time step

// Calculate scaling factors (pixels per physics unit)
#define SCALE_X (RES_X / (BOX_RIGHT - BOX_LEFT))
#define SCALE_Y (RES_Y / (BOX_TOP - BOX_BOTTOM))

#define FPS 60      // Target frames per second
#define MAX_SIZE 10 // Maximum number of particles
#define MASS_SCALE 4

// Global SDL variables
static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static Uint64 start_time = 0;

// 2D vector structure
typedef struct {
  double x;
  double y;
} vec2;

// Particle structure
typedef struct particle {
  vec2 position;
  vec2 prevPosition;
  vec2 acceleration;
  float mass;
  float radius;
} part_t;

// World structure (currently unused in update)
typedef struct {
  part_t *array[MAX_SIZE];
  int num_particles;
} world;

// Integrate using Verlet integration
void integrate(part_t *p, double dt) {
  vec2 velocity = {p->position.x - p->prevPosition.x,
                   p->position.y - p->prevPosition.y};

  vec2 temp = p->position;

  p->position.x += velocity.x + p->acceleration.x * dt * dt;
  p->position.y += velocity.y + p->acceleration.y * dt * dt;

  printf("Position: %f, %f\n", p->position.x, p->position.y);
  printf("Acc: %f, %f\n", p->acceleration.x, p->acceleration.y);
  printf("Acc shift: %f \n", p->acceleration.y * dt * dt);
  printf("Velocity: %f, %f\n", velocity.x, velocity.y);
  printf("Delta position: %lf, %lf\n", velocity.x + p->acceleration.x * dt * dt,
         velocity.y + p->acceleration.y * dt * dt);

  // Reset acceleration after each integration step
  p->acceleration.x = 0.0;
  p->acceleration.y = 0.0;

  p->prevPosition = temp;
}

// Initialize particle with starting position and velocity
void init_particle(part_t *p, double x0, double y0, double vx0, double vy0,
                   double mass) {
  p->position.x = x0;
  p->position.y = y0;

  // For Verlet integration, previous position is set as position - initial
  // velocity.
  p->prevPosition.x = x0 - vx0 * TIME_STEP;
  p->prevPosition.y = y0 - vy0 * TIME_STEP;

  p->acceleration.x = 0.0;
  p->acceleration.y = 0.0;
  p->mass = mass;
  p->radius = 3*mass;
}

// Check and resolve collisions with world boundaries
void check_world_collision(part_t *p) {
  double radius = p->radius;
  double v_x, v_y;

  // Left boundary
  if (p->position.x - radius < BOX_LEFT) {
    v_x = p->position.x - p->prevPosition.x;
    v_y = p->position.y - p->prevPosition.y;
    p->position.x = BOX_LEFT + radius;
    p->prevPosition.x =
        p->position.x + 0.8 * v_x;           // Reverse and dampen x velocity
    p->prevPosition.y = p->position.y - v_y; // Keep y velocity unchanged
  }
  // Right boundary
  if (p->position.x + radius > BOX_RIGHT) {
    v_x = p->position.x - p->prevPosition.x;
    v_y = p->position.y - p->prevPosition.y;
    p->position.x = BOX_RIGHT - radius;
    p->prevPosition.x =
        p->position.x + 0.8 * v_x;           // Reverse and dampen x velocity
    p->prevPosition.y = p->position.y - v_y; // Keep y velocity unchanged
  }
  // Top boundary
  if (p->position.y + radius > BOX_TOP) {
    v_x = p->position.x - p->prevPosition.x;
    v_y = p->position.y - p->prevPosition.y;
    p->position.y = BOX_TOP - radius;
    p->prevPosition.y =
        p->position.y + 0.8 * v_y;           // Reverse and dampen y velocity
    p->prevPosition.x = p->position.x - v_x; // Keep x velocity unchanged
  }
  // Bottom boundary
  if (p->position.y - radius < BOX_BOTTOM) {
    v_x = p->position.x - p->prevPosition.x;
    v_y = p->position.y - p->prevPosition.y;
    p->position.y = BOX_BOTTOM + radius;
    p->prevPosition.y =
        p->position.y + 0.8 * v_y;           // Reverse and dampen y velocity
    p->prevPosition.x = p->position.x - v_x; // Keep x velocity unchanged
  }
}

void check_particle_collision(part_t *p1, part_t *p2, double e) {
  vec2 pos1 = p1->position;
  vec2 v1 = {p1->position.x - p1->prevPosition.x,
             p1->position.y - p1->prevPosition.y};
  double r1 = p1->radius;
  double m1 = p1->mass;

  vec2 pos2 = p2->position;
  vec2 v2 = {p2->position.x - p2->prevPosition.x,
             p2->position.y - p2->prevPosition.y};
  double r2 = p2->radius;
  double m2 = p2->mass;

  // Squared distance between centers
  float center_dist = (pos1.x - pos2.x) * (pos1.x - pos2.x) +
                      (pos1.y - pos2.y) * (pos1.y - pos2.y);

  if (center_dist < pow(r1 + r2, 2)) {
    // Normalize the collision normal vector
    double dist = sqrt(center_dist);
    double nx = (pos1.x - pos2.x) / dist;
    double ny = (pos1.y - pos2.y) / dist;

    // Tangent vector (perpendicular to normal)
    double tx = -ny;
    double ty = nx;

    // Project velocities onto normal and tangent vectors
    double p1n = nx * v1.x + ny * v1.y; // v1 projected onto n
    double p1t = tx * v1.x + ty * v1.y; // v1 projected onto t

    double p2n = nx * v2.x + ny * v2.y; // v2 projected onto n
    double p2t = tx * v2.x + ty * v2.y; // v2 projected onto t

    // Compute new normal velocities after collision
    double v1n = (p1n * (m1 - e * m2) + p2n * m2 * (1 + e)) / (m1 + m2);
    double v2n = (p2n * (m2 - e * m1) + p1n * m1 * (1 + e)) / (m1 + m2);

    // Update velocities
    v1.x = v1n * nx + p1t * tx;
    v1.y = v1n * ny + p1t * ty;

    v2.x = v2n * nx + p2t * tx;
    v2.y = v2n * ny + p2t * ty;

    // Adjust positions to prevent overlap
    double overlap = (r1 + r2) - dist;
    p1->position.x += overlap * nx / 2;
    p1->position.y += overlap * ny / 2;
    p2->position.x -= overlap * nx / 2;
    p2->position.y -= overlap * ny / 2;

    p1->prevPosition.x = p1->position.x - v1.x;
    p1->prevPosition.y = p1->position.y - v1.y;
    p2->prevPosition.x = p2->position.x - v2.x;
    p2->prevPosition.y = p2->position.y - v2.y;
  }
}
// (Optional) Function to add particles to a world structure
void new_particles(world *w, int index, part_t *p) {
  if (index < MAX_SIZE) {
    w->array[index] = p;
    w->num_particles++;
  }
}

// Structure for render coordinates
typedef struct render_pos {
  double x, y;
  double r;
} rpos_t;

// Convert physics coordinates to screen coordinates
rpos_t coordinate_converter(part_t *particle) {
  rpos_t world_c;
  world_c.x = SCALE_X * particle->position.x + RES_X / 2.0;
  world_c.y = -SCALE_Y * particle->position.y + RES_Y / 2.0;
  // Use the minimum scale factor for radius to maintain aspect ratio
  world_c.r = fmin(SCALE_X, SCALE_Y) * particle->radius;
  return world_c;
}

// Draw a filled circle using the midpoint circle algorithm
void midPointCircleFill(SDL_Renderer *renderer, int x_centre, int y_centre,
                        int r) {
  int x = r, y = 0;

  if (r > 0) {
    // Draw horizontal lines for the initial points
    SDL_RenderLine(renderer, x + x_centre, y + y_centre, -x + x_centre,
                   y + y_centre);
    SDL_RenderLine(renderer, y + x_centre, x + y_centre, -y + x_centre,
                   x + y_centre);
  }

  int P = 1 - r;
  while (x > y) {
    y++;
    if (P <= 0) {
      P += 2 * y + 1;
    } else {
      x--;
      P += 2 * y - 2 * x + 1;
    }
    if (x < y)
      break;

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

// Global particle variables
part_t part1;
part_t part2;
part_t part3;
part_t part4;
part_t part5, part6, part7, part8, part9, part10;
part_t *particles[MAX_SIZE];
double dt = TIME_STEP;

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]) {
  SDL_SetAppMetadata("Physics Simulation", "1.0", "com.example.physics-sim");

  // Initialize a single particle for now
  particles[0] = &part1;
  particles[1] = &part2;
  particles[2] = &part3;
  particles[3] = &part4;
  particles[4] = &part5;
  particles[5] = &part6;
  particles[6] = &part7;
  particles[7] = &part8;
  particles[8] = &part9;
  particles[9] = &part10;
  init_particle(&part1, 100, 75, 20, -10.0, 5);
  init_particle(&part1, 100, 75, 20, -10.0, 5);    // Particle 1
  init_particle(&part2, 50, 10, 10, -10.0, 2);     // Particle 2
  init_particle(&part3, -50, -100, 0, 30.0, 1);    // Particle 3
  init_particle(&part4, 120, -200, 5, 15.0, 3);    // Particle 4
  init_particle(&part5, -150, 50, -15, -25.0, 4);  // Particle 5
  init_particle(&part6, 200, -50, 30, 40.0, 6);    // Particle 6
  init_particle(&part7, -200, -200, -20, 10.0, 2); // Particle 7
  init_particle(&part8, 30, 100, 0, -20.0, 3);     // Particle 8
  init_particle(&part9, -100, 150, 5, -5.0, 1);    // Particle 9
  init_particle(
      &part10, 50, 50, 15, 0.0,
      4); // Particle 10   init_particle(&part2, 50, 10, 10, -10.0, 2);

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  if (!SDL_CreateWindowAndRenderer("Physics Simulation", RES_X, RES_Y, 0,
                                   &window, &renderer)) {
    SDL_Log("Couldn't create window/renderer: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  // Initialize the start time for frame timing.
  start_time = SDL_GetTicks();

  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
  if (event->type == SDL_EVENT_QUIT)
    return SDL_APP_SUCCESS;
  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate) {
  Uint64 current = SDL_GetTicks();
  Uint64 delta = current - start_time;

  // Run update/render at the target FPS.
  if (delta > 1000 * TIME_STEP) {
    // Clear the screen
    SDL_SetRenderDrawColor(renderer, 108, 122, 137, 0);
    SDL_RenderClear(renderer);

    // Render all particles
    for (int i = 0; i < MAX_SIZE; i++) {
      if (particles[i]) {
        rpos_t world_c = coordinate_converter(particles[i]);
        SDL_SetRenderDrawColor(renderer, 236, 236, 236, 0);
        midPointCircleFill(renderer, (int)world_c.x, (int)world_c.y,
                           (int)world_c.r);
      }
    }
    SDL_RenderPresent(renderer);

    // Use substeps for physics updates
    int substeps = 10;
    double dt_sub = dt / substeps;

    for (int step = 0; step < substeps; step++) {
      // Apply gravity and integrate motion
      for (int i = 0; i < MAX_SIZE; i++) {
        if (particles[i]) {
          particles[i]->acceleration.y = -20;
          integrate(particles[i], dt_sub);
          check_world_collision(particles[i]);
        }
      }

      // Check for particle collisions
      for (int i = 0; i < MAX_SIZE; i++) {
        if (particles[i]) {
          for (int j = i + 1; j < MAX_SIZE; j++) {
            if (particles[j]) {
              check_particle_collision(
                  particles[i], particles[j],
                  0.8); // e = 0.8 (coefficient of restitution)
            }
          }
        }
      }
    }

    // Reset frame timer
    start_time = current;
  }

  return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) {
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}
