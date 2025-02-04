#define SDL_MAIN_USE_CALLBACKS 1
#define RES_X 800 // Screen width in pixels
#define RES_Y 800 // Screen height in pixels

#define BOX_LEFT -200.0   // Physics world left boundary
#define BOX_RIGHT 200.0   // Physics world right boundary
#define BOX_TOP 200.0     // Physics world top boundary
#define BOX_BOTTOM -200.0 // Physics world bottom boundary
#define TIME_STEP 0.001

// Calculate scaling factors
#define SCALE_X                                                                \
  (RES_X / (BOX_RIGHT - BOX_LEFT)) // Scale X: pixels per physics unit
#define SCALE_Y                                                                \
  (RES_Y / (BOX_TOP - BOX_BOTTOM)) // Scale Y: pixels per physics unit

#define FPS 60      // Target frames per second
#define MAX_SIZE 10 // Maximum number of particles
#define MASS_SCALE 4
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_render.h>
#include <math.h>
#include <stdio.h>

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static Uint64 start_time = 0;

// world box
struct WorldBox {
  float minX, maxX; // Horizontal boundaries
  float minY, maxY; // Vertical boundaries
};

// particle dynamics
typedef struct {
  double x;
  double y;
} vec2d_t;

typedef struct particle {
  vec2d_t position;
  vec2d_t velocity;
  vec2d_t acceleration;
  int id;
  bool on_ground;
  float mass;
  float radius;
} part_t;

typedef struct particles {
  part_t *particles[MAX_SIZE];
  int num_particles;
} parts_t;

void state_update(part_t *particle, double dt);

void update_kinematics(part_t *particle, double dt) {
  if (particle->on_ground) {
    particle->acceleration.x = 0;
    particle->acceleration.y = 0; // Cancel gravity
  } else {
    particle->acceleration.x = 0;
    particle->acceleration.y = -10; // Apply gravity
  }
  particle->velocity.x += dt * particle->acceleration.x;
  particle->velocity.y += dt * particle->acceleration.y;

  particle->position.x += dt * particle->velocity.x;
  particle->position.y += dt * particle->velocity.y;

  printf("Particle with id %d, has position %f, %f, \n", particle->id,
         particle->position.x, particle->position.y);
  printf("Particle with id %d, has velocity %f, %f, \n", particle->id,
         particle->velocity.x, particle->velocity.y);
}
void init_particle(parts_t *particles, part_t *particle, double x0, double y0,
                   double vx0, double vy0, double mass, int id) {
  particle->position.x = x0;
  particle->position.y = y0;
  particle->velocity.x = vx0;
  particle->velocity.y = vy0;
  particle->mass = mass;
  particle->radius = mass * MASS_SCALE;
  particle->id = id;

  particles->particles[particles->num_particles] = particle;
  particles->num_particles += 1;
}
void check_box_collision(part_t *particle) {
  particle->on_ground = false;
  // Current particle position and radius
  double x = particle->position.x;
  double y = particle->position.y;
  double radius = particle->radius;

  // Check left boundary
  if (x - radius < BOX_LEFT) {
    particle->position.x = BOX_LEFT + radius; // Clamp position
    particle->velocity.x *= -0.8;             // Reverse X velocity
  }

  // Check right boundary
  if (x + radius > BOX_RIGHT) {
    particle->position.x = BOX_RIGHT - radius; // Clamp position
    particle->velocity.x *= -0.8;              // Reverse X velocity
  }

  // Check top boundary
  if (y + radius > BOX_TOP) {
    particle->position.y = BOX_TOP - radius; // Clamp position
    particle->velocity.y *= -0.8;            // Reverse Y velocity
  }

  // Check bottom boundary
  if (y - radius < BOX_BOTTOM) {
    particle->position.y = BOX_BOTTOM + radius; // Clamp position
    particle->velocity.y *= -0.8;               // Reverse Y velocity
    particle->on_ground = true; // Mark as grounded
  }
}

double inner_product(vec2d_t v1, vec2d_t v2) {
  return v1.x * v2.x + v1.y * v2.y;
}
void check_particle_collision(part_t *part1, part_t *part2, double e) {
  vec2d_t pos1 = part1->position;
  vec2d_t vel1 = part1->velocity;
  double r1 = part1->radius;
  double m1 = part1->mass;

  vec2d_t pos2 = part2->position;
  vec2d_t vel2 = part2->velocity;
  double r2 = part2->radius;
  double m2 = part2->mass;

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
    double p1n = nx * vel1.x + ny * vel1.y; // vel1 projected onto n
    double p1t = tx * vel1.x + ty * vel1.y; // vel1 projected onto t

    double p2n = nx * vel2.x + ny * vel2.y; // vel2 projected onto n
    double p2t = tx * vel2.x + ty * vel2.y; // vel2 projected onto t

    // Compute new normal velocities after collision
    double v1n = (p1n * (m1 - e * m2) + p2n * m2 * (1 + e)) / (m1 + m2);
    double v2n = (p2n * (m2 - e * m1) + p1n * m1 * (1 + e)) / (m1 + m2);

    // Update velocities
    part1->velocity.x = v1n * nx + p1t * tx;
    part1->velocity.y = v1n * ny + p1t * ty;

    part2->velocity.x = v2n * nx + p2t * tx;
    part2->velocity.y = v2n * ny + p2t * ty;

    // Adjust positions to prevent overlap
    double overlap = (r1 + r2) - dist;
    part1->position.x += overlap * nx / 2;
    part1->position.y += overlap * ny / 2;
    part2->position.x -= overlap * nx / 2;
    part2->position.y -= overlap * ny / 2;
  }
}
void update_all_particles(parts_t particles, double dt) {
  int num_particles = particles.num_particles;
  for (int i = 0; i < num_particles; i++) {
    state_update(particles.particles[i], dt);
  }
  for (int i = 0; i < num_particles; i++) {
    for (int j = i + 1; j < num_particles; j++) {
      check_particle_collision(particles.particles[i], particles.particles[j],
                               0.8);
    }
  }
}

double system_energy(parts_t particles) {
  int num_particles = particles.num_particles;
  double energy = 0;
  for (int i = 0; i < num_particles; i++) {
    part_t *particle = particles.particles[i];
    energy += 0.5 * particle->mass *
              (pow(particle->velocity.x, 2) + pow(particle->velocity.y, 2));
  }
  return energy;
}
void state_update(part_t *particle, double dt) {
  update_kinematics(particle, dt);
  check_box_collision(particle);
}

// render code

typedef struct render_pos {
  double x, y;
  double r;
} rpos_t;

rpos_t coordinate_converter(part_t *particle) {
  rpos_t world_c;
  world_c.x = (float)(SCALE_X * particle->position.x) + RES_X / 2.0f;
  world_c.y = (float)(-SCALE_Y * particle->position.y) + RES_Y / 2.0f;
  // I think it should be the min of the scales
  world_c.r = fmin(SCALE_X, SCALE_Y) * particle->radius;
  return world_c;
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

parts_t particles;
part_t part1;
part_t part2;
part_t part3;
part_t part4;
part_t part5;
double dt = 0.001;

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]) {
  SDL_SetAppMetadata("Physics Simulation", "1.0", "com.example.physics-sim");
  particles.num_particles = 0;

  init_particle(&particles, &part1, 100, 75, 20, 10.0, 5, 1);
  init_particle(&particles, &part2, 20, 75, 5, 30, 3, 2);
  init_particle(&particles, &part3, 30, 20, 15, -20, 4, 3);
  init_particle(&particles, &part4, 40, 40, 50, 15, 2, 4);
  init_particle(&particles, &part5, 50, 10, 25, -25, 3, 4);
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
  for(int i=0; i<1;i++){
    update_all_particles(particles, dt);
  }
  if (delta > 1000 / FPS) {
    SDL_SetRenderDrawColor(renderer, 108, 122, 137, 0);
    SDL_RenderClear(renderer);
    // Convert particle position to screen coordinates
    // Present render
    for (int i = 0; i < particles.num_particles; i++) {
      rpos_t world_c = coordinate_converter(particles.particles[i]);
      SDL_SetRenderDrawColor(renderer, 236, 236, 236, 0);
      midPointCircleFill(renderer, (int)world_c.x, (int)world_c.y,
                         (int)world_c.r);
    }
    SDL_RenderPresent(renderer);

    // Update physics
    update_all_particles(particles, dt);
    double energy = system_energy(particles);
    printf("Total energy: %f\n", energy);

    start_time = current;
    double fp = 1000.f / delta;
  }
  return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) {
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}
