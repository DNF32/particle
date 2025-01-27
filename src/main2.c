#define SDL_MAIN_USE_CALLBACKS 1
#define RES_X 800
#define RES_Y 800
#define SCALE_X 8.0f
#define SCALE_Y 8.0f
#define FPS 144
#define MAX_SIZE 10

#define BOX_LEFT -50.0
#define BOX_RIGHT 50.0
#define BOX_TOP 50.0
#define BOX_BOTTOM -50.0

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_render.h>
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
typedef struct position {
  double x_pos;
  double y_pos;
} pos_t;

typedef struct velocity {
  double x_vel;
  double y_vel;
} vel_t;

typedef struct acceleration {
  double x_acc;
  double y_acc;
} acc_t;

typedef struct particle {
  pos_t position;
  vel_t velocity;
  acc_t acceleration;
  float radius;
} part_t;

typedef struct particles {
  part_t *particles[MAX_SIZE];
  int num_particles;
} parts_t;

void state_update(part_t *particle);

void update_position(part_t *particle) {
  particle->position.x_pos += (1.0f / FPS) * particle->velocity.x_vel;
  particle->position.y_pos += (1.0f / FPS) * particle->velocity.y_vel;
}
void update_velocity(part_t *particle) {
  particle->velocity.x_vel += (1.0f / FPS) * particle->acceleration.x_acc;
  particle->velocity.y_vel += (1.0f / FPS) * particle->acceleration.y_acc;
}

// here is the main driving force (hehehe) of the dynamics, we could have some
// particle whose accelariton depends of the time.
void update_acc(part_t *particle) {
  particle->acceleration.x_acc = 0;
  particle->acceleration.y_acc = -10 * particle->radius;
}

void init_particle(parts_t *particles, part_t *particle, double x_pos,
                   double y_pos, double x_vel, double y_vel, double radius) {
  particle->position.x_pos = x_pos;
  particle->position.y_pos = y_pos;
  particle->velocity.x_vel = x_vel;
  particle->velocity.y_vel = y_vel;
  particle->radius = radius;

  particles->particles[particles->num_particles] = particle;
  particles->num_particles += 1;
}
void check_box_collision(part_t *particle) {
  // Current particle position and radius
  double x = particle->position.x_pos;
  double y = particle->position.y_pos;
  double radius = particle->radius;

  // Check left boundary
  if (x - radius < BOX_LEFT) {
    particle->position.x_pos = BOX_LEFT + radius; // Clamp position
    particle->velocity.x_vel *= -0.8;             // Reverse X velocity
  }

  // Check right boundary
  if (x + radius > BOX_RIGHT) {
    particle->position.x_pos = BOX_RIGHT - radius; // Clamp position
    particle->velocity.x_vel *= -0.8;              // Reverse X velocity
  }

  // Check top boundary
  if (y + radius > BOX_TOP) {
    particle->position.y_pos = BOX_TOP - radius; // Clamp position
    particle->velocity.y_vel *= -0.8;            // Reverse Y velocity
  }

  // Check bottom boundary
  if (y - radius < BOX_BOTTOM) {
    particle->position.y_pos = BOX_BOTTOM + radius; // Clamp position
    particle->velocity.y_vel *= -0.8;               // Reverse Y velocity
  }
}

void check_particle_collision(part_t *part1, part_t *part2) {
  float center_dist = (part1->position.x_pos - part2->position.x_pos) *
                          (part1->position.x_pos - part2->position.x_pos) +
                      (part1->position.y_pos - part2->position.y_pos) *
                          (part1->position.y_pos - part2->position.y_pos);

  if (center_dist <
      (part1->radius + part2->radius) * (part1->radius + part2->radius)) {

    pos_t pos1 = part1->position;
    vel_t vel1 = part1->velocity;
    float m1 = part1->radius;

    pos_t pos2 = part2->position;
    vel_t vel2 = part2->velocity;
    float m2 = part2->radius;

    double dif_velx = vel1.x_vel - vel2.x_vel;
    double dif_vely = vel1.x_vel - vel2.x_vel;

    double dif_posx = pos1.x_pos - pos2.x_pos;
    double dif_posy = pos1.x_pos - pos2.x_pos;

    double inner_prod = dif_velx * dif_posx + dif_vely * dif_posy;  

    part1->velocity.x_vel =
        vel1.x_vel -
        (2 * m2 / (m1 + m2)) *
            inner_prod *
            (pos1.x_pos - pos2.x_pos) / (center_dist);

    part1->velocity.y_vel =
        vel1.y_vel -
        (2 * m2 / (m1 + m2)) *
            inner_prod *
            (pos1.y_pos - pos2.y_pos) / (center_dist);

    part2->velocity.x_vel =
        vel2.x_vel -
        (2 * m1 / (m1 + m2)) *
            inner_prod *
            (pos2.x_pos - pos1.x_pos) / (center_dist);

    part2->velocity.y_vel =
        vel2.y_vel -
        (2 * m1 / (m1 + m2)) *
            inner_prod *
            (pos2.y_pos - pos1.y_pos) / (center_dist);
  }
}

void update_all_particles(parts_t particles) {
  int num_particles = particles.num_particles;
  for (int i = 0; i < num_particles; i++) {
    state_update(particles.particles[i]);
  }
  for (int i = 0; i < num_particles; i++) {
    for (int j = i + 1; j < num_particles; j++) {
      check_particle_collision(particles.particles[i], particles.particles[j]);
    }
  }
}

void state_update(part_t *particle) {
  update_acc(particle);
  update_velocity(particle);
  update_position(particle);
  check_box_collision(particle);
}

// render code

typedef struct render_pos {
  double x, y;
  double r;
} rpos_t;

rpos_t coordinate_converter(part_t *particle) {
  rpos_t world_c;
  world_c.x = (float)(SCALE_X * particle->position.x_pos) + RES_X / 2.0f;
  world_c.y = (float)(-SCALE_Y * particle->position.y_pos) + RES_Y / 2.0f;
  // I think it should be the min of the scales
  world_c.r = SCALE_X * particle->radius;
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

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]) {
  SDL_SetAppMetadata("Physics Simulation", "1.0", "com.example.physics-sim");
  particles.num_particles = 0;

  init_particle(&particles, &part1, 100, 75, 20, 10.0, 5);
  init_particle(&particles, &part2, 10, 75, 20, 2, 1);

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
    update_all_particles(particles);
    start_time = current;
    double fp = 1000.f / delta;
    printf("the current fps %f", fp);
  }
  return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) {
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}
