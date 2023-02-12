/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: October 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/*
In this exercise you will generate a circle-based collision detection function.
To do so, you'll have to:
1) calculate the center coordinates (x,y) of the 3 circles that will represent
every car. To do that, you will use: a) CIRCLE_OFFSETS = distance that the
circles should be at wrt to the center of the vehicle and on the centerline of
the vehicle. b) CIRCLE_RADII = the radioses of the circles. c) The vehicles'
heading/yaw.
circle_center.x =
         vehicle.location.x +
         CIRCLE_OFFSETS[c] * std::cos(vehicle.rotation.yaw);
circle_center.y =
         vehicle.location.y +
         CIRCLE_OFFSETS[c] * std::sin(vehicle.rotation.yaw);

2) Once the circles are placed (i.e we know their centers'
coordinates) for both, the ego car and the "actor", we can proceed to check for
collisions. Here you will have to, pairwise (taking 1 circle from ego and 1
circle form the actor) check if they intersect (collision) or not.
dist = std::sqrt(
           std::pow((ego_circle_center.x - actor_circle_center.x), 2) +
           std::pow((ego_circle_center.y - actor_circle_center.y), 2));
collision = (dist < (CIRCLE_RADII[c1] + CIRCLE_RADII[c2]));
*/

#include <math.h>

#include <array>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>

constexpr std::array<float, 3> CIRCLE_OFFSETS = {-1.0, 1.0, 2.0};  // m
constexpr std::array<float, 3> CIRCLE_RADII = {1.5, 1.5, 1.5};     // m

// In Meters
struct Location {
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
};

// In Radians
struct Rotation {
  float pitch = 0.0;
  float yaw = 0.0;
  float roll = 0.0;
};

struct State {
  Location location;
  Rotation rotation;
};

std::vector<bool> collision_checker(const State& ego_vehicle,
                                    const std::vector<State>& obstacles) {
  std::vector<bool> collisions(obstacles.size(), false);
  auto n_circles = CIRCLE_OFFSETS.size();

  for (size_t obs = 0; obs < obstacles.size(); ++obs) {
    auto actor = obstacles[obs];
    bool collision{false};

    for (size_t c = 0; c < n_circles && !collision; ++c) {
      // TODO-Circle placement: Where should the circles be at? The code below
      // is NOT complete. HINT: use CIRCLE_OFFSETS[c], sine and cosine to
      // calculate x and y.
      // circle_center.x =
      //           vehicle.location.x +
      //           CIRCLE_OFFSETS[c] * std::cos(vehicle.rotation.yaw);
      // circle_center.y =
      //           vehicle.location.y +
      //           CIRCLE_OFFSETS[c] * std::sin(vehicle.rotation.yaw);
      Location ego_circle_center;
      ego_circle_center.x = ego_vehicle.location.x + CIRCLE_OFFSETS[c] * cos(ego_vehicle.rotation.yaw);
      ego_circle_center.y = ego_vehicle.location.y + CIRCLE_OFFSETS[c] * sin(ego_vehicle.rotation.yaw);

      for (size_t c2 = 0; c2 < n_circles && !collision; ++c2) {
        Location actor_circle_center;

        actor_circle_center.x =
            actor.location.x +
            CIRCLE_OFFSETS[c2] * std::cos(actor.rotation.yaw);

        actor_circle_center.y =
            actor.location.y +
            CIRCLE_OFFSETS[c2] * std::sin(actor.rotation.yaw);

        // TODO-Distance from circles to obstacles/actor: How do you calculate
        // the distance between the center of both circles? HINT: distance
        // between 2 points. Use std::sqrt(...)
        // dist = std::sqrt(
        //     std::pow((ego_circle_center.x - actor_circle_center.x), 2) +
        //     std::pow((ego_circle_center.y - actor_circle_center.y), 2));
        double dist = sqrt( pow(ego_circle_center.x - actor_circle_center.x, 2) + pow(ego_circle_center.y - actor_circle_center.y, 2) );

        // TODO-What do we consider a collision? When the distance between the 2
        // centers is smaller that the sum of their radii.
        // collision = (dist < (CIRCLE_RADII[c] + CIRCLE_RADII[c2]));
        collision = dist <= ( CIRCLE_RADII[c] + CIRCLE_RADII[c2] );

        std::cout << "c " << c << ", c2 " << c2                 
                  << " c_x: " << ego_circle_center.x
                  << " c_y: " << ego_circle_center.y
                  << " a_x: " << actor_circle_center.x
                  << " a_y: " << actor_circle_center.y
                  << " distance: " << dist
                  << ", RadiiSum: " << (CIRCLE_RADII[c] + CIRCLE_RADII[c2])
                  << ", collision: " << collision << std::endl;
      }
    }

    collisions[obs] = collision;
    // std::cout << "obs " << obs << ": " << collision << std::endl;
  }
  return collisions;
}

// ******* MAIN FUNCTION ********
int main(int argc, const char* argv[]) {
  State ego_vehicle;
  std::cout << "Test Case 1: " << std::endl;
  ego_vehicle.location.x = 10.0;
  ego_vehicle.location.y = 5.5;
  ego_vehicle.rotation.yaw = 0.349066;

  std::vector<State> obstacles;
  State obstacle;

  obstacle.location.x = 10.0;
  obstacle.location.y = 5.5;
  obstacle.rotation.yaw = 0.34;
  obstacles.push_back(obstacle);

  obstacle.location.x = 3;
  obstacle.location.y = -2;
  obstacle.rotation.yaw = 0.44;
  obstacles.push_back(obstacle);

  obstacle.location.x = -5;
  obstacle.location.y = 5.5;
  obstacle.rotation.yaw = 1.57;
  obstacles.push_back(obstacle);

  obstacle.location.x = 8;
  obstacle.location.y = 5;
  obstacle.rotation.yaw = 0.3;
  obstacles.push_back(obstacle);

  obstacle.location.x = 12;
  obstacle.location.y = 6;
  obstacle.rotation.yaw = 0;
  obstacles.push_back(obstacle);

  obstacle.location.x = -1;
  obstacle.location.y = 2;
  obstacle.rotation.yaw = 0.5;
  obstacles.push_back(obstacle);

  obstacle.location.x = 11.5;
  obstacle.location.y = 6;
  obstacle.rotation.yaw = 0.34;
  obstacles.push_back(obstacle);

  std::vector<bool> expected{true, false, false, true, true, false, true};
  auto result = collision_checker(ego_vehicle, obstacles);
  std::cout << (expected == result ? "PASS" : "FAIL") << std::endl;

  return 0;
}


/*
(env) Ruis-MacBook-Pro-15:33_exercise_circle_based_collision_detection ruiwang$ ./output.out
Test Case 1:
c 0, c2 0 c_x: 9.06031 c_y: 5.15798 a_x: 9.05725 a_y: 5.16651 distance: 0.00906584, RadiiSum: 3, collision: 1
c 0, c2 0 c_x: 9.06031 c_y: 5.15798 a_x: 2.09525 a_y: -2.42594 distance: 10.297, RadiiSum: 3, collision: 0
c 0, c2 1 c_x: 9.06031 c_y: 5.15798 a_x: 3.90475 a_y: -1.57406 distance: 8.47939, RadiiSum: 3, collision: 0
c 0, c2 2 c_x: 9.06031 c_y: 5.15798 a_x: 4.8095 a_y: -1.14812 distance: 7.60501, RadiiSum: 3, collision: 0
c 1, c2 0 c_x: 10.9397 c_y: 5.84202 a_x: 2.09525 a_y: -2.42594 distance: 12.1072, RadiiSum: 3, collision: 0
c 1, c2 1 c_x: 10.9397 c_y: 5.84202 a_x: 3.90475 a_y: -1.57406 distance: 10.222, RadiiSum: 3, collision: 0
c 1, c2 2 c_x: 10.9397 c_y: 5.84202 a_x: 4.8095 a_y: -1.14812 distance: 9.29738, RadiiSum: 3, collision: 0
c 2, c2 0 c_x: 11.8794 c_y: 6.18404 a_x: 2.09525 a_y: -2.42594 distance: 13.0331, RadiiSum: 3, collision: 0
c 2, c2 1 c_x: 11.8794 c_y: 6.18404 a_x: 3.90475 a_y: -1.57406 distance: 11.1258, RadiiSum: 3, collision: 0
c 2, c2 2 c_x: 11.8794 c_y: 6.18404 a_x: 4.8095 a_y: -1.14812 distance: 10.1855, RadiiSum: 3, collision: 0
c 0, c2 0 c_x: 9.06031 c_y: 5.15798 a_x: -5.0008 a_y: 4.5 distance: 14.0765, RadiiSum: 3, collision: 0
c 0, c2 1 c_x: 9.06031 c_y: 5.15798 a_x: -4.9992 a_y: 6.5 distance: 14.1234, RadiiSum: 3, collision: 0
c 0, c2 2 c_x: 9.06031 c_y: 5.15798 a_x: -4.99841 a_y: 7.5 distance: 14.2525, RadiiSum: 3, collision: 0
c 1, c2 0 c_x: 10.9397 c_y: 5.84202 a_x: -5.0008 a_y: 4.5 distance: 15.9969, RadiiSum: 3, collision: 0
c 1, c2 1 c_x: 10.9397 c_y: 5.84202 a_x: -4.9992 a_y: 6.5 distance: 15.9525, RadiiSum: 3, collision: 0
c 1, c2 2 c_x: 10.9397 c_y: 5.84202 a_x: -4.99841 a_y: 7.5 distance: 16.0241, RadiiSum: 3, collision: 0
c 2, c2 0 c_x: 11.8794 c_y: 6.18404 a_x: -5.0008 a_y: 4.5 distance: 16.964, RadiiSum: 3, collision: 0
c 2, c2 1 c_x: 11.8794 c_y: 6.18404 a_x: -4.9992 a_y: 6.5 distance: 16.8815, RadiiSum: 3, collision: 0
c 2, c2 2 c_x: 11.8794 c_y: 6.18404 a_x: -4.99841 a_y: 7.5 distance: 16.929, RadiiSum: 3, collision: 0
c 0, c2 0 c_x: 9.06031 c_y: 5.15798 a_x: 7.04466 a_y: 4.70448 distance: 2.06603, RadiiSum: 3, collision: 1
c 0, c2 0 c_x: 9.06031 c_y: 5.15798 a_x: 11 a_y: 6 distance: 2.11457, RadiiSum: 3, collision: 1
c 0, c2 0 c_x: 9.06031 c_y: 5.15798 a_x: -1.87758 a_y: 1.52057 distance: 11.5268, RadiiSum: 3, collision: 0
c 0, c2 1 c_x: 9.06031 c_y: 5.15798 a_x: -0.122417 a_y: 2.47943 distance: 9.56541, RadiiSum: 3, collision: 0
c 0, c2 2 c_x: 9.06031 c_y: 5.15798 a_x: 0.755165 a_y: 2.95885 distance: 8.59137, RadiiSum: 3, collision: 0
c 1, c2 0 c_x: 10.9397 c_y: 5.84202 a_x: -1.87758 a_y: 1.52057 distance: 13.5262, RadiiSum: 3, collision: 0
c 1, c2 1 c_x: 10.9397 c_y: 5.84202 a_x: -0.122417 a_y: 2.47943 distance: 11.5619, RadiiSum: 3, collision: 0
c 1, c2 2 c_x: 10.9397 c_y: 5.84202 a_x: 0.755165 a_y: 2.95885 distance: 10.5848, RadiiSum: 3, collision: 0
c 2, c2 0 c_x: 11.8794 c_y: 6.18404 a_x: -1.87758 a_y: 1.52057 distance: 14.5259, RadiiSum: 3, collision: 0
c 2, c2 1 c_x: 11.8794 c_y: 6.18404 a_x: -0.122417 a_y: 2.47943 distance: 12.5606, RadiiSum: 3, collision: 0
c 2, c2 2 c_x: 11.8794 c_y: 6.18404 a_x: 0.755165 a_y: 2.95885 distance: 11.5823, RadiiSum: 3, collision: 0
c 0, c2 0 c_x: 9.06031 c_y: 5.15798 a_x: 10.5572 a_y: 5.66651 distance: 1.58096, RadiiSum: 3, collision: 1

PASS
*/