#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/VideoMode.hpp>
#include <cmath>
#include <vector>
#include "../vec.h"
#include "../ui.h"

struct Ray {
  Vector2 pos;
  Vector2 direc;
};

struct Blackhole {
  Vector2 position;
  double rs;
};

struct Camera {
  Vector2 position;
};

double christoffel(int u, int a, int b, double r, double rs) {
    if (u==0 && a==0 && b==0) return -rs / (2.0*r*(r-rs));
    if (u==0 && a==1 && b==1) return -(r - rs);
    if (u==1 && a==0 && b==1) return 1.0/r;
    if (u==1 && a==1 && b==0) return 1.0/r;
    return 0.0;
}

struct PolarVec {
    double r, phi;
};

PolarVec geodesic_acceleration(PolarVec pos, PolarVec vel, double rs) {
    double v[2] = {vel.r, vel.phi};
    double a[2] = {0.0, 0.0};
    for (int u = 0; u < 2; u++)
        for (int alpha = 0; alpha < 2; alpha++)
            for (int beta = 0; beta < 2; beta++)
                a[u] -= christoffel(u, alpha, beta, pos.r, rs) * v[alpha] * v[beta];
    return {a[0], a[1]};
}

void step(PolarVec& pos, PolarVec& vel, double rs, double dlambda) {
    // k1
    PolarVec a1 = geodesic_acceleration(pos, vel, rs);
    PolarVec dp1 = {vel.r, vel.phi};
    PolarVec dv1 = {a1.r, a1.phi};

    // k2
    PolarVec pos2 = {pos.r + dp1.r * dlambda/2, pos.phi + dp1.phi * dlambda/2};
    PolarVec vel2 = {vel.r + dv1.r * dlambda/2, vel.phi + dv1.phi * dlambda/2};
    PolarVec a2 = geodesic_acceleration(pos2, vel2, rs);
    PolarVec dp2 = {vel2.r, vel2.phi};
    PolarVec dv2 = {a2.r, a2.phi};

    // k3
    PolarVec pos3 = {pos.r + dp2.r * dlambda/2, pos.phi + dp2.phi * dlambda/2};
    PolarVec vel3 = {vel.r + dv2.r * dlambda/2, vel.phi + dv2.phi * dlambda/2};
    PolarVec a3 = geodesic_acceleration(pos3, vel3, rs);
    PolarVec dp3 = {vel3.r, vel3.phi};
    PolarVec dv3 = {a3.r, a3.phi};

    // k4
    PolarVec pos4 = {pos.r + dp3.r * dlambda, pos.phi + dp3.phi * dlambda};
    PolarVec vel4 = {vel.r + dv3.r * dlambda, vel.phi + dv3.phi * dlambda};
    PolarVec a4 = geodesic_acceleration(pos4, vel4, rs);
    PolarVec dp4 = {vel4.r, vel4.phi};
    PolarVec dv4 = {a4.r, a4.phi};

    // combine
    pos.r   += (dlambda/6) * (dp1.r   + 2*dp2.r   + 2*dp3.r   + dp4.r);
    pos.phi += (dlambda/6) * (dp1.phi + 2*dp2.phi + 2*dp3.phi + dp4.phi);
    vel.r   += (dlambda/6) * (dv1.r   + 2*dv2.r   + 2*dv3.r   + dv4.r);
    vel.phi += (dlambda/6) * (dv1.phi + 2*dv2.phi + 2*dv3.phi + dv4.phi);
}

int main() {
  uint windowWidth = 1600;
  uint windowHeight = 900;

  std::vector<sf::CircleShape> trail;

  Ray ray;
  ray.pos = Vector2(-10.0, 2.0);
  ray.direc = Vector2(1.0, 0.0);

  Camera cam;
  cam.position = Vector2(0.0, 0.0);

  Blackhole blackhole;
  blackhole.position = Vector2(0.0, 0.0);
  blackhole.rs = 1.0;
  float screenRadius = worldToScreenLength(blackhole.rs, windowWidth, 20.0f);
  sf::CircleShape blackholeCircle(screenRadius);
  blackholeCircle.setPosition(worldToPixel(blackhole.position, cam.position, windowWidth, windowHeight, 20.0f));
  blackholeCircle.setOrigin(sf::Vector2f(screenRadius, screenRadius));
  blackholeCircle.setFillColor(sf::Color::Red);

  Vector2 relativePos = ray.pos - blackhole.position;
  PolarVec polarPos;

  polarPos.r  = (float) Vector2::length(relativePos);
  polarPos.phi = (float) std::atan2(relativePos.y, relativePos.x);

  PolarVec startPos = polarPos;

  PolarVec polarDirec;

  polarDirec.r = (ray.pos.x * ray.direc.x + ray.pos.y * ray.direc.y) / polarPos.r;
  polarDirec.phi = (ray.pos.x * ray.direc.y - ray.pos.y * ray.direc.x) / (polarPos.r*polarPos.r);

  
  sf::RenderWindow window(sf::VideoMode({windowWidth, windowHeight}), "blac hol");
  window.setFramerateLimit(240);

  //   ---- main loop ----
  while (window.isOpen()) {
    while (const std::optional event = window.pollEvent()) {
      if (event->is<sf::Event::Closed>()) {
        window.close();
      }
    }
    window.clear();
    // polarPos = startPos;
    // for (int i = 0; i < 500; ++i) {
      double cx = polarPos.r * std::cos(polarPos.phi);
      double cy = polarPos.r * std::sin(polarPos.phi);

      sf::CircleShape circle(2, 10);
      circle.setPosition(worldToPixel(Vector2(cx, cy), cam.position, windowWidth, windowHeight, 20.0f));
      circle.setOrigin(sf::Vector2f(5.0f, 5.0f));
      circle.setFillColor(sf::Color::White);
      trail.push_back(circle);

      step(polarPos, polarDirec, blackhole.rs, 0.01);
    // }

    window.draw(blackholeCircle);
    for (sf::CircleShape circle : trail) {
      window.draw(circle);
    }

    window.display();
  }
  return 0;
}
