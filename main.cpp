#include <SFML/Graphics.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/VideoMode.hpp>
#include <cmath>
#include "../vec.h"
#include "../ui.h"

int main() {
  uint windowWidth = 1600;
  uint windowHeight = 900;
  
  sf::RenderWindow window(sf::VideoMode({windowWidth, windowHeight}), "blac hol");
  while (window.isOpen()) {
    while (const std::optional event = window.pollEvent()) {
      if (event->is<sf::Event::Closed>()) {
        window.close();
      }
    }

    window.clear();
    window.display();
  }
  return 0;
}
