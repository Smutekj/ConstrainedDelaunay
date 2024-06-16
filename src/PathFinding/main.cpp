#include "Application.h"


int main(int argc, char **argv)
{
    Application app({100, 100});
    app.run();
    return 0;
}


struct Boid
{
    cdt::Vector2f pos;
    cdt::Vector2f vel;
    cdt::Vector2f acc;

    cdt::Vector2f path;
    cdt::Vector2f path_next;

    cdt::Vector2f target;
    float radius = 2.f;

    void draw(sf::RenderWindow &window)
    {
        sf::CircleShape c;
        c.setRadius(radius);
        c.setFillColor(sf::Color::Blue);
        c.setPosition({pos.x - radius, pos.y - radius});
        window.draw(c);
    }
};
