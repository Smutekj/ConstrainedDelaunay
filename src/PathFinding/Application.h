#pragma once

#include <SFML/Graphics.hpp>

#include "../Vector2.h"
#include "../Triangulation.h"
#include "PathFinder.h"
#include "../Shadows/MapGrid.h"

struct Player
{
    cdt::Vector2f pos;

    void update()
    {
        cdt::Vector2f vel = {0, 0};
        if (moving_up)
            vel.y -= 0.1f;
        if (moving_down)
            vel.y += 0.1f;
        if (moving_left)
            vel.x -= 0.1f;
        if (moving_right)
            vel.x += 0.1f;

        if (moving_down + moving_left + moving_right + moving_up == 2)
        {
            vel /= std::sqrt(2.f);
        }
        pos += vel;
    }

    void handleEvent(sf::Event event)
    {

        if (event.type == sf::Event::KeyPressed)
        {
            if (event.key.code == sf::Keyboard::W)
                moving_up = true;
            else if (event.key.code == sf::Keyboard::A)
                moving_left = true;
            else if (event.key.code == sf::Keyboard::S)
                moving_down = true;
            else if (event.key.code == sf::Keyboard::D)
                moving_right = true;
        }
        else if (event.type == sf::Event::KeyReleased)
        {
            if (event.key.code == sf::Keyboard::W)
                moving_up = false;
            else if (event.key.code == sf::Keyboard::A)
                moving_left = false;
            else if (event.key.code == sf::Keyboard::S)
                moving_down = false;
            else if (event.key.code == sf::Keyboard::D)
                moving_right = false;
        }
    }

    float radius = 0.99f;

    bool moving_up = false;
    bool moving_down = false;
    bool moving_left = false;
    bool moving_right = false;
};

class Application
{

public:
    Application(cdt::Vector2i box_size);

    void run();

    void update(float dt);

    void draw();

    void handleEvent(sf::Event event);
private:
    void drawTriangulation();

    void drawTriInds();

    void drawWalls();

    void drawPath();
    void drawFunnel();

    void drawUI();

private:
    sf::RenderWindow m_window;

    sf::Clock m_clock;

    Player m_player;

    bool m_draw_path = true;
    bool m_draw_path_funnel = true;
    bool m_draw_triangulation = false;
    bool m_draw_tri_inds = false;

    sf::Font m_font;

    PathFinder::PathData m_path;

    cdt::Triangulation<Triangle> m_cdt;
    PathFinder m_pf;
    MapGridDiagonal m_map;
};
