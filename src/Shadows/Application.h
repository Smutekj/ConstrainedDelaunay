#pragma once

#include <SFML/Graphics.hpp>

#include "VisibilityField.h"
#include "MapGrid.h"

void inline drawLine(sf::RenderWindow &window, sf::Vector2f from, sf::Vector2f to, sf::Color color = sf::Color::Green);

void inline combineTextures(const sf::RenderTexture &source, sf::RenderTarget &destination,
                            sf::Shader &shader, sf::BlendMode blend_mode);

struct Player
{

    void update();
    void handleEvent(sf::Event event);

    cdt::Vector2f pos;
    float vision_distance = 50.f;

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
    void smoothLights(const sf::RenderTexture &light_texture, sf::RenderTarget &scene);

private:
    sf::RenderWindow m_window;
    sf::Clock m_clock;

    sf::RenderTexture m_texture_pass[2];
    sf::RenderTexture m_light_cut;
    sf::RenderTexture m_light_texture;

    sf::Shader m_light_smoother;
    sf::Shader m_light_combiner;
    sf::Shader m_full_pass;

    Player m_player;

    float m_light_color[3] = {1, 1, 1};

    cdt::Triangulation<cdt::Vector2i> m_cdt;
    MapGrid m_map;
    VisionField m_vision;
};