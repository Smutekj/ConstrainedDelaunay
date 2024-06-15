#include "../Triangulation.h"
#include "../Shadows/MapGrid.h"
#include "PathFinder.h"

#include <SFML/Graphics.hpp>

#include "imgui.h"
#include "imgui-SFML.h"

void drawLine(sf::RenderWindow &window, sf::Vector2f from, sf::Vector2f to, sf::Color color = sf::Color::Green);

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

    bool moving_up = false;
    bool moving_down = false;
    bool moving_left = false;
    bool moving_right = false;
};

struct Boids
{

    float m_repulsion;
    float m_path;

    void update(float dt)
    {
    }

    std::vector<Boid> m_boids;
};

class Application
{

public:
    Application(cdt::Vector2i box_size)
        : m_window({800, 600}, "Pathfinding"), m_map(box_size, box_size), m_cdt(box_size), m_pf(m_cdt)
    {

        m_window.setFramerateLimit(60.f);
        float aspect = 6. / 8.;

        m_player.pos = asFloat(box_size) / 2.f;

        ImGui::SFML::Init(m_window);

        sf::View view;
        view.setCenter(sf::Vector2f(box_size.x / 2.f, box_size.y / 2.f));
        view.setSize(box_size.x, box_size.y * aspect);
        m_window.setView(view);
    }

    void run()
    {
        while (m_window.isOpen())
        {
            sf::Event event;
            while (m_window.pollEvent(event))
            {
                handleEvent(event);
            }
            update(0.016f);
            draw();
        }
    }

    void update(float dt)
    {
        m_player.update();
    }

    void draw()
    {
        m_window.clear(sf::Color::White);
        drawUI();
        drawWalls();
        
        if (m_draw_path)
        {
            drawPath();
        }
        if (m_draw_path_funnel)
        {
            drawFunnel();
        }
        if (m_draw_triangulation)
        {
            drawTriangulation();
        }

        //! draw Player
        sf::RectangleShape player_rect;
        player_rect.setPosition(m_player.pos.x, m_player.pos.y);
        player_rect.setSize({1.f, 1.f});
        player_rect.setFillColor(sf::Color::Red);
        m_window.draw(player_rect);

        m_window.display();
    }

    void handleEvent(sf::Event event)
    {
        ImGui::SFML::ProcessEvent(event);

        auto mouse_pos_sf = m_window.mapPixelToCoords(sf::Mouse::getPosition(m_window));
        auto mouse_pos = cdt::Vector2f{mouse_pos_sf.x, mouse_pos_sf.y};
        if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right) && sf::Keyboard::isKeyPressed(sf::Keyboard::LControl))
        {
            m_cdt.reset();
            m_map.changeTiles(MapGridDiagonal::Tile::Wall, mouse_pos, {2, 2});
            m_map.extractBoundaries();
            m_map.transformCorners();
            auto edges = m_map.extractEdges();
            std::vector<cdt::EdgeVInd> edge_inds;
            for (auto &e : edges)
            {
                cdt::EdgeVInd e_ind;
                auto v_ind1 = m_cdt.insertVertexAndGetData(e.from).overlapping_vertex;
                e_ind.from = (v_ind1 == -1 ? m_cdt.m_vertices.size() - 1 : v_ind1);

                auto v_ind2 = m_cdt.insertVertexAndGetData(e.to()).overlapping_vertex;
                e_ind.to = (v_ind2 == -1 ? m_cdt.m_vertices.size() - 1 : v_ind2);

                edge_inds.push_back(e_ind);
            }
            for (auto &e : edge_inds)
            {
                m_cdt.insertConstraint(e);
            }

            m_pf.update();
        }
        else if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Right)
        {
            m_path = m_pf.doPathFinding({m_player.pos.x, m_player.pos.y}, mouse_pos, 1.f);
        }

        if (event.type == sf::Event::MouseWheelMoved)
        {
            auto view = m_window.getView();
            if (event.mouseWheelScroll.wheel > 0)
            {
                view.zoom(0.9f);
            }
            else
            {
                view.zoom(1. / 0.9f);
            }
            m_window.setView(view);
        }

        if (event.type == sf::Event::Closed)
        {
            m_window.close();
        }

        m_player.handleEvent(event);
    }

private:
    void drawTriangulation()
    {
        for (auto &tri : m_cdt.m_triangles)
        {
            sf::Vector2f v1(tri.verts[0].x, tri.verts[0].y);
            sf::Vector2f v2(tri.verts[1].x, tri.verts[1].y);
            sf::Vector2f v3(tri.verts[2].x, tri.verts[2].y);
            if (!tri.is_constrained[0])
            {
                drawLine(m_window, v1, v2, sf::Color::Green);
            }
            if (!tri.is_constrained[1])
            {
                drawLine(m_window, v2, v3, sf::Color::Green);
            }
            if (!tri.is_constrained[2])
            {
                drawLine(m_window, v3, v1, sf::Color::Green);
            }
        }
    }

    void drawWalls()
    {
        for (auto &tri : m_cdt.m_triangles)
        {
            sf::Vector2f v1(tri.verts[0].x, tri.verts[0].y);
            sf::Vector2f v2(tri.verts[1].x, tri.verts[1].y);
            sf::Vector2f v3(tri.verts[2].x, tri.verts[2].y);
            if (tri.is_constrained[0])
            {
                drawLine(m_window, v1, v2, sf::Color::Red);
            }
            if (tri.is_constrained[1])
            {
                drawLine(m_window, v2, v3, sf::Color::Red);
            }
            if (tri.is_constrained[2])
            {
                drawLine(m_window, v3, v1, sf::Color::Red);
            }
        }
    }

    void drawPath()
    {
        for (int k = 0; k < static_cast<int>(m_path.path.size()) - 1; ++k)
        {
            sf::Vector2f v1(m_path.path[k].x, m_path.path[k].y);
            sf::Vector2f v2(m_path.path[k + 1].x, m_path.path[k + 1].y);
            drawLine(m_window, v1, v2, sf::Color::Yellow);
        }
    }
    void drawFunnel()
    {
        for (int k = 0; k < (int)m_path.funnel.size() - 1; ++k)
        {
            auto portal1 = m_path.funnel[k];
            auto portal2 = m_path.funnel[k + 1];
            sf::Vector2f v_left(portal1.first.x, portal1.first.y);
            sf::Vector2f v_right(portal1.second.x, portal1.second.y);
            sf::Vector2f v_left2(portal2.first.x, portal2.first.y);
            sf::Vector2f v_right2(portal2.second.x, portal2.second.y);
            drawLine(m_window, v_left, v_left2, sf::Color::Cyan);
            drawLine(m_window, v_right, v_right2, sf::Color::Magenta);
        }
    }

    void drawUI()
    {
        ImGui::SFML::Update(m_window, m_clock.restart());

        ImGui::Begin("Control Panel"); // Create a window called "Hello, world!" and append into it.
        if (ImGui::Button("Draw Triangulation"))
        {
            m_draw_triangulation = !m_draw_triangulation;
        }
        if (ImGui::Button("Draw Path"))
        {
            m_draw_path = !m_draw_path;
        }
        if (ImGui::Button("Draw Funnel"))
        {
            m_draw_path_funnel = !m_draw_path_funnel;
        }
        ImGui::End();

        ImGui::SFML::Render(m_window);
    }

private:
    sf::RenderWindow m_window;

    sf::Clock m_clock;

    Player m_player;

    bool m_draw_path = true;
    bool m_draw_path_funnel = true;
    bool m_draw_triangulation = false;

    PathFinder::PathData m_path;

    cdt::Triangulation<Triangle> m_cdt;
    PathFinder m_pf;
    MapGridDiagonal m_map;
};

int main(int argc, char **argv)
{
    Application app({100, 100});
    app.run();
    return 0;
}

void drawLine(sf::RenderWindow &window, sf::Vector2f p1, sf::Vector2f p2, sf::Color color)
{
    sf::RectangleShape line;
    line.setFillColor(color);
    line.setOrigin({0, 0.1});
    line.setPosition(p1);
    sf::Vector2f dr = p2 - p1;
    line.setSize({cdt::norm(dr), 0.2});
    line.setRotation(std::atan2(dr.y, dr.x) * 180.f / M_PI);

    window.draw(line);
}