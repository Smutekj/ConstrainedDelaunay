#include "../Triangulation.h"

#include <SFML/Graphics.hpp>

#include "imgui.h"
#include "imgui-SFML.h"

void drawLine(sf::RenderWindow &window, sf::Vector2f from, sf::Vector2f to, sf::Color color = sf::Color::Green);

int main(int argc, char **argv)
{

    sf::Vector2i box_size = {100, 100};
    cdt::Triangulation cdt({box_size.x, box_size.y});

    sf::RenderWindow window({800, 600}, "Demo");
    window.setFramerateLimit(60.f);
    float aspect = 6. / 8.;

    ImGui::SFML::Init(window);
    sf::Clock m_clock;

    sf::View view;
    view.setCenter(static_cast<sf::Vector2f>(box_size) / 2.f);
    view.setSize(box_size.x, box_size.y * aspect);
    window.setView(view);

    std::deque<cdt::VertInd> picked_vertex_inds;

    while (window.isOpen())
    {

        window.clear(sf::Color::White);

        sf::Event event;
        while (window.pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(event);

            auto mouse_pos_sf = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            auto mouse_pos = cdt::Vector2f{mouse_pos_sf.x, mouse_pos_sf.y};
            if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Right)
            {
                cdt.insertVertex(mouse_pos);
            }
            if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left)
            {
                auto tri_ind = cdt.findTriangle(mouse_pos);
                if (tri_ind != -1)
                {
                    auto &tri = cdt.m_triangles.at(tri_ind);
                    float min_dist = 2 * box_size.x;
                    cdt::VertInd closest_vert_ind;
                    for (int i = 0; i < 3; ++i)
                    {
                        float new_dist = dist(asFloat(tri.verts[i]), mouse_pos);
                        if (new_dist < min_dist)
                        {
                            min_dist = new_dist;
                            closest_vert_ind = cdt.m_tri_ind2vert_inds[tri_ind][i];
                        }
                    }
                    picked_vertex_inds.push_back(closest_vert_ind);
                }

                if (picked_vertex_inds.size() > 2)
                {
                    picked_vertex_inds.pop_front();
                }
            }
            if (event.type == sf::Event::KeyReleased && event.key.code == sf::Keyboard::C)
            {
                if (picked_vertex_inds.size() == 2)
                {
                    cdt.insertConstraint({picked_vertex_inds[0], picked_vertex_inds[1]});
                }
            }
            if (event.type == sf::Event::MouseWheelMoved)
            {
                view = window.getView();
                if (event.mouseWheelScroll.wheel > 0)
                {
                    view.zoom(0.9f);
                }
                else
                {
                    view.zoom(1. / 0.9f);
                }
                window.setView(view);
            }

            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
        }

        ImGui::SFML::Update(window, m_clock.restart());
        ImGui::Begin("Control Panel"); // Create a window called "Hello, world!" and append into it.
        if (ImGui::Button("Constrain Selected") && picked_vertex_inds.size() == 2)
        {
            cdt.insertConstraint({picked_vertex_inds[0], picked_vertex_inds[1]});
        }
        ImGui::End();

        ImGui::SFML::Render(window);

        for (auto &tri : cdt.m_triangles)
        {
            sf::Vector2f v1(tri.verts[0].x, tri.verts[0].y);
            sf::Vector2f v2(tri.verts[1].x, tri.verts[1].y);
            sf::Vector2f v3(tri.verts[2].x, tri.verts[2].y);
            tri.is_constrained[0] ? drawLine(window, v1, v2, sf::Color::Red) : drawLine(window, v1, v2);
            tri.is_constrained[1] ? drawLine(window, v2, v3, sf::Color::Red) : drawLine(window, v2, v3);
            tri.is_constrained[2] ? drawLine(window, v3, v1, sf::Color::Red) : drawLine(window, v3, v1);
        }

        sf::CircleShape selected_vert_circle;
        selected_vert_circle.setRadius(0.5f);
        selected_vert_circle.setFillColor(sf::Color::Blue);
        for (auto v_ind : picked_vertex_inds)
        {
            auto vert = cdt.m_vertices.at(v_ind);
            sf::Vector2f pos(vert.x, vert.y);
            selected_vert_circle.setPosition(pos.x - 0.5, pos.y - 0.5);
            window.draw(selected_vert_circle);
        }

        window.display();
    }

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