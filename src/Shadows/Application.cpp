#include "Application.h"

#include "imgui.h"
#include "imgui-SFML.h"

    void Player::update()
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

    void Player::handleEvent(sf::Event event)
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


Application::Application(cdt::Vector2i box_size)
        : m_window({800, 600}, "Shadows"), m_map(box_size, box_size), m_cdt(box_size), m_vision(m_cdt)
    {
        m_window.setFramerateLimit(60.f);
        //! init gui
        ImGui::SFML::Init(m_window);

        //! init player pos and view
        m_player.pos = asFloat(box_size) / 2.f;
        float aspect = 6. / 8.;
        sf::View view;
        view.setCenter(sf::Vector2f(box_size.x / 2.f, box_size.y / 2.f));
        view.setSize(box_size.x, box_size.y * aspect);
        m_window.setView(view);

        //! initialize textures and shaders
        m_light_cut.create(800, 600);
        m_light_cut.setSmooth(true);
        m_texture_pass[0].create(800, 600);
        m_texture_pass[1].create(800, 600);
        m_texture_pass[0].setSmooth(true);
        m_texture_pass[1].setSmooth(true);
        m_light_texture.create(m_window.getSize().x, m_window.getSize().y);
        m_light_texture.setSmooth(true);

        m_light_smoother.loadFromFile("../Resources/Shaders/basic.vert", "../Resources/Shaders/lightSmootherRGB.frag");
        m_full_pass.loadFromFile("../Resources/Shaders/basic.vert", "../Resources/Shaders/fullpass.frag");
    }

    void Application::run()
    {
        while (m_window.isOpen())
        {

            m_window.clear(sf::Color::White);

            sf::Event event;
            while (m_window.pollEvent(event))
            {
                handleEvent(event);
            }
            update(0.016f);
            draw();
        }
    }

    void Application::update(float dt)
    {
        m_player.update();
    }

    void Application::draw()
    {

        ImGui::SFML::Update(m_window, m_clock.restart());
        ImGui::Begin("Control Panel"); // Create a window called "Hello, world!" and append into it.
        ImGui::SliderFloat("vision distance", &m_player.vision_distance, 0, 100);
        ImGui::ColorPicker3("vision color", m_light_color);
        ImGui::End();

        ImGui::SFML::Render(m_window);

        for (auto &tri : m_cdt.m_triangles)
        {
            sf::Vector2f v1(tri.verts[0].x, tri.verts[0].y);
            sf::Vector2f v2(tri.verts[1].x, tri.verts[1].y);
            sf::Vector2f v3(tri.verts[2].x, tri.verts[2].y);
            tri.is_constrained[0] ? drawLine(m_window, v1, v2, sf::Color::Red) : drawLine(m_window, v1, v2);
            tri.is_constrained[1] ? drawLine(m_window, v2, v3, sf::Color::Red) : drawLine(m_window, v2, v3);
            tri.is_constrained[2] ? drawLine(m_window, v3, v1, sf::Color::Red) : drawLine(m_window, v3, v1);
        }

        sf::RectangleShape player_rect;
        player_rect.setPosition(m_player.pos.x, m_player.pos.y);
        player_rect.setSize({1.f, 1.f});
        player_rect.setFillColor(sf::Color::Red);
        m_window.draw(player_rect);

        m_vision.contrstuctField(m_player.pos, {0, 0});
        auto vision_poly = m_vision.getDrawVertices();

        sf::Color base_color = {0, 0, 0, 255};
        m_light_texture.clear(base_color);

        m_light_texture.setView(m_window.getView());
        m_light_texture.draw(vision_poly);
        m_light_texture.display();

        m_light_cut.clear(base_color);

        sf::VertexArray circle_verts;
        circle_verts.resize(50 + 2);
        circle_verts.setPrimitiveType(sf::TriangleFan);
        sf::Color color(m_light_color[0] * 255, m_light_color[1] * 255, m_light_color[2] * 255, 255);
        circle_verts[0] = {{m_player.pos.x, m_player.pos.y}, color};
        for (auto i = 0; i < 50; ++i)
        {
            float angle = i / 50. * 360.f;
            cdt::Vector2f r = m_player.pos + m_player.vision_distance * angle2dir(angle);
            circle_verts[i + 1].position = {r.x, r.y};
            circle_verts[i + 1].color = base_color;
            circle_verts[i + 1].color.a = 30;
        }
        circle_verts[50 + 1] = circle_verts[1];

        sf::RenderStates states;
        states.blendMode.colorSrcFactor = sf::BlendMode::SrcColor;
        states.blendMode.colorDstFactor = sf::BlendMode::DstColor;
        states.blendMode.colorEquation = sf::BlendMode::Add;

        m_light_cut.setView(m_window.getView());
        m_light_cut.draw(circle_verts, states);
        m_light_cut.display();

        combineTextures(m_light_cut, m_light_texture, m_full_pass, sf::BlendMultiply);
        m_light_texture.display();
        smoothLights(m_light_texture, m_window);

        m_window.display();

    }

    void Application::handleEvent(sf::Event event)
    {

        ImGui::SFML::ProcessEvent(event);

        auto mouse_pos_sf = m_window.mapPixelToCoords(sf::Mouse::getPosition(m_window));
        auto mouse_pos = cdt::Vector2f{mouse_pos_sf.x, mouse_pos_sf.y};
        if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Right)
        {
            m_cdt.reset();
            m_map.changeTiles(MapGrid::Tile::Wall, mouse_pos, {2, 2});
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

    void Application::smoothLights(const sf::RenderTexture &light_texture, sf::RenderTarget &scene)
    {

        combineTextures(light_texture, m_texture_pass[0], m_light_smoother, sf::BlendNone);

        for (int i = 0; i < 5; ++i)
        {
            m_light_smoother.setUniform("vertical", false);
            combineTextures(m_texture_pass[0], m_texture_pass[1], m_light_smoother, sf::BlendNone);

            m_light_smoother.setUniform("vertical", true);
            combineTextures(m_texture_pass[1], m_texture_pass[0], m_light_smoother, sf::BlendNone);
        }

        auto old_view = scene.getView();
        scene.setView(scene.getDefaultView());
        combineTextures(m_texture_pass[0], scene, m_full_pass, sf::BlendMultiply);
        scene.setView(old_view);
    }

void inline drawLine(sf::RenderWindow &window, sf::Vector2f p1, sf::Vector2f p2, sf::Color color)
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

void inline combineTextures(const sf::RenderTexture &source, sf::RenderTarget &destination,
                     sf::Shader &shader, sf::BlendMode blend_mode)
{
    auto old_view = destination.getView();

    sf::VertexArray texture_rect;
    texture_rect.resize(4);
    texture_rect.setPrimitiveType(sf::Quads);
    sf::Vector2f rect_size(destination.getSize().x, destination.getSize().y);
    texture_rect[0] = sf::Vertex{{0, 0}, sf::Color::Transparent, {0, 1}};
    texture_rect[1] = sf::Vertex{{rect_size.x, 0}, sf::Color::Transparent, {1, 1}};
    texture_rect[2] = sf::Vertex{{rect_size.x, rect_size.y}, sf::Color::Transparent, {1, 0}};
    texture_rect[3] = sf::Vertex{{0, rect_size.y}, sf::Color::Transparent, {0, 0}};

    shader.setUniform("image", source.getTexture());
    sf::RenderStates states;
    states.blendMode = blend_mode;
    states.shader = &shader;

    sf::View view;
    view.setCenter(rect_size / 2.f);
    view.setSize(rect_size);
    destination.setView(view);
    destination.draw(texture_rect, states);
    destination.setView(old_view);
}