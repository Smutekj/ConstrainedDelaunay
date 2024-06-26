#include "Triangulation.h"

#include <fstream>
#include <iostream>
#include <string>
#include <stack>
#include <queue>

#include <SFML/Graphics.hpp>

void inline drawLine(sf::RenderWindow &window, sf::Vector2f p1, sf::Vector2f p2, sf::Color color = sf::Color::Green)
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

sf::Font m_font;

void inline drawTriInds(cdt::Triangulation<cdt::Vector2i> &cdt, sf::RenderWindow &window)
{
    sf::Text num;
    num.setFont(m_font);
    num.setFillColor(sf::Color::Blue);
    for (int ind = 0; ind < cdt.m_triangles.size(); ++ind)
    {
        auto &tri = cdt.m_triangles.at(ind);
        num.setString(std::to_string(ind));
        auto center = asFloat(tri.verts[0] + tri.verts[1] + tri.verts[2]) / 3.f;
        num.setPosition(center.x, center.y); // - sf::Vector2f(num.getGlobalBounds().width, num.getLocalBounds().height)
        num.setScale(0.03f, 0.03f);
        window.draw(num);
    }
}

void inline drawTriangulation(cdt::Triangulation<cdt::Vector2i> &cdt, sf::RenderWindow &window)
{
    window.clear(sf::Color::White);
    int tri_ind = 0;
    for (auto &tri : cdt.m_triangles)
    {
        sf::Vector2f v1(tri.verts[0].x, tri.verts[0].y);
        sf::Vector2f v2(tri.verts[1].x, tri.verts[1].y);
        sf::Vector2f v3(tri.verts[2].x, tri.verts[2].y);
        tri.is_constrained[0] ? drawLine(window, v1, v2, sf::Color::Red) : drawLine(window, v1, v2);
        tri.is_constrained[1] ? drawLine(window, v2, v3, sf::Color::Red) : drawLine(window, v2, v3);
        tri.is_constrained[2] ? drawLine(window, v3, v1, sf::Color::Red) : drawLine(window, v3, v1);
    }
}

namespace cdt
{

    template <class Vertex>
    Triangulation<Vertex>::Triangulation(Vertex box_size)
        : m_boundary(box_size)
    {
        createBoundary(box_size);
    }

    //! \brief Clears all vertices triangles and edges
    template <class Vertex>
    void Triangulation<Vertex>::reset()
    {
        m_vertices.clear();
        m_triangles.clear();
        m_tri_ind2vert_inds.clear();
        m_fixed_edges.clear();
        m_last_found = 0;

        std::fill(m_cell2tri_ind.begin(), m_cell2tri_ind.end(), -1);
        createBoundary(m_boundary);
    }

    //! \brief searches for triangle containing query_point
    //! \param query_point point whose containing triangle we are looking for
    //! \param start_from_last_found whether we start looking from previously found triangle... uses search grid if false
    //!\returns index of a triangle containing query_point or -1 if no such triangle is found
    template <class Vertex>
    TriInd Triangulation<Vertex>::findTriangle(Vertex query_point, bool from_last_found)
    {

        TriInd tri_ind = m_last_found;
        if (!from_last_found)
        {
            auto cell_ind = m_grid->coordToCell(asFloat(query_point));
            tri_ind = m_cell2tri_ind.at(cell_ind);
            if (tri_ind == -1)
            { //! if there is no triangle yet in a cell we default to linear search
                for (TriInd i = 0; i < m_triangles.size(); ++i)
                {
                    if (isInTriangle(query_point, m_triangles[i]))
                    {
                        m_last_found = i;
                        return i;
                    }
                }
            }
        }

        Vertex v_current;

        if (isInTriangle(query_point, m_triangles[tri_ind]))
        {
            return tri_ind;
        }
        { //!  we look at triangle edges and find in which direction we need to go
            const auto &tri = m_triangles[tri_ind];

            const auto v0 = asFloat(tri.verts[0]);
            const auto v1 = asFloat(tri.verts[1]);
            const auto v2 = asFloat(tri.verts[2]);
            const auto r_start = (v0 + v1 + v2) / 3.f;

            if (segmentsIntersectOrTouch(r_start, asFloat(query_point), v0, v1))
            {
                tri_ind = tri.neighbours[0];
                v_current = tri.verts[0];
            }
            else if (segmentsIntersectOrTouch(r_start, asFloat(query_point), v1, v2))
            {
                tri_ind = tri.neighbours[1];
                v_current = tri.verts[1];
            }
            else if (segmentsIntersectOrTouch(r_start, asFloat(query_point), v2, v0))
            {
                tri_ind = tri.neighbours[2];
                v_current = tri.verts[2];
            }
            else
            {
                tri_ind = -1;
            } // throw std::runtime_error("we should never get here!");}
        }
        if (tri_ind == -1)
        { //! if we cannot find way (no idea why yet) we do brute force
            for (TriInd i = 0; i < m_triangles.size(); ++i)
            {
                if (isInTriangle(query_point, m_triangles[i]))
                {
                    m_last_found = i;
                    return i;
                }
            }
        }
        //! walk in the found diraction to triangle containing end_ind;
        while (!isInTriangle(query_point, m_triangles[tri_ind]))
        {
            const auto &tri = m_triangles[tri_ind];

            const auto index_in_tri = indexOf(v_current, tri);
            assert(index_in_tri != -1);

            const auto v1 = asFloat(tri.verts[next(index_in_tri)]);
            const auto v2 = asFloat(tri.verts[prev(index_in_tri)]);

            if (segmentsIntersectOrTouch(asFloat(v_current), asFloat(query_point), v1, v2))
            {
                tri_ind = tri.neighbours[next(index_in_tri)];
                v_current = tri.verts[next(index_in_tri)];
            }
            else
            {
                tri_ind = tri.neighbours[index_in_tri];
            }
        }

        m_last_found = tri_ind;
        return tri_ind;
    }

    //! \brief searches for triangle containing query_point
    //! \param query_point point whose containing triangle we are looking for
    //! \param start_from_last_found whether we start looking from previously found triangle... uses search grid if false
    //!\returns index of a triangle containing query_point or -1 if no such triangle is found
    template <class Vertex>
    TriInd Triangulation<Vertex>::findTriangle(cdt::Vector2f query_point, bool from_last_found)
    {

        if(!withinBoundary(query_point))
        {
            return -1;
        }

        TriInd tri_ind = m_last_found;
        if (!from_last_found)
        {
            auto cell_ind = m_grid->coordToCell(query_point);
            tri_ind = m_cell2tri_ind.at(cell_ind);
            if (tri_ind == -1)
            { //! if there is no triangle yet in a cell we default to linear search
                for (TriInd i = 0; i < m_triangles.size(); ++i)
                {
                    if (isInTriangle(query_point, m_triangles[i]))
                    {
                        m_last_found = i;
                        return i;
                    }
                }
            }
        }

        Vertex v_current;

        if (isInTriangle(query_point, m_triangles[tri_ind]))
        {
            return tri_ind;
        }
        { //!  we look at triangle edges and find in which direction we need to go
            const auto &tri = m_triangles[tri_ind];

            const auto v0 = asFloat(tri.verts[0]);
            const auto v1 = asFloat(tri.verts[1]);
            const auto v2 = asFloat(tri.verts[2]);
            const cdt::Vector2f r_start = (v0 + v1 + v2) / 3.f;

            if (segmentsIntersectOrTouch(r_start, query_point, v0, v1))
            {
                tri_ind = tri.neighbours[0];
                v_current = tri.verts[0];
            }
            else if (segmentsIntersectOrTouch(r_start, query_point, v1, v2))
            {
                tri_ind = tri.neighbours[1];
                v_current = tri.verts[1];
            }
            else if (segmentsIntersectOrTouch(r_start, query_point, v2, v0))
            {
                tri_ind = tri.neighbours[2];
                v_current = tri.verts[2];
            }
            else
            {
                tri_ind = -1;
            } // throw std::runtime_error("we should never get here!");}
        }
        if (tri_ind == -1)
        { //! if we cannot find way (no idea why yet) we do brute force
            for (TriInd i = 0; i < m_triangles.size(); ++i)
            {
                if (isInTriangle(query_point, m_triangles[i]))
                {
                    m_last_found = i;
                    return i;
                }
            }
        }
        //! walk in the found diraction to triangle containing query;
        while (!isInTriangle(query_point, m_triangles[tri_ind]))
        {
            const auto &tri = m_triangles[tri_ind];

            const auto index_in_tri = indexOf(v_current, tri);
            assert(index_in_tri != -1);

            const auto &v1 = asFloat(tri.verts[next(index_in_tri)]);
            const auto &v2 = asFloat(tri.verts[prev(index_in_tri)]);

            if (segmentsIntersectOrTouch(asFloat(v_current), query_point, v1, v2))
            {
                tri_ind = tri.neighbours[next(index_in_tri)];
                v_current = tri.verts[next(index_in_tri)];
            }
            else
            {
                tri_ind = tri.neighbours[index_in_tri];
            }
        }

        m_last_found = tri_ind; //! cache the result
        return tri_ind;
    }

    //! \brief creates supertriangle which contains specified boundary then
    //! \param boundary dimensions of a boundary contained in supertriangle
    template <class Vertex>
    void Triangulation<Vertex>::createSuperTriangle(cdt::Vector2i box_size)
    {
        m_boundary = box_size;

        m_grid = std::make_unique<Grid>(cdt::Vector2i{20, 20}, box_size);
        m_cell2tri_ind.resize(m_grid->getNCells(), -1);

        Triangle<Vertex> super_triangle;
        m_tri_ind2vert_inds.push_back({0, 1, 2});

        Vertex super_tri0 = {m_boundary.x / 2, m_boundary.y * 3};
        Vertex super_tri1 = {3 * m_boundary.x, -m_boundary.y / 2};
        Vertex super_tri2 = {-3 * m_boundary.x, -m_boundary.y / 2};

        // super_triangle.verts[0] = super_tri0;
        // super_triangle.verts[1] = super_tri1;
        // super_triangle.verts[2] = super_tri2;
        m_triangles.push_back(super_triangle);

        m_vertices.push_back(super_tri0);
        m_vertices.push_back(super_tri1);
        m_vertices.push_back(super_tri2);
    }

    //! \brief creates supertriangle which contains specified boundary then
    //!        inserts 4 vertices corresponding to the boundary (upper left point is [0,0])
    //! \param boundary dimensions of a boundary contained in supertriangle
    template <class Vertex>
    void Triangulation<Vertex>::createBoundaryAndSuperTriangle(cdt::Vector2i box_size)
    {

        m_grid = std::make_unique<Grid>(cdt::Vector2i{20, 20}, box_size);
        m_cell2tri_ind.resize(m_grid->getNCells(), -1);

        Triangle<Vertex> super_triangle;
        m_tri_ind2vert_inds.push_back({0, 1, 2});

        Vertex super_tri0 = {m_boundary.x / 2, m_boundary.y * 3};
        Vertex super_tri1 = {3 * m_boundary.x, -m_boundary.y / 2};
        Vertex super_tri2 = {-3 * m_boundary.x, -m_boundary.y / 2};

        super_triangle.verts[0] = super_tri0;
        super_triangle.verts[1] = super_tri1;
        super_triangle.verts[2] = super_tri2;
        m_triangles.push_back(super_triangle);

        m_vertices.push_back(super_tri0);
        m_vertices.push_back(super_tri1);
        m_vertices.push_back(super_tri2);

        Vertex v1 = {0, 0};
        Vertex v2 = Vertex{m_boundary.x, 0};
        Vertex v3 = m_boundary;
        Vertex v4 = Vertex{0, m_boundary.y};
        insertVertex(v1, false);
        assert(triangulationIsConsistent());
        insertVertex(v2, true);
        assert(triangulationIsConsistent());
        insertVertex(v3, true);
        assert(triangulationIsConsistent());
        insertVertex(v4, true);
        assert(triangulationIsConsistent());

        EdgeVInd e1 = {3, 4};
        EdgeVInd e2 = {4, 5};
        EdgeVInd e3 = {5, 6};
        EdgeVInd e4 = {6, 3};

        insertConstraint(e1);
        insertConstraint(e2);
        insertConstraint(e3);
        insertConstraint(e4);

        assert(triangulationIsConsistent());
    }

    //! \brief creates supertriangle which contains specified boundary then
    //!        inserts 4 vertices corresponding to the boundary (upper left point is [0,0])
    //! \param boundary dimensions of a boundary contained in supertriangle
    template <class Vertex>
    void Triangulation<Vertex>::createBoundary(cdt::Vector2i box_size)
    {

        m_boundary = box_size;

        m_grid = std::make_unique<Grid>(cdt::Vector2i{20, 20}, box_size);
        m_cell2tri_ind.resize(m_grid->getNCells(), -1);

        Triangle<Vertex> tri_up;
        Triangle<Vertex> tri_down;
        m_tri_ind2vert_inds.push_back({0, 2, 1});
        m_tri_ind2vert_inds.push_back({0, 3, 2});

        Vertex v0 = {0, 0};
        Vertex v1 = Vertex{m_boundary.x, 0};
        Vertex v2 = m_boundary;
        Vertex v3 = Vertex{0, m_boundary.y};
        tri_up.verts[0] = v0;
        tri_up.verts[1] = v2;
        tri_up.verts[2] = v1;
        tri_up.neighbours = {1, -1u, -1u};
        tri_up.is_constrained[0] = false;
        tri_up.is_constrained[1] = true;
        tri_up.is_constrained[2] = true;

        tri_down.verts[0] = v0;
        tri_down.verts[1] = v3;
        tri_down.verts[2] = v2;
        tri_down.neighbours = {-1u, -1u, 0};
        tri_down.is_constrained[0] = true;
        tri_down.is_constrained[1] = true;
        tri_down.is_constrained[2] = false;

        m_triangles.push_back(tri_up);
        m_triangles.push_back(tri_down);

        m_vertices.push_back(v0);
        m_vertices.push_back(v1);
        m_vertices.push_back(v2);
        m_vertices.push_back(v3);

        EdgeVInd e1 = {0, 1};
        EdgeVInd e2 = {1, 2};
        EdgeVInd e3 = {2, 3};
        EdgeVInd e4 = {3, 0};

        m_fixed_edges.insert({v0, v1});
        m_fixed_edges.insert({v1, v2});
        m_fixed_edges.insert({v2, v3});
        m_fixed_edges.insert({v3, v0});

        assert(triangulationIsConsistent());
    }

    //! \param np index of the neighbouring triangle
    //! \param tri
    //! \returns index in triangle of the vertex in tri opposite of triangle \p np
    template <class Vertex>
    int Triangulation<Vertex>::oppositeIndex(const TriInd np, const Triangle<Vertex> &tri)
    {
        if (np == tri.neighbours[0])
        {
            return 2;
        }
        else if (np == tri.neighbours[1])
        {
            return 0;
        }
        else if (np == tri.neighbours[2])
        {
            return 1;
        }
        else
        {
            throw std::runtime_error("triangles not neighbours!");
        }
    }

    //! \param tri  trian
    //! \param e    edge containing vertex coordinates it connects
    //! \returns index of triangle opposite of \p tri accross \p edge
    template <class Vertex>
    TriInd Triangulation<Vertex>::triangleOppositeOfEdge(const Triangle<Vertex> &tri, const EdgeI<Vertex> &e) const
    {
        const auto i2 = indexOf(e.from, tri);
        const auto i1 = indexOf(e.to(), tri);
        if (i1 == -1 or i2 == -1)
        {
            return -1;
        }
        assert(i1 != -1 && i2 != -1);
        return tri.neighbours[(oppositeOfEdge(tri, e) + 1) % 3];
    }

    //! \param e1 edge containg indices of vertices forming a first edge
    //! \param e2 edge containg indices of vertices forming a second edge
    //! \returns true if lines representing given edges intersect
    template <class Vertex>
    bool Triangulation<Vertex>::edgesIntersect(const EdgeVInd e1, const EdgeVInd e2) const noexcept
    {
        return segmentsIntersect(m_vertices[e1.from], m_vertices[e1.to], m_vertices[e2.from], m_vertices[e2.to]);
    }

    //! \param e1 edge containg vertices forming a first edge
    //! \param e2 edge containg vertices forming a second edge
    //! \returns true if lines representing given edges intersect
    template <class Vertex>
    bool Triangulation<Vertex>::edgesIntersect(const EdgeI<Vertex> e1, const EdgeI<Vertex> e2) const noexcept
    {
        return segmentsIntersect(e1.from, e1.to(), e2.from, e2.to());
    }

    //! \brief updates search grid used to find triangles
    //! \brief should be called whenever triagulation changes
    template <class Vertex>
    void Triangulation<Vertex>::updateCellGrid()
    {
        const auto dx = m_grid->m_cell_size.x;
        const auto dy = m_grid->m_cell_size.y;

        const auto n_cells_x = m_grid->m_cell_count.x;
        const auto n_cells_y = m_grid->m_cell_count.y;
        for (int j = 0; j < n_cells_y - 1; j++)
        { //! we walk zig-zag so that each next cell grid is close to the last one which helps findTriangle
            for (int i = 0; i < n_cells_x; i++)
            {
                const cdt::Vector2f r_center = {i * dx + dx / 2.f, j * dy + dy / 2.f};
                const auto cell_ind = j * n_cells_x + i;

                bool from_previous_one = true;
                m_cell2tri_ind.at(cell_ind) = findTriangle(r_center, from_previous_one);
            }

            j++;
            for (int i = n_cells_x - 1; i >= 0; i--)
            {
                const cdt::Vector2f r_center = {i * dx + dx / 2.f, j * dy + dy / 2.f};
                const auto cell_ind = j * n_cells_x + i;

                bool from_previous_one = true;
                m_cell2tri_ind.at(cell_ind) = findTriangle(r_center, from_previous_one);
            }
        }
        if (n_cells_y % 2 == 1)
        {
            int j = n_cells_y - 1;
            for (int i = n_cells_x - 1; i >= 0; i--)
            {
                const cdt::Vector2f r_center = {i * dx + dx / 2.f, j * dy + dy / 2.f};
                const auto cell_ind = j * n_cells_x + i;

                bool from_previous_one = true;
                m_cell2tri_ind.at(cell_ind) = findTriangle(r_center, from_previous_one);
            }
        }
    }

    //! \brief inserts vertex given we know that it lies directly on the given edge
    //! \param v_ind index of inserted vertex
    //! \param tri_ind_a index of counterclockwise triangle
    //! \param tri_ind_b index of a clockwise triangle
    //! \param edge
    template <class Vertex>
    void Triangulation<Vertex>::insertVertexOnEdge(const Vertex &new_vertex, TriInd tri_ind_a, TriInd tri_ind_b, const EdgeI<Vertex> &e)
    {
        const auto new_vertex_ind = m_vertices.size() - 1;

        const auto tri_ind_a_new = m_triangles.size();
        const auto tri_ind_b_new = m_triangles.size() + 1;

        auto tri_a = m_triangles.at(tri_ind_a);
        auto tri_b = m_triangles.at(tri_ind_b);

        const auto ind_in_tri_a = oppositeOfEdge(tri_a, e);
        const auto ind_in_tri_b = oppositeOfEdge(tri_b, e);

        Triangle<Vertex> tri_a_new = tri_a;
        Triangle<Vertex> tri_b_new = tri_b;

        m_tri_ind2vert_inds.push_back(m_tri_ind2vert_inds[tri_ind_a]);
        m_tri_ind2vert_inds.push_back(m_tri_ind2vert_inds[tri_ind_b]);

        for (int i = 0; i < 3; ++i)
        {
            tri_a_new.is_constrained[i] = false;
        }
        for (int i = 0; i < 3; ++i)
        {
            tri_b_new.is_constrained[i] = false;
        }

        //    tri_a_new.vertinds[(ind_in_tri_a + 1) % 3] = new_vertex_ind;
        m_tri_ind2vert_inds[tri_ind_a_new][(ind_in_tri_a + 1) % 3] = new_vertex_ind;
        tri_a_new.verts[(ind_in_tri_a + 1) % 3] = new_vertex;
        tri_a_new.neighbours[ind_in_tri_a] = tri_ind_a;
        tri_a_new.neighbours[(ind_in_tri_a + 1) % 3] = tri_ind_b;
        tri_a_new.is_constrained[(ind_in_tri_a + 1) % 3] = true;
        if (tri_a.is_constrained[(ind_in_tri_a + 2) % 3])
        {
            tri_a_new.is_constrained[(ind_in_tri_a + 2) % 3] = true;
        }
        //    tri_a.vertinds[(ind_in_tri_a + 2) % 3] = new_vertex_ind;
        m_tri_ind2vert_inds[tri_ind_a][(ind_in_tri_a + 2) % 3] = new_vertex_ind;
        tri_a.verts[(ind_in_tri_a + 2) % 3] = new_vertex;
        tri_a.neighbours[(ind_in_tri_a + 1) % 3] = tri_ind_b_new;
        tri_a.neighbours[(ind_in_tri_a + 2) % 3] = tri_ind_a_new;
        tri_a.is_constrained[(ind_in_tri_a + 2) % 3] = false;

        //    tri_b_new.vertinds[(ind_in_tri_b + 1) % 3] = new_vertex_ind;
        m_tri_ind2vert_inds[tri_ind_b_new][(ind_in_tri_b + 1) % 3] = new_vertex_ind;
        tri_b_new.verts[(ind_in_tri_b + 1) % 3] = new_vertex;
        tri_b_new.neighbours[ind_in_tri_b] = tri_ind_b;
        tri_b_new.neighbours[(ind_in_tri_b + 1) % 3] = tri_ind_a;
        tri_b_new.is_constrained[(ind_in_tri_b + 1) % 3] = true;
        if (tri_b.is_constrained[(ind_in_tri_b + 2) % 3])
        {
            tri_b_new.is_constrained[(ind_in_tri_b + 2) % 3] = true;
        }
        //    tri_b.vertinds[(ind_in_tri_b + 2) % 3] = new_vertex_ind;
        m_tri_ind2vert_inds[tri_ind_b][(ind_in_tri_b + 2) % 3] = new_vertex_ind;
        tri_b.verts[(ind_in_tri_b + 2) % 3] = new_vertex;
        tri_b.neighbours[(ind_in_tri_b + 1) % 3] = tri_ind_a_new;
        tri_b.neighbours[(ind_in_tri_b + 2) % 3] = tri_ind_b_new;
        tri_b.is_constrained[(ind_in_tri_b + 2) % 3] = false;

        //! we tell old triangles that they have a new neighbour;
        if (tri_a_new.neighbours[(ind_in_tri_a + 2) % 3] != -1)
        {
            auto &tri_next = m_triangles[tri_a_new.neighbours[(ind_in_tri_a + 2) % 3]];
            for (int i = 0; i < 3; ++i)
            {
                if (tri_next.neighbours[i] == tri_ind_a)
                {
                    tri_next.neighbours[i] = tri_ind_a_new;
                    break;
                }
            }
        }
        //! we tell old triangles that they have a new neighbour;
        if (tri_b_new.neighbours[(ind_in_tri_b + 2) % 3] != -1)
        {
            auto &tri_next = m_triangles[tri_b_new.neighbours[(ind_in_tri_b + 2) % 3]];
            for (int i = 0; i < 3; ++i)
            {
                if (tri_next.neighbours[i] == tri_ind_b)
                {
                    tri_next.neighbours[i] = tri_ind_b_new;
                    break;
                }
            }
        }

        m_triangles.push_back(tri_a_new);
        m_triangles.push_back(tri_b_new);
        m_triangles[tri_ind_a] = tri_a;
        m_triangles[tri_ind_b] = tri_b;
        m_fixed_edges.erase(e);
        m_fixed_edges.insert({e.from, new_vertex});
        m_fixed_edges.insert({new_vertex, e.to()});

        //! fix delaunay property
        std::stack<std::pair<TriInd, TriInd>> triangles_to_fix;
        if (tri_a_new.neighbours[(ind_in_tri_a + 2) % 3] != -1)
        {
            triangles_to_fix.push({tri_ind_a_new, tri_a_new.neighbours[(ind_in_tri_a + 2) % 3]});
        }
        if (tri_a.neighbours[ind_in_tri_a] != -1)
        {
            triangles_to_fix.push({tri_ind_a, tri_a.neighbours[ind_in_tri_a]});
        }

        if (tri_b_new.neighbours[(ind_in_tri_b + 2) % 3] != -1)
        {
            triangles_to_fix.push({tri_ind_b_new, tri_b_new.neighbours[(ind_in_tri_b + 2) % 3]});
        }
        if (tri_b.neighbours[ind_in_tri_b] != -1)
        {
            triangles_to_fix.push({tri_ind_b, tri_b.neighbours[ind_in_tri_b]});
        }

        while (!triangles_to_fix.empty())
        {
            auto next_tri_ind = triangles_to_fix.top().second;
            auto &next_tri = m_triangles[next_tri_ind];
            auto old_tri_ind = triangles_to_fix.top().first;
            auto &old_tri = m_triangles[old_tri_ind];
            triangles_to_fix.pop();

            auto opposite_ind_in_tri = oppositeIndex(old_tri_ind, next_tri);
            auto new_vert_ind_in_tri = indexOf(new_vertex, old_tri);
            auto v3 = next_tri.verts[opposite_ind_in_tri];
            auto v1 = next_tri.verts[(opposite_ind_in_tri + 1) % 3];
            auto v2 = next_tri.verts[(opposite_ind_in_tri + 2) % 3];
            auto vp = new_vertex;

            ;
            //        EdgeVInd flipped_edge = {next_tri.vertinds[(opposite_ind_in_tri + 1) % 3],
            //        next_tri.vertinds[(opposite_ind_in_tri + 2) % 3]};
            if (needSwap(vp, v1, v2, v3) && !next_tri.is_constrained[next(opposite_ind_in_tri)])
            {

                auto &a = old_tri.verts[(new_vert_ind_in_tri + 2) % 3];
                assert(isCounterClockwise(a, v3, vp));
                swapConnectingEdgeClockwise(old_tri_ind, next_tri_ind);

                if (old_tri.neighbours[(new_vert_ind_in_tri + 1) % 3] != -1)
                {
                    triangles_to_fix.emplace(old_tri_ind, old_tri.neighbours[(new_vert_ind_in_tri + 1) % 3]);
                }
                if (next_tri.neighbours[(opposite_ind_in_tri + 2) % 3] != -1)
                {
                    triangles_to_fix.emplace(next_tri_ind, next_tri.neighbours[(opposite_ind_in_tri + 2) % 3]);
                }
            }
        }
    }

    //! \param new_vertex
    //! \param tri_ind triangle index of an existing triangle containing new_vertex
    //! \returns index of the overlapping vertex
    //! \returns -1 in case there is no existing overlapping vertex
    template <class Vertex>
    VertInd Triangulation<Vertex>::findOverlappingVertex(const Vertex &new_vertex, const TriInd tri_ind) const
    {
        assert(tri_ind != -1);
        const auto old_triangle = m_triangles[tri_ind];
        const auto v0 = old_triangle.verts[0];
        const auto v1 = old_triangle.verts[1];
        const auto v2 = old_triangle.verts[2];
        if (v0 == new_vertex)
        {
            return m_tri_ind2vert_inds[tri_ind][0];
        }
        if (v1 == new_vertex)
        {
            return m_tri_ind2vert_inds[tri_ind][1];
        }
        if (v2 == new_vertex)
        {
            return m_tri_ind2vert_inds[tri_ind][2];
        }
        return -1;
    }

    //! \brief checks whether \p new_vertex lies on some existing edge
    //! \param new_vertex
    //! \param tri_ind triangle index of an existing triangle containing new_vertex
    //! \returns overlapping edge
    //! \returns {-1, -1} in case there is no overlapping edge
    template <class Vertex>
    EdgeVInd Triangulation<Vertex>::findOverlappingEdge(const Vertex &new_vertex, const TriInd tri_ind) const
    {
        const auto &old_triangle = m_triangles[tri_ind];
        const auto v0 = old_triangle.verts[0];
        const auto v1 = old_triangle.verts[1];
        const auto v2 = old_triangle.verts[2];

        const auto &vert_inds = m_tri_ind2vert_inds[tri_ind];

        cdt::Vector2i edge_normal0 = {-(v1.y - v0.y), v1.x - v0.x};
        cdt::Vector2i edge_normal1 = {-(v2.y - v1.y), v2.x - v1.x};
        cdt::Vector2i edge_normal2 = {-(v0.y - v2.y), v0.x - v2.x};
        EdgeI edge0 = {old_triangle.verts[0], old_triangle.verts[1]};
        EdgeI edge1 = {old_triangle.verts[1], old_triangle.verts[2]};
        EdgeI edge2 = {old_triangle.verts[2], old_triangle.verts[0]};
        if (old_triangle.is_constrained[0] && dot(new_vertex - v0, edge_normal0) == 0)
        {
            return {vert_inds[0], vert_inds[1]};
        }
        else if (old_triangle.is_constrained[1] && dot(new_vertex - v1, edge_normal1) == 0)
        {
            return {vert_inds[1], vert_inds[2]};
        }
        else if (old_triangle.is_constrained[2] && dot(new_vertex - v2, edge_normal2) == 0)
        {
            return {vert_inds[2], vert_inds[0]};
            ;
        }
        return EdgeVInd();
    }

    //! \brief inserts \p new_vertex into triangulation
    //! \param new_vertex
    //! \param search_from_last_one should be true if last added vertex is not far from \p new_vertex
    template <class Vertex>
    void Triangulation<Vertex>::insertVertex(const Vertex &new_vertex, bool search_from_last_one)
    {
        auto data = insertVertexAndGetData(new_vertex, search_from_last_one);
        assert(allTrianglesValid());
    }

    //! \brief inserts \p new_vertex into triangulation, the inserted vertex can either:
    //! \brief already exist, or it may lie on an existing edge or it lies in free space
    //! \param new_vertex
    //! \param search_from_last_one should be true if last added vertex is not far from \p new_vertex
    //! \returns data relating to the actual type of insertion performed
    template <class Vertex>
    VertexInsertionData Triangulation<Vertex>::insertVertexAndGetData(int vx, int vy, bool search_from_last_one)
    {
        return insertVertexAndGetData({vx, vy}, search_from_last_one);
    }

    //! \brief inserts all \p verts into the triangulation
    //!
    template <class Vertex>
    void Triangulation<Vertex>::insertVertices(const std::vector<Vertex> &verts)
    {
        int vert_ind = m_vertices.size();
        std::vector<std::vector<Vertex>> m_grid2vert_inds(m_grid->getNCells());

        //! sort vertices into bins formed by grid cells
        for (auto &v : verts)
        {
            m_grid2vert_inds.at(m_grid->cellIndex(v)).push_back(v);
            vert_ind++;
        }

        auto cells_x = m_grid->m_cell_count.x;
        for (int iy = 0; iy < m_grid->m_cell_count.y; iy++)
        {
            bool odd_line = iy % 2 == 1;
            for (int ix = 0; ix < m_grid->m_cell_count.x; ix++)
            {

                int ix_walk = odd_line * (cells_x - 1) + (1 - 2 * odd_line) * ix;
                auto cell_ind = m_grid->cellIndex(ix_walk, iy);
                for (auto &v : m_grid2vert_inds.at(cell_ind))
                {
                    auto data = insertVertexAndGetData(v, true);
                }
            }
        }
    }

    //! \brief inserts \p new_vertex into triangulation, the inserted vertex can either:
    //! \brief already exist, or it may lie on an existing edge or it lies in free space
    //! \param new_vertex
    //! \param search_from_last_one should be true if last added vertex is not far from \p new_vertex
    //! \returns data relating to the actual type of insertion performed
    template <class Vertex>
    VertexInsertionData Triangulation<Vertex>::insertVertexAndGetData(const Vertex &new_vertex, bool search_from_last_one)
    {
        VertexInsertionData data;
        
        if(!withinBoundary(new_vertex))
        {
            return data;
        }

        auto tri_ind = findTriangle(new_vertex, search_from_last_one);
        const auto old_triangle = m_triangles[tri_ind];

        data.overlapping_vertex = findOverlappingVertex(new_vertex, tri_ind);
        if (data.overlapping_vertex != -1)
        {
            return data;
        }

        const auto new_vertex_ind = m_vertices.size();
        m_vertices.push_back(new_vertex);

        const auto overlapping_edge = findOverlappingEdge(new_vertex, tri_ind);
        if (overlapping_edge.from != -1)
        {
            const EdgeI edge_i = {m_vertices[overlapping_edge.from], m_vertices[overlapping_edge.to]};
            insertVertexOnEdge(new_vertex, tri_ind, triangleOppositeOfEdge(old_triangle, edge_i), edge_i);
            data.overlapping_edge = overlapping_edge;
            return data;
        }

        insertVertexIntoSpace(new_vertex, tri_ind, new_vertex_ind);
        return data;
    }

    //! \brief inserts \p new_vertex into triangulation knowing it lies in free space (not on edge or vertex)
    //! \param new_vertex
    //! \param search_from_last_one should be true if last added vertex is not far from \p new_vertex
    //! \returns data relating to the actual type of insertion performed
    template <class Vertex>
    void Triangulation<Vertex>::insertVertexIntoSpace(const Vertex &new_vertex, TriInd tri_ind, VertInd new_vertex_ind)
    {

        const auto old_triangle = m_triangles[tri_ind];

        const auto first_new_triangle_ind = tri_ind;
        const auto second_new_triangle_ind = m_triangles.size();
        const auto third_new_triangle_ind = m_triangles.size() + 1;

        Triangle t1_new = old_triangle;
        Triangle t2_new = old_triangle;
        Triangle t3_new = old_triangle;

        m_tri_ind2vert_inds.push_back(m_tri_ind2vert_inds[tri_ind]);
        m_tri_ind2vert_inds.push_back(m_tri_ind2vert_inds[tri_ind]);

        // assert(hasGoodOrientation(old_triangle));

        for (int i = 0; i < 3; ++i)
        {
            t1_new.is_constrained[i] = false;
        }
        for (int i = 0; i < 3; ++i)
        {
            t2_new.is_constrained[i] = false;
        }
        for (int i = 0; i < 3; ++i)
        {
            t3_new.is_constrained[i] = false;
        }
        //    m_tri_ind2vert_inds[first_new_triangle_ind] = new_vertex_ind;
        m_tri_ind2vert_inds[first_new_triangle_ind][2] = new_vertex_ind;
        t1_new.verts[2] = new_vertex;
        t1_new.neighbours[2] = third_new_triangle_ind;
        t1_new.neighbours[1] = second_new_triangle_ind;
        if (old_triangle.is_constrained[0])
        {
            t1_new.is_constrained[0] = true;
        }

        m_triangles[tri_ind] = t1_new;

        //    t2_new.vertinds[0] =  new_vertex_ind;
        m_tri_ind2vert_inds[second_new_triangle_ind][0] = new_vertex_ind;
        t2_new.verts[0] = new_vertex;
        t2_new.neighbours[0] = first_new_triangle_ind;
        t2_new.neighbours[2] = third_new_triangle_ind;
        if (old_triangle.is_constrained[1])
        {
            t2_new.is_constrained[1] = true;
        }

        //    t3_new.vertinds[1] = new_vertex_ind;
        m_tri_ind2vert_inds[third_new_triangle_ind][1] = new_vertex_ind;
        t3_new.verts[1] = new_vertex;
        t3_new.neighbours[1] = second_new_triangle_ind;
        t3_new.neighbours[0] = first_new_triangle_ind;
        if (old_triangle.is_constrained[2])
        {
            t3_new.is_constrained[2] = true;
        }

        //! fix delaunay property
        std::stack<std::pair<TriInd, TriInd>> triangles_to_fix;
        if (old_triangle.neighbours[0] != -1)
        {
            triangles_to_fix.push({first_new_triangle_ind, old_triangle.neighbours[0]});
        }
        if (old_triangle.neighbours[1] != -1)
        {
            triangles_to_fix.push({second_new_triangle_ind, old_triangle.neighbours[1]});
        }
        if (old_triangle.neighbours[2] != -1)
        {
            triangles_to_fix.push({third_new_triangle_ind, old_triangle.neighbours[2]});
        }

        //! we tell old triangles that they have a new neighbour;
        if (old_triangle.neighbours[1] != -1)
        {
            auto neighbour = old_triangle.neighbours[1];
            auto &tri_next = m_triangles[neighbour];
            for (int i = 0; i < 3; ++i)
            {
                if (tri_next.neighbours[i] == tri_ind)
                {
                    tri_next.neighbours[i] = second_new_triangle_ind;
                    break;
                }
            }
        }
        if (old_triangle.neighbours[2] != -1)
        {
            auto neighbour = old_triangle.neighbours[2];
            auto &tri_next = m_triangles[neighbour];
            for (int i = 0; i < 3; ++i)
            {
                if (tri_next.neighbours[i] == tri_ind)
                {
                    tri_next.neighbours[i] = third_new_triangle_ind;
                    break;
                }
            }
        }

        //        removeTriangle(tri_ind);

        //        m_triangles.push_back(t1_new);
        m_triangles.push_back(t2_new);
        m_triangles.push_back(t3_new);

        while (!triangles_to_fix.empty())
        {
            auto next_tri_ind = triangles_to_fix.top().second;
            auto &next_tri = m_triangles[next_tri_ind];
            auto old_tri_ind = triangles_to_fix.top().first;
            auto &old_tri = m_triangles[old_tri_ind];
            triangles_to_fix.pop();

            auto opposite_ind_in_tri = oppositeIndex(old_tri_ind, next_tri);
            auto newvert_ind_in_tri = indexOf(new_vertex, old_tri);
            auto v3 = next_tri.verts[opposite_ind_in_tri];
            auto v1a = next_tri.verts[(opposite_ind_in_tri + 1) % 3];
            auto v2a = next_tri.verts[(opposite_ind_in_tri + 2) % 3];
            auto vp = new_vertex;

            //        EdgeVInd flipped_edge = {next_tri.vertinds[(opposite_ind_in_tri + 1) % 3],
            //        next_tri.vertinds[(opposite_ind_in_tri + 2) % 3]};

            if (needSwap(vp, v1a, v2a, v3) && !next_tri.is_constrained[next(opposite_ind_in_tri)])
            {

                auto &a = old_tri.verts[(newvert_ind_in_tri + 2) % 3];
                assert(isCounterClockwise(a, v3, vp));
                swapConnectingEdgeCounterClockwise(old_tri_ind, next_tri_ind);

                if (old_tri.neighbours[(newvert_ind_in_tri + 1) % 3] != -1)
                {
                    triangles_to_fix.emplace(old_tri_ind, old_tri.neighbours[(newvert_ind_in_tri + 1) % 3]);
                }
                if (next_tri.neighbours[(opposite_ind_in_tri + 2) % 3] != -1)
                {
                    triangles_to_fix.emplace(next_tri_ind, next_tri.neighbours[(opposite_ind_in_tri + 2) % 3]);
                }
            }
        }
    }

    //! \returns true if quadrilateral formed by the giver four vertices is convex
    template <class Vertex>
    bool Triangulation<Vertex>::isConvex(const Vertex v1, const Vertex v2, const Vertex v3,
                                         const Vertex v4) const
    { //! v1-v3 and v2-v4 are diagonals
        return segmentsIntersect(v1, v3, v2, v4);
    }

    //! \param edge represented by vertex coordinates
    //! \param tri
    //! \returns index in triangle of the vertex in \p tri which is opposite of the \p edge
    template <class Vertex>
    int Triangulation<Vertex>::oppositeOfEdge(const Triangle<Vertex> &tri, const EdgeI<Vertex> &e) const
    {
        const auto i1 = indexOf(e.from, tri);
        const auto i2 = indexOf(e.to(), tri);
        return 3 - (i1 + i2); //! i1 + i2 has values 1,2,3.. corresponding output should be 2,1,0
    }

    //! \brief checks if neighbours of each triangles are consistent and
    //! \brief if m_tri_ind2vert_inds points to right place in m_vertices array
    template <class Vertex>
    bool Triangulation<Vertex>::triangulationIsConsistent() const
    {
        for (int tri_ind = 0; tri_ind < m_triangles.size(); ++tri_ind)
        {
            const auto &tri = m_triangles[tri_ind];
            for (int k = 0; k < 3; ++k)
            {
                const auto neighbour_tri_ind = tri.neighbours[k];
                if (neighbour_tri_ind != -1)
                {
                    const auto &neighbour_tri = m_triangles[neighbour_tri_ind];
                    const auto ind_in_neirhbour_tri = 0;
                    const auto found_at =
                        std::find(neighbour_tri.neighbours.begin(), neighbour_tri.neighbours.end(), tri_ind) -
                        neighbour_tri.neighbours.begin();
                    if (found_at < 0 or found_at > 2)
                    {
                        return false;
                    }
                    if (tri.is_constrained[k] xor neighbour_tri.is_constrained[found_at])
                    {
                        return false;
                    }
                }
                const auto v = tri.verts[k];
                if (v != m_vertices[m_tri_ind2vert_inds[tri_ind][k]])
                {
                    return false;
                }
            }
        }
        return true;
    }

    //! \brief forces triangulation to have a constrained edge connecting \p e.from and \p e.to
    //! \param e edge representing the constraint
    template <class Vertex>
    void Triangulation<Vertex>::insertConstraint(const EdgeVInd e)
    {

        auto vi_ind = e.from;
        auto vj_ind = e.to;
        auto vi = m_vertices[vi_ind];
        auto vj = m_vertices[vj_ind];
        if (m_fixed_edges.count({vi, vj}) > 0 || e.from == e.to) //! constrained edge alread exists
        {
            return;
        }
        m_fixed_edges.insert({vi, vj});

        std::deque<EdgeI<Vertex>> intersected_edges;
        std::deque<TriInd> intersected_tri_inds;
        findIntersectingEdges(e, intersected_edges, intersected_tri_inds);

        auto overlapsx = findOverlappingConstraints(vi, vj);
        auto overlaps = findOverlappingConstraints2(vi, vj);
        if (!overlaps.empty())
        {
            for (const auto &overlap : overlaps)
            {
                insertConstraint(overlap);
            }
            insertConstraint({e.from, overlaps[0].from});
            for (int i = 1; i < overlaps.size(); ++i)
            {
                insertConstraint({overlaps.at(i - 1).to, overlaps.at(i).from});
            }
            insertConstraint({e.to, overlaps.back().to});
            return;
        }

        std::vector<EdgeI<Vertex>> newly_created_edges;
        std::vector<std::pair<TriInd, TriInd>> newly_created_edge_tris;

        EdgeI e_inserted = {vi, vj};

        //! remove intersecting edges
        while (!intersected_edges.empty())
        {
            auto tri_ind = intersected_tri_inds.front();
            auto &tri = m_triangles[tri_ind];
            intersected_tri_inds.pop_front();
            auto e_next = intersected_edges.front();
            intersected_edges.pop_front();

            auto next_tri_ind = triangleOppositeOfEdge(tri, e_next);
            auto &next_tri = m_triangles[next_tri_ind];

            auto v_current_ind = m_tri_ind2vert_inds[tri_ind][oppositeOfEdge(tri, e_next)];
            auto v_current = tri.verts[oppositeOfEdge(tri, e_next)];
            auto v_opposite_ind_in_tri = oppositeIndex(tri_ind, next_tri);
            auto v_opposite_current = next_tri.verts[v_opposite_ind_in_tri];

            //! we can swap edges only in convex quadrilaterals otherwise bad shapes get created
            if (isConvex(v_current, e_next.from, v_opposite_current, e_next.to()))
            {
                if (isCounterClockwise(v_opposite_current, vi, vj))
                {
                    swapConnectingEdgeCounterClockwise(next_tri_ind, tri_ind);
                    assert(triangulationIsConsistent());
                }
                else
                {
                    swapConnectingEdgeClockwise(tri_ind, next_tri_ind);
                    assert(triangulationIsConsistent());
                }
                assert(allTrianglesValid());

                e_next = {v_current, v_opposite_current};

                if (edgesIntersect(e_next, e_inserted))
                {
                    intersected_edges.push_back(e_next);
                    intersected_tri_inds.push_back(tri_ind);
                    auto ind_in_tri = oppositeOfEdge(m_triangles.at(tri_ind), e_next);
                }
                else
                {
                    newly_created_edges.push_back(e_next);
                    newly_created_edge_tris.push_back({tri_ind, next_tri_ind});
                }
            }
            else
            {
                intersected_edges.push_back(e_next);
                intersected_tri_inds.push_back(tri_ind);
            }
        }

        //! Fix Delaunay triangulation (Steps 4.1 - 4.3)
        bool some_swap_happened = true;
        while (some_swap_happened)
        {
            some_swap_happened = false;

            for (int i = 0; i < newly_created_edges.size(); ++i)
            {
                const auto &e_new = newly_created_edges[i];
                const auto tri_ind_a = newly_created_edge_tris[i].first;
                auto &tri_a = m_triangles[tri_ind_a];
                auto tri_ind_b = triangleOppositeOfEdge(tri_a, e_new);
                if (tri_ind_b == -1)
                {
                    tri_ind_b = newly_created_edge_tris[i].second;
                }
                auto &tri_b = m_triangles[tri_ind_b];

                const auto opposite_ind_in_tri_a = oppositeOfEdge(tri_a, e_new);
                const auto opposite_ind_in_tri_b = oppositeOfEdge(tri_b, e_new);

                vi = tri_a.verts[opposite_ind_in_tri_a];
                vj = tri_b.verts[opposite_ind_in_tri_b];
                vi_ind = m_tri_ind2vert_inds[tri_ind_a][opposite_ind_in_tri_a];
                auto vi_ind_next = m_tri_ind2vert_inds[tri_ind_a][next(opposite_ind_in_tri_a)];
                auto vi_ind_prev = m_tri_ind2vert_inds[tri_ind_a][prev(opposite_ind_in_tri_a)];

                if (e_new == e_inserted)
                {
                    tri_a.is_constrained[next(opposite_ind_in_tri_a)] = true;
                    tri_b.is_constrained[next(opposite_ind_in_tri_b)] = true;
                    continue;
                }

                const auto v1 = m_vertices.at(vi_ind_next);
                const auto v2 = m_vertices.at(vi_ind_prev);

                bool edge_needs_swap = needSwap(vi, v1, v2, vj);
                bool is_convex = true; //! Convexity check should be automatically taken care of by needSwap but it doesn't
                                       //! and I don't know why yet :(
                if (!isConvex(m_vertices[vi_ind], v1, v2, m_vertices[vj_ind]))
                {
                    is_convex = false;
                }
                if (edge_needs_swap && is_convex)
                {

                    swapConnectingEdgeClockwise(tri_ind_a, tri_ind_b);
                    assert(allTrianglesValid());

                    some_swap_happened = true;
                    newly_created_edges[i] = {vi, vj};
                }
            }
        }
    }

    //! \brief forces triangulation to have a constrained edge connecting \p e.from and \p e.to
    //! \param e edge representing the constraint
    template <class Vertex>
    void Triangulation<Vertex>::insertConstraint(const EdgeVInd e, sf::RenderWindow &window)
    {

        auto vi_ind = e.from;
        auto vj_ind = e.to;
        auto vi = m_vertices[vi_ind];
        auto vj = m_vertices[vj_ind];
        if (m_fixed_edges.count({vi, vj}) > 0 || e.from == e.to) //! constrained edge alread exists
        {
            return;
        }
        m_fixed_edges.insert({vi, vj});

        auto overlapsx = findOverlappingConstraints(vi, vj);
        auto overlaps = findOverlappingConstraints2(vi, vj);
        if (!overlaps.empty())
        {
            for (auto overlap : overlaps)
            {
                if (overlap != e)
                {
                    insertConstraint(overlap);
                }
            }
            //! inserts constraints between overlapping edges to fill the empty space
            insertConstraint({e.from, overlaps[0].from});
            for (int i = 1; i < overlaps.size(); ++i)
            {
                insertConstraint({overlaps.at(i - 1).to, overlaps.at(i).from});
            }
            insertConstraint({e.to, overlaps.back().to});
            return;
        }

        std::deque<EdgeI<Vertex>> intersected_edges;
        std::deque<TriInd> intersected_tri_inds;
        findIntersectingEdges(e, intersected_edges, intersected_tri_inds);

        std::vector<EdgeI<Vertex>> newly_created_edges;
        std::vector<std::pair<TriInd, TriInd>> newly_created_edge_tris;

        EdgeI e_inserted = {vi, vj};

        m_font.loadFromFile("../Resources/arial.ttf");

        //! remove intersecting edges (steps 3.1 3.2)
        while (!intersected_edges.empty())
        {

            auto tri_ind = intersected_tri_inds.front();
            auto &tri = m_triangles[tri_ind];
            intersected_tri_inds.pop_front();
            auto e_next = intersected_edges.front();
            intersected_edges.pop_front();

            auto next_tri_ind = triangleOppositeOfEdge(tri, e_next);
            auto &next_tri = m_triangles[next_tri_ind];

            assert(next_tri_ind != -1); //! triangle must contain e_next;

            // auto v_current_ind = m_tri_ind2vert_inds[tri_ind][oppositeOfEdge(tri, e_next)];
            auto v_current = tri.verts[oppositeOfEdge(tri, e_next)];
            auto v_opposite_ind_in_tri = oppositeIndex(tri_ind, next_tri);
            auto v_opposite_current = next_tri.verts[v_opposite_ind_in_tri];

            //! we can swap edges only in convex quadrilaterals otherwise bad shapes get created
            if (isConvex(v_current, e_next.from, v_opposite_current, e_next.to()))
            {
                if (isCounterClockwise(v_opposite_current, vi, vj))
                {
                    swapConnectingEdgeCounterClockwise(next_tri_ind, tri_ind);
                }
                else
                {
                    swapConnectingEdgeClockwise(next_tri_ind, tri_ind);
                }
                drawTriangulation(*this, window);
                drawTriInds(*this, window);
                window.display();

                assert(allTrianglesValid());
                assert(triangulationIsConsistent());

                e_next = {v_current, v_opposite_current};

                if (edgesIntersect(e_next, e_inserted))
                {
                    intersected_edges.push_back(e_next);
                    intersected_tri_inds.push_back(tri_ind);
                    auto ind_in_tri = oppositeOfEdge(m_triangles.at(tri_ind), e_next);
                }
                else
                {
                    newly_created_edges.push_back(e_next);
                    newly_created_edge_tris.push_back({tri_ind, next_tri_ind});
                }
            }
            else
            {
                intersected_edges.push_back(e_next);
                intersected_tri_inds.push_back(tri_ind);
            }
        }

        m_fixed_edges.insert(e_inserted);

        //! Fix Delaunay triangulation (Steps 4.1 - 4.3)
        bool some_swap_happened = true;
        while (some_swap_happened)
        {
            some_swap_happened = false;

            for (int i = 0; i < newly_created_edges.size(); ++i)
            {
                const auto &e_new = newly_created_edges[i];
                const auto tri_ind_a = newly_created_edge_tris[i].first;
                auto &tri_a = m_triangles[tri_ind_a];
                auto tri_ind_b = triangleOppositeOfEdge(tri_a, e_new);
                if (tri_ind_b == -1)
                {
                    tri_ind_b = newly_created_edge_tris[i].second;
                }
                auto &tri_b = m_triangles[tri_ind_b];

                const auto opposite_ind_in_tri_a = oppositeOfEdge(tri_a, e_new);
                const auto opposite_ind_in_tri_b = oppositeOfEdge(tri_b, e_new);

                vi = tri_a.verts[opposite_ind_in_tri_a];
                vj = tri_b.verts[opposite_ind_in_tri_b];
                vi_ind = m_tri_ind2vert_inds[tri_ind_a][opposite_ind_in_tri_a];
                auto vi_ind_next = m_tri_ind2vert_inds[tri_ind_a][next(opposite_ind_in_tri_a)];
                auto vi_ind_prev = m_tri_ind2vert_inds[tri_ind_a][prev(opposite_ind_in_tri_a)];

                if (e_new == e_inserted)
                {
                    tri_a.is_constrained[next(opposite_ind_in_tri_a)] = true;
                    tri_b.is_constrained[next(opposite_ind_in_tri_b)] = true;
                    continue;
                }

                const auto v1 = m_vertices.at(vi_ind_next);
                const auto v2 = m_vertices.at(vi_ind_prev);

                bool edge_needs_swap = needSwap(vi, v1, v2, vj);
                bool is_convex = true; //! Convexity check should be automatically taken care of by needSwap but it doesn't
                                       //! and I don't know why yet :(
                if (!isConvex(m_vertices[vi_ind], v1, v2, m_vertices[vj_ind]))
                {
                    is_convex = false;
                }
                if (edge_needs_swap && is_convex)
                {

                    swapConnectingEdgeClockwise(tri_ind_a, tri_ind_b);
                    assert(allTrianglesValid());

                    some_swap_happened = true;
                    newly_created_edges[i] = {vi, vj};
                }
            }
        }
    }

    //! \brief swaps edge connecting \p tri_ind_a with tri_ind_b such that they move in a clockwise manner
    //! \param tri_ind_a index of a triangle
    //! \param tri_ind_a index of another triangle
    template <class Vertex>
    void Triangulation<Vertex>::swapConnectingEdgeClockwise(const TriInd &tri_ind_a, const TriInd &tri_ind_b)
    {
        auto &tri_a = m_triangles[tri_ind_a];
        auto &tri_b = m_triangles[tri_ind_b];

        auto v_a_ind_in_tri = oppositeIndex(tri_ind_b, tri_a);
        auto v_b_ind_in_tri = oppositeIndex(tri_ind_a, tri_b);

        //! change vertices -> prev(a) becomes b and prev(b) becomes a;
        const auto &v_b = tri_b.verts[v_b_ind_in_tri];
        const auto &v_a = tri_a.verts[v_a_ind_in_tri];
        const auto &v_left = tri_a.verts[prev(v_a_ind_in_tri)];
        const auto &v_right = tri_a.verts[next(v_a_ind_in_tri)];
        tri_a.verts[next(v_a_ind_in_tri)] = v_b;
        tri_b.verts[next(v_b_ind_in_tri)] = v_a;

        //! change neighbours
        const auto na = tri_a.neighbours[(v_a_ind_in_tri)];
        const auto nb = tri_b.neighbours[(v_b_ind_in_tri)];
        const auto na_prev = tri_a.neighbours[prev(v_a_ind_in_tri)];
        const auto nb_prev = tri_b.neighbours[prev(v_b_ind_in_tri)];
        tri_a.neighbours[next(v_a_ind_in_tri)] = nb;
        tri_a.neighbours[(v_a_ind_in_tri)] = tri_ind_b;
        tri_b.neighbours[next(v_b_ind_in_tri)] = na;
        tri_b.neighbours[(v_b_ind_in_tri)] = tri_ind_a;
        //! change constraints
        tri_a.is_constrained[next(v_a_ind_in_tri)] = tri_b.is_constrained[(v_b_ind_in_tri)];
        tri_b.is_constrained[next(v_b_ind_in_tri)] = tri_a.is_constrained[(v_a_ind_in_tri)];
        tri_a.is_constrained[(v_a_ind_in_tri)] = false;
        tri_b.is_constrained[(v_b_ind_in_tri)] = false;

        m_tri_ind2vert_inds.at(tri_ind_a)[next(v_a_ind_in_tri)] = m_tri_ind2vert_inds.at(tri_ind_b)[v_b_ind_in_tri];
        m_tri_ind2vert_inds.at(tri_ind_b)[next(v_b_ind_in_tri)] = m_tri_ind2vert_inds.at(tri_ind_a)[v_a_ind_in_tri];

        //! tell neighbours that there was a swap changed
        if (nb != -1)
        {
            auto &changed_neighbour_tri = m_triangles[nb];
            auto ind_in_neighbour = oppositeIndex(tri_ind_b, changed_neighbour_tri);
            changed_neighbour_tri.neighbours[next(ind_in_neighbour)] = tri_ind_a;
        }
        if (na != -1)
        {
            auto &changed_neighbour_tri = m_triangles[na];
            auto ind_in_neighbour = oppositeIndex(tri_ind_a, changed_neighbour_tri);
            changed_neighbour_tri.neighbours[next(ind_in_neighbour)] = tri_ind_b;
        }
    }

    //! \brief swaps edge connecting \p tri_ind_a with tri_ind_b such that they move in a counter-clockwise manner
    //! \param tri_ind_a index of a triangle
    //! \param tri_ind_a index of another triangle
    template <class Vertex>
    void Triangulation<Vertex>::swapConnectingEdgeCounterClockwise(const TriInd &tri_ind_a, const TriInd &tri_ind_b)
    {
        auto &tri_a = m_triangles[tri_ind_a];
        auto &tri_b = m_triangles[tri_ind_b];

        auto v_a_ind_in_tri = oppositeIndex(tri_ind_b, tri_a);
        auto v_b_ind_in_tri = oppositeIndex(tri_ind_a, tri_b);

        //! change vertices -> prev(a) becomes b and prev(b) becomes a;
        const auto &v_b = tri_b.verts[v_b_ind_in_tri];
        const auto &v_a = tri_a.verts[v_a_ind_in_tri];
        const auto &v_left = tri_a.verts[prev(v_a_ind_in_tri)];
        const auto &v_right = tri_a.verts[next(v_a_ind_in_tri)];

        tri_a.verts[prev(v_a_ind_in_tri)] = v_b;
        tri_b.verts[prev(v_b_ind_in_tri)] = v_a;

        //! change neighbours
        const auto na_prev = tri_a.neighbours[prev(v_a_ind_in_tri)];
        const auto nb_prev = tri_b.neighbours[prev(v_b_ind_in_tri)];
        tri_a.neighbours[next(v_a_ind_in_tri)] = nb_prev;
        tri_a.neighbours[prev(v_a_ind_in_tri)] = tri_ind_b;
        tri_b.neighbours[next(v_b_ind_in_tri)] = na_prev;
        tri_b.neighbours[prev(v_b_ind_in_tri)] = tri_ind_a;
        //! change constraints
        tri_a.is_constrained[next(v_a_ind_in_tri)] = tri_b.is_constrained[prev(v_b_ind_in_tri)];
        tri_b.is_constrained[next(v_b_ind_in_tri)] = tri_a.is_constrained[prev(v_a_ind_in_tri)];
        tri_a.is_constrained[prev(v_a_ind_in_tri)] = false;
        tri_b.is_constrained[prev(v_b_ind_in_tri)] = false;

        m_tri_ind2vert_inds.at(tri_ind_a)[prev(v_a_ind_in_tri)] = m_tri_ind2vert_inds.at(tri_ind_b)[v_b_ind_in_tri];
        m_tri_ind2vert_inds.at(tri_ind_b)[prev(v_b_ind_in_tri)] = m_tri_ind2vert_inds.at(tri_ind_a)[v_a_ind_in_tri];

        //! tell neighbours that there was a swap changed
        if (nb_prev != -1)
        {
            auto &changed_neighbour_tri = m_triangles[nb_prev];
            auto ind_in_neighbour = oppositeIndex(tri_ind_b, changed_neighbour_tri);
            changed_neighbour_tri.neighbours[next(ind_in_neighbour)] = tri_ind_a;
        }
        if (na_prev != -1)
        {
            auto &changed_neighbour_tri = m_triangles[na_prev];
            auto ind_in_neighbour = oppositeIndex(tri_ind_a, changed_neighbour_tri);
            changed_neighbour_tri.neighbours[next(ind_in_neighbour)] = tri_ind_b;
        }
    }

    template <class Vertex>
    std::vector<EdgeI<Vertex>> Triangulation<Vertex>::findOverlappingConstraints(const Vertex &vi, const Vertex &vj)
    {

        //! walk from tri_ind_start to  tri_ind_end while looking for collinear constrained edges
        const auto start_tri_ind = findTriangle(vi, false);
        const auto start_tri = m_triangles[start_tri_ind];
        const auto end_tri_ind = findTriangle(vj, true);
        const auto end_tri = m_triangles[end_tri_ind];

        auto tri_ind = start_tri_ind;
        auto tri = m_triangles[tri_ind];
        auto index_in_tri = indexOf(vi, tri);

        auto v_left = tri.verts[prev(index_in_tri)];
        auto v_right = tri.verts[next(index_in_tri)];

        std::vector<EdgeI<Vertex>> overlapps;

        bool prev_touched = false;

        // check if the vj is already connected to vi
        do
        {
            if (liesBetween(v_right, vi, vj))
            {
                if (tri.is_constrained[index_in_tri])
                {
                    overlapps.push_back({vi, v_right});
                }
                break;
            }
            if (segmentsIntersect(v_left, v_right, vi, vj))
            {
                tri_ind = triangleOppositeOfEdge(tri, {v_left, v_right});
                break;
            }
            tri_ind = tri.neighbours[index_in_tri];
            tri = m_triangles[tri_ind];
            index_in_tri = indexOf(vi, tri);
            v_left = v_right;
            v_right = tri.verts[next(index_in_tri)];
        } while (tri_ind != start_tri_ind);

        auto v_current = v_right;
        auto prev_tri_ind = tri_ind;
        tri_ind = tri.neighbours[next(index_in_tri)];

        auto opp_vertex = v_right;
        auto prev_opp_vertex = opp_vertex;

        while (v_current != vj)
        {
            auto &tri = m_triangles.at(tri_ind);
            index_in_tri = indexOf(v_current, tri);
            opp_vertex = tri.verts[next(index_in_tri)];

            if (liesBetween(v_current, vi, vj) && liesBetween(opp_vertex, vi, vj) && tri.is_constrained[index_in_tri])
            {
                overlapps.push_back({v_current, opp_vertex});
            }

            //!
            if (orient(vi, vj, opp_vertex) > 0)
            {
                v_current = tri.verts[next(index_in_tri)];
                tri_ind = tri.neighbours[next(index_in_tri)];
                prev_touched = false;
            }
            else if (strictly_less(orient(vi, vj, opp_vertex), 0))
            {
                tri_ind = tri.neighbours[index_in_tri];
            }
            else
            {
                tri_ind = tri.neighbours[next(index_in_tri)];
                v_current = tri.verts[next(index_in_tri)];
            }
        }
        return overlapps;
    }

    template <class Vertex>
    std::vector<EdgeVInd> Triangulation<Vertex>::findOverlappingConstraints2(const Vertex &vi, const Vertex &vj)
    {

        //! walk from tri_ind_start to  tri_ind_end while looking for collinear constrained edges
        const auto start_tri_ind = findTriangle(vi, false);
        const auto start_tri = m_triangles[start_tri_ind];
        const auto end_tri_ind = findTriangle(vj, true);
        const auto end_tri = m_triangles[end_tri_ind];

        auto tri_ind = start_tri_ind;
        auto tri = m_triangles[tri_ind];
        auto index_in_tri = indexOf(vi, tri);

        auto v_left = tri.verts[prev(index_in_tri)];
        auto v_right = tri.verts[next(index_in_tri)];

        std::vector<EdgeVInd> overlapps;

        bool prev_touched = false;

        // check if the vj is already connected to vi
        do
        {
            if (liesBetween(v_right, vi, vj))
            {
                // if (tri.is_constrained[index_in_tri])
                {
                    // overlapps.push_back({vi, v_right});
                    auto a = indexOf(vi, tri);
                    auto b = indexOf(v_right, tri);
                    auto a_ind = m_tri_ind2vert_inds.at(tri_ind)[a];
                    auto b_ind = m_tri_ind2vert_inds.at(tri_ind)[b];

                    overlapps.push_back({a_ind, b_ind});
                }
                break;
            }
            if (segmentsIntersect(v_left, v_right, vi, vj))
            {
                tri_ind = triangleOppositeOfEdge(tri, {v_left, v_right});
                break;
            }
            tri_ind = tri.neighbours[index_in_tri];
            tri = m_triangles[tri_ind];
            index_in_tri = indexOf(vi, tri);
            v_left = v_right;
            v_right = tri.verts[next(index_in_tri)];
        } while (tri_ind != start_tri_ind);

        auto v_current = v_right;
        auto prev_tri_ind = tri_ind;
        tri_ind = tri.neighbours[next(index_in_tri)];

        auto opp_vertex = v_right;
        auto prev_opp_vertex = opp_vertex;

        while (v_current != vj)
        {
            auto &tri = m_triangles.at(tri_ind);
            index_in_tri = indexOf(v_current, tri);
            opp_vertex = tri.verts[next(index_in_tri)];

            if (liesBetween(v_current, vi, vj) && liesBetween(opp_vertex, vi, vj)) // && tri.is_constrained[index_in_tri])
            {                                                                      //! opposite vertex touches the inserted constraints
                auto a = indexOf(v_current, tri);
                auto b = indexOf(opp_vertex, tri);
                auto a_ind = m_tri_ind2vert_inds.at(tri_ind)[a];
                auto b_ind = m_tri_ind2vert_inds.at(tri_ind)[b];
                if (dot(vj - vi, opp_vertex - v_current) > 0)
                {
                    overlapps.push_back({a_ind, b_ind});
                }
                else
                {
                    overlapps.push_back({b_ind, a_ind});
                }
            }

            //!
            if (orient(vi, vj, opp_vertex) > 0)
            {
                v_current = tri.verts[next(index_in_tri)];
                tri_ind = tri.neighbours[next(index_in_tri)];
                prev_touched = false;
            }
            else if (strictly_less(orient(vi, vj, opp_vertex), 0))
            {
                tri_ind = tri.neighbours[index_in_tri];
            }
            else
            {
                tri_ind = tri.neighbours[next(index_in_tri)];
                v_current = tri.verts[next(index_in_tri)];
            }
        }
        return overlapps;
    }

    // template <class Vertex>
    // void Triangulation<Vertex>::findIntersectingEdges(const EdgeVInd &e, std::deque<EdgeI<Vertex>> &intersected_edges,
    //                                                   std::deque<TriInd> &intersected_tri_inds)
    // {
    //     const auto vi_ind = e.from;
    //     const auto vj_ind = e.to;
    //     if (vi_ind == vj_ind)
    //     {
    //         return;
    //     }
    //     const auto vi = m_vertices[vi_ind];
    //     const auto vj = m_vertices[vj_ind];

    //     const auto start_tri_ind = findTriangle(vi, false);
    //     const auto start_tri = m_triangles[start_tri_ind];
    //     const auto end_tri_ind = findTriangle(vj, true);
    //     const auto end_tri = m_triangles[end_tri_ind];

    //     EdgeI e_inserted(vi, vj);

    //     auto tri_ind = start_tri_ind;
    //     auto tri = m_triangles[tri_ind];
    //     auto index_in_tri = indexOf(vi, tri);
    //     EdgeI e_next = {tri.verts[prev(index_in_tri)], tri.verts[next(index_in_tri)]};
    //     if (index_in_tri == -1)
    //     {
    //         findTriangle(vi, false);
    //     }
    //     bool second_round = false;
    //     //! find first direction of walk by looking at triangles that contain vi;
    //     while (!edgesIntersect(e_next, e_inserted))
    //     {
    //         tri_ind = tri.neighbours[index_in_tri];
    //         tri = m_triangles[tri_ind];
    //         index_in_tri = indexOf(vi, tri);

    //         e_next = {tri.verts[prev(index_in_tri)], tri.verts[next(index_in_tri)]};

    //         if (tri_ind == start_tri_ind)
    //         {
    //             if (second_round) //! we are walking in circles so
    //                 return;
    //             second_round = true;
    //         }

    //         if (e_next.from == vj)
    //         {
    //             m_triangles[tri_ind].is_constrained[prev(index_in_tri)] = true;
    //             const auto tri_ind_opposite = triangleOppositeOfEdge(tri, e_inserted);
    //             const auto ind_in_opposite_tri = indexOf(vi, m_triangles[tri_ind_opposite]);
    //             m_triangles[tri_ind_opposite].is_constrained[ind_in_opposite_tri] = true;
    //             return; //! the end vertex of the constraint is already connected to start vertex;
    //         }
    //         else if (e_next.to() == vj)
    //         {
    //             m_triangles[tri_ind].is_constrained[index_in_tri] = true;
    //             const auto tri_ind_opposite = triangleOppositeOfEdge(tri, e_inserted);
    //             const auto ind_in_opposite_tri = indexOf(vj, m_triangles[tri_ind_opposite]);
    //             m_triangles[tri_ind_opposite].is_constrained[ind_in_opposite_tri] = true;
    //             return;
    //         }
    //     }
    //     intersected_edges.push_back(e_next);
    //     intersected_tri_inds.push_back({tri_ind});
    //     auto v_current = tri.verts[(index_in_tri + 1) % 3];
    //     tri_ind = tri.neighbours[(index_in_tri + 1) % 3];
    //     EdgeI<Vertex> e_next1;
    //     EdgeI<Vertex> e_next2;

    //     //! walk in the found direction to the triangle containing end_ind;
    //     while (tri_ind != end_tri_ind)
    //     {
    //         tri = m_triangles[tri_ind];
    //         index_in_tri = indexOf(v_current, tri);
    //         assert(index_in_tri != -1); //! we expect v_current to always exist in tri

    //         e_next1 = {tri.verts[(index_in_tri) % 3], tri.verts[(index_in_tri + 1) % 3]};
    //         e_next2 = {tri.verts[(index_in_tri + 1) % 3], tri.verts[(index_in_tri + 2) % 3]};
    //         if (e_next1.from == vj or e_next2.to() == vj or e_next1.to() == vj)
    //         {
    //             break; //! we found end_v_ind;
    //         }
    //         intersected_tri_inds.push_back(tri_ind);

    //         if (edgesIntersect(e_next1, e_inserted))
    //         {
    //             tri_ind = tri.neighbours[index_in_tri];
    //             intersected_edges.push_back(e_next1);
    //         }
    //         else if (edgesIntersect(e_next2, e_inserted))
    //         {
    //             intersected_edges.push_back(e_next2);
    //             tri_ind = tri.neighbours[(index_in_tri + 1) % 3];
    //             v_current = tri.verts[(index_in_tri + 1) % 3];
    //         }
    //         else
    //         {
    //             break;
    //         }
    //     }
    //     //    intersected_tri_inds.push_back(tri_ind); //! there is one more triangle compared to intersected edges
    // }

    //! \brief finds existing edges and their corresponding triangles that would intersect with edge \p e
    //! \brief writes the edges into \p intersected_edges and triangles into \p intersected_tri_inds
    //! \param e edge containing vertex indices
    //! \param intersected_edges here the intersected edges are written;
    //! \param intersected_tri_inds here the tri inds corresponding to \p intersected_edges are written
    template <class Vertex>
    void Triangulation<Vertex>::findIntersectingEdges(const EdgeVInd &e, std::deque<EdgeI<Vertex>> &intersected_edges,
                                                      std::deque<TriInd> &intersected_tri_inds)
    {
        const auto vi_ind = e.from;
        const auto vj_ind = e.to;
        if (vi_ind == vj_ind)
        {
            return;
        }
        const auto vi = m_vertices[vi_ind];
        const auto vj = m_vertices[vj_ind];

        const auto start_tri_ind = findTriangle(vi, false);
        const auto start_tri = m_triangles[start_tri_ind];
        const auto end_tri_ind = findTriangle(vj, true);
        const auto end_tri = m_triangles[end_tri_ind];

        EdgeI<Vertex> e_inserted(vi, vj);

        auto tri_ind = start_tri_ind;
        auto tri = m_triangles[tri_ind];
        auto index_in_tri = indexOf(vi, tri);

        auto v_left = tri.verts[prev(index_in_tri)];
        auto v_right = tri.verts[next(index_in_tri)];

        // check if the vj is already connected to vi
        do
        {
            if (v_right == vj)
            {
                m_triangles[tri_ind].is_constrained[index_in_tri] = true;
                const auto tri_ind_opposite = triangleOppositeOfEdge(tri, e_inserted);
                const auto ind_in_opposite_tri = indexOf(vj, m_triangles[tri_ind_opposite]);
                m_triangles[tri_ind_opposite].is_constrained[ind_in_opposite_tri] = true;
                return;
            }
            tri_ind = tri.neighbours[index_in_tri];
            tri = m_triangles[tri_ind];
            index_in_tri = indexOf(vi, tri);
            v_right = tri.verts[next(index_in_tri)];
        } while (tri_ind != start_tri_ind);

        tri_ind = start_tri_ind;
        tri = m_triangles[tri_ind];
        index_in_tri = indexOf(vi, tri);
        v_left = tri.verts[prev(index_in_tri)];
        v_right = tri.verts[next(index_in_tri)];
        //! find first direction of walk by looking at triangles that contain vi;
        while (!segmentsIntersectOrTouch(v_left, v_right, vi, vj))
        {
            tri_ind = tri.neighbours[index_in_tri];
            tri = m_triangles[tri_ind];
            index_in_tri = indexOf(vi, tri);

            v_left = v_right;
            v_right = tri.verts[next(index_in_tri)];
        }

        intersected_edges.push_back({v_left, v_right});
        intersected_tri_inds.push_back({tri_ind});

        auto v_current = tri.verts[next(index_in_tri)];
        tri_ind = tri.neighbours[next(index_in_tri)];
        EdgeI<Vertex> e_next1;
        EdgeI<Vertex> e_next2;

        //! walk in the found direction to the triangle containing end_ind;
        while (tri_ind != end_tri_ind)
        {
            tri = m_triangles[tri_ind];
            index_in_tri = indexOf(v_current, tri);
            assert(index_in_tri != -1); //! we expect v_current to always exist in tri

            e_next1 = {v_current, tri.verts[next(index_in_tri)]};
            e_next2 = {tri.verts[next(index_in_tri)], tri.verts[prev(index_in_tri)]};
            if (e_next1.from == vj || e_next2.to() == vj || e_next1.to() == vj)
            {
                break; //! we found end_v_ind;
            }
            intersected_tri_inds.push_back(tri_ind);

            if (edgesIntersect(e_next1, e_inserted))
            {
                intersected_edges.push_back(e_next1);
                tri_ind = tri.neighbours[index_in_tri];
            }
            else if (edgesIntersect(e_next2, e_inserted))
            {
                intersected_edges.push_back(e_next2);
                tri_ind = tri.neighbours[next(index_in_tri)];
                v_current = tri.verts[next(index_in_tri)];
            }
            else
            {
                break;
            }
        }
    }

    template <class Vertex>
    bool Triangulation<Vertex>::hasGoodOrientation(const Triangle<Vertex> &tri) const
    {
        return isCounterClockwise(tri.verts[2], tri.verts[1], tri.verts[0]) &&
               isCounterClockwise(tri.verts[0], tri.verts[2], tri.verts[1]) &&
               isCounterClockwise(tri.verts[1], tri.verts[0], tri.verts[2]);
    }

    template <class Vertex>
    bool Triangulation<Vertex>::isCounterClockwise(const Vertex &v_query, const Vertex &v1, const Vertex &v2) const
    {
        // Vertex v21_norm = {-v2.y + v1.y, v2.x - v1.x};
        // dot(v_query - v1, v21_norm) >= 0;
        return orient(v_query, v1, v2) >= 0;
    }

    //! \param v    vertex
    //! \param tri  triangle
    //! \returns index in triangle tri corresponding to vertex \p v
    template <class Vertex>
    int Triangulation<Vertex>::indexOf(const Vertex &v, const Triangle<Vertex> &tri) const
    {
        if (tri.verts[0] == v)
        {
            return 0;
        }
        else if (tri.verts[1] == v)
        {
            return 1;
        }
        else if (tri.verts[2] == v)
        {
            return 2;
        }
        return -1;
    }

    //! \brief vertex \p vp and \p v3 must lie opposite to each other
    //! \param vp
    //! \param v1
    //! \param v2
    //! \param v3
    //! \returns true if quadrilateral formed by four vertices is Delaunay
    template <class Vertex>
    bool Triangulation<Vertex>::needSwap(const Vertex &vp, const Vertex &v1, const Vertex &v2, const Vertex &v3) const
    {

        auto v13 = cdt::Vector2f(v1 - v3);
        auto v23 = cdt::Vector2f(v2 - v3);
        auto v1p = cdt::Vector2f(v1 - vp);
        auto v2p = cdt::Vector2f(v2 - vp);

        const auto cos_a = dot(v13, v23);
        const auto cos_b = dot(v1p, v2p);

        if (cos_a >= 0 && cos_b >= 0)
        {
            return false;
        }
        if (cos_a < 0 && cos_b < 0)
        {
            return true;
        }
        const auto sin_ab = static_cast<float>(v13.x * v23.y - v23.x * v13.y) * cos_b +
                            static_cast<float>(v2p.x * v1p.y - v1p.x * v2p.y) * cos_a;

        if (-sin_ab < 0)
        { //! I HAVE NO CLUE WHY THERE HAS TO BE MINUS AND IT MAKES ME SAD!!! (in the article there is plus!)
            return true;
        }

        return false;
    }

    template <class Vertex>
    void Triangulation<Vertex>::dumpToFile(const std::string filename) const
    {

        std::ofstream file(filename);
        if (file.is_open())
        {
            file << "Vertices:\n";
            for (const auto vertex : m_vertices)
            {
                file << vertex.x << " " << vertex.y << "\n";
            }
            file << "Triangles:\n";
            for (const auto &tri : m_triangles)
            {
                file << (int)tri.neighbours[0] << " " << (int)tri.neighbours[1] << " " << (int)tri.neighbours[2] << "\n";
            }
            file.close();
        }
        else
        {
            //        "say some warning message or something";
        }
    }
    template <class Vertex>
    long long Triangulation<Vertex>::det(const Vertex &v1, const Vertex &v2, const Vertex &v3) const
    {
        long long a = v1.x * v2.y + v2.x * v3.y + v3.x * v1.y;
        long long b = v1.y * v2.x + v2.y * v3.x + v3.y * v1.x;
        return a - b;
    }

    //! \brief checks if there are no degenerate triangles (whose points are colinear)
    template <class Vertex>
    bool Triangulation<Vertex>::allTrianglesValid() const
    {
        for (auto &tri : m_triangles)
        {
            if (det(tri.verts[0], tri.verts[1], tri.verts[2]) == 0)
            {
                return false;
            }
        }
        return true;
    }

    template <class Vertex>
    bool Triangulation<Vertex>::isDelaunay(const Triangle<Vertex> &tri) const
    {
        for (int i = 0; i < 3; ++i)
        {
            auto n_ind = tri.neighbours[i];

            if (n_ind != -1)
            {
                auto v_opp = m_vertices.at(oppositeOfEdge(tri, EdgeI{tri.verts[i], tri.verts[next(i)]}));
                if (needSwap(v_opp, tri.verts[i], tri.verts[next(i)], tri.verts[prev(i)]))
                {
                    return false;
                }
            }
        }
        return true;
    }

    template <class Vertex>
    bool Triangulation<Vertex>::allAreDelaunay() const
    {
        for (const auto &tri : m_triangles)
        {
            if (!isDelaunay(tri))
            {
                return false;
            }
        }
        return true;
    }

    template class Triangulation<cdt::Vector2i>;

} // namespace cdt