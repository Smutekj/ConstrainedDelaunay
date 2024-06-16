#include "PathFinder.h"

#include <future>
#include <thread>
#include <chrono>
#include <fstream>
#include <iostream>

PathFinder::PathFinder(Triangulation<Triangle> &cdt)
    : m_cdt(&cdt)
{
    m_rtg = std::make_unique<ReducedTriangulationGraph>();
}

//! \brief creates everything from p_triangulation_
//! \brief should be called on triangulation change
void PathFinder::update()
{

    const auto n_triangles = m_cdt->m_triangles.size();

    triangle2tri_widths_.resize(n_triangles);
    for (int tri_ind = 0; tri_ind < n_triangles; ++tri_ind)
    {
        triangle2tri_widths_[tri_ind] = TriangleWidth(m_cdt->m_triangles[tri_ind], *m_cdt);
    }
    m_g_values.resize(n_triangles);
    m_back_pointers.resize(n_triangles);

    m_cdt->updateCellGrid();
}

void dumpFunnelToFile(const Funnel &funnel, float radius, std::string filename)
{
    std::ofstream file(filename);

    for (const auto [r_left, r_right] : funnel)
    {
        file << r_left.x << " " << r_left.y << " " << r_right.x << " " << r_right.y << '\n';
    }
    file.close();
}

void dumpPathToFile(const std::deque<cdt::Vector2f> &path, float radius, std::string filename)
{
    std::ofstream file(filename);

    for (const auto r : path)
    {
        file << r.x << " " << r.y << '\n';
    }
    file.close();
}

void dumpFunnelToFile2(const std::vector<Vertex> &r_lefts, const std::vector<Vertex> &r_rights, std::string filename)
{
    std::ofstream file(filename);

    for (auto r_left : r_lefts)
    {
        file << r_left.x << " " << r_left.y << '\n';
    }
    file << "\n";

    for (auto r_right : r_rights)
    {
        file << r_right.x << " " << r_right.y << '\n';
    }

    file.close();
}

PathFinder::PathData PathFinder::doPathFinding(const cdt::Vector2f r_start, const cdt::Vector2f r_end, const float radius)
{

    Funnel funnel;
    findSubOptimalPathCenters(r_start, r_end, radius, funnel);

    funnel.push_back({r_start, r_start});
    std::reverse(funnel.begin(), funnel.end());
    funnel.push_back({r_end, r_end});

    return pathFromFunnel(r_start, r_end, radius, funnel);
    ;
}

//! \brief finds sequence of triangles such that the path going through centers of the triangles is the shortest
//! \param r_start starting position
//! \param r_end end position
//! \param radius to block paths that are too narrow
//! \param funnel stores data used to create real path going through the triangles
//! \param thread_id which thread runs the job
void PathFinder::findSubOptimalPathCenters(cdt::Vector2f r_start, cdt::Vector2f r_end, float radius, Funnel &funnel)
{

    const auto &triangles = m_cdt->m_triangles;
    const auto &vertices = m_cdt->m_vertices;
    const auto start = m_cdt->findTriangle(r_start, false);
    const auto end = m_cdt->findTriangle(r_end, false);

    if (start == end or start == -1 or end == -1)
    {
        return;
    }

    std::vector<AstarDataPQ> to_visit;
    for (int i = 0; i < triangles.size(); ++i)
    {
        m_g_values[i] = MAXFLOAT;
        m_back_pointers[i] = -1;
    }

    std::priority_queue to_visit_pque(
        to_visit.begin(), to_visit.end(),
        [&](const AstarDataPQ &a1, const AstarDataPQ &a2)
        { return a1.f_value > a2.f_value; });

    m_g_values.at(start) = 0;
    to_visit_pque.push({start, 0});

    while (!to_visit_pque.empty())
    {
        auto current_tri_ind = to_visit_pque.top().next;
        auto f_value = to_visit_pque.top().f_value;
        to_visit_pque.pop();

        const auto &current_tri = triangles[current_tri_ind];
        std::vector<int> free_neighbours;
        int prev_ind_in_tri = -1;

        for (const auto neighbour : current_tri.neighbours)
        {

            auto ind_in_tri = indInTriOf(current_tri, neighbour);
            const auto &neighbour_tri = triangles[neighbour];
            if (current_tri.is_constrained[ind_in_tri] || ind_in_tri == 3)
            { //! This means that the edge is a wall
                continue;
            }

            float width = MAXFLOAT;
            width = triangle2tri_widths_[current_tri_ind].widths[ind_in_tri];

            const auto t1 = asFloat(current_tri.verts[0] + current_tri.verts[1] + current_tri.verts[2]) / 3.f;
            const auto t2 = asFloat(neighbour_tri.verts[0] + neighbour_tri.verts[1] + neighbour_tri.verts[2]) / 3.f;
            const auto distance = dist(t1, t2);
            const auto distance_to_end = dist(r_end, t1);
            const auto h_value = dist(r_end, t2);
            const auto new_g_value = m_g_values.at(current_tri_ind) + distance;
            if (m_g_values.at(neighbour) > new_g_value && width > 2 * radius)
            {
                m_g_values.at(neighbour) = m_g_values.at(current_tri_ind) + distance;
                m_back_pointers.at(neighbour) = current_tri_ind;
                to_visit_pque.push({neighbour, h_value + new_g_value});
            }
        }
    }
    //! walk backwards from finish to start;
    //! current_tri_ind = end;
    Funnel path;
    auto current_tri_ind = end;
    while (current_tri_ind != start)
    {
        if (m_back_pointers[current_tri_ind] == -1)
        {
            return;
        }
        const auto &tri = triangles[current_tri_ind];

        auto ind_in_tri = indInTriOf(tri, m_back_pointers[current_tri_ind]);
        if (ind_in_tri == 3)
        {
            return;
        }
        auto next_neighbour = tri.neighbours.at(ind_in_tri);

        const auto left_vertex_of_portal = asFloat(tri.verts[ind_in_tri]);
        const auto right_vertex_of_portal = asFloat(tri.verts[next(ind_in_tri)]);
        assert(!std::isnan(right_vertex_of_portal.x) && !std::isnan(left_vertex_of_portal.x));
        funnel.emplace_back(right_vertex_of_portal, left_vertex_of_portal);

        current_tri_ind = m_back_pointers[current_tri_ind];
    }
}

//! \param r  starting point
//! \param start_tri_ind index of triangle containing starting point of the sought path
//! \param end_component triangulation component of the end point
//! \param navigable_component triangulation component of the start point
//! \returns first in pair is triangle ind of the given navigable_component
//! \returns  second is ind_in_tri looking into end_component
std::pair<TriInd, int> PathFinder::closestPointOnNavigableComponent(const cdt::Vector2f &query_point, const TriInd start_tri_ind,
                                                                    const int end_component,
                                                                    const int navigable_component) const
{
    const auto &triangles = m_cdt->m_triangles;
    const auto &vertices = m_cdt->m_vertices;
    auto current_tri_ind = start_tri_ind;
    std::queue<TriInd> to_visit({current_tri_ind});

    auto current_component = end_component;
    auto prev_tri_ind = current_tri_ind;
    while (current_component != navigable_component and !to_visit.empty())
    {
        current_tri_ind = to_visit.front();
        const auto &tri = triangles[current_tri_ind];
        to_visit.pop();

        auto tri_center = triangles[start_tri_ind].getCenter();

        for (int k = 0; k < 3; ++k)
        {
            const auto neighbour = tri.neighbours[k];
            if (neighbour != prev_tri_ind and neighbour != -1)
            {
                auto v1 = asFloat(tri.verts[k]);
                auto v2 = asFloat(tri.verts[next(k)]);
                if (segmentsIntersect(tri_center, query_point, v1, v2))
                {
                    to_visit.push(neighbour);
                }
            }
        }
        prev_tri_ind = current_tri_ind;
        current_component = tri_ind2component_[current_tri_ind];
    }
    const auto to_end_component = indInTriOf(triangles[current_tri_ind], prev_tri_ind);
    return {current_tri_ind, to_end_component};
}

float calcWidth(cdt::Vector2f pos, Edgef segment)
{
    auto d1 = dist(pos, segment.from);
    auto d2 = dist(pos, segment.to());

    auto proj_on_segment = dot(pos - segment.from, segment.t);
    if (proj_on_segment < 0) //! point lies in front of segment.from
    {
        return dist(pos, segment.from);
    }
    if (proj_on_segment > segment.l) //! point lies behind segment.to
    {
        return dist(pos, segment.to());
    }

    //! point lies "in between" segment.from and segment.to
    cdt::Vector2f n = {segment.t.y, -segment.t.x};
    //! distance of orthogonal projection on segment
    auto norm_dist = std::abs(dot(pos - segment.from, n));
    return norm_dist;
}

PathFinder::TriangleWidth::TriangleWidth(Triangle &tri, const Triangulation<Triangle> &cdt)
{

    int n_constraints = 0;
    int free_ind = 0;

    for (int i = 0; i < 3; ++i)
    {
        widths[i] = dist(tri.verts[i], tri.verts[next(i)]); //;
        n_constraints += tri.is_constrained[i];

        if (!tri.is_constrained[i] && !tri.is_constrained[prev(i)])
        {
            free_ind = i;
        }
    }
    //! if there are two free edges, the width is the orthogonal projection to the constrained edge
    if (n_constraints == 1)
    {
        widths[free_ind] = calcWidth(tri.verts[free_ind],
                                     {tri.verts[prev(free_ind)], tri.verts[next(free_ind)]});
        widths[prev(free_ind)] = calcWidth(tri.verts[free_ind],
                                     {tri.verts[prev(free_ind)], tri.verts[next(free_ind)]});
    }
}

//! \brief imagine r_to_push being between r_prev and r_next
//! \brief r_to_push is pushed in the average of normal directions of (r_to_push - r_prev) and (r_next - r_to_push)
//! \param r_to_push point to push away
//! \param r_prev
//! \param r_next
//! \param distance how far away the point is pushed
//! \returns path portal coming from pushed
Edgef PathFinder::pushAwayFromCorner(cdt::Vector2f &r_to_push, const cdt::Vector2f &r_prev, const cdt::Vector2f &r_next,
                                     const float distance, bool left) const
{

    const auto v0 = r_to_push;
    const auto v1 = r_prev;
    const auto v2 = r_next;

    cdt::Vector2f v10 = v1 - v0;
    cdt::Vector2f v20 = v2 - v0;
    v10 /= norm(v10);
    v20 /= norm(v20);
    auto vertex_normal = v10 + v20;
    if (left && orient(r_prev, r_to_push, r_next) < 0)
    {
        vertex_normal *= -1.f;
    }
    if (!left && orient(r_prev, r_to_push, r_next) > 0)
    {
        vertex_normal *= -1.f;
    }

    if (approx_equal_zero(vertex_normal.x) && approx_equal_zero(vertex_normal.y))
    {
        if (left)
        {
            vertex_normal = {v10.y, -v10.x};
        }
        else
        {
            vertex_normal = {v20.y, -v20.x};
        }
    }

    r_to_push += (vertex_normal)*distance / norm(vertex_normal);

    assert(!std::isnan(r_to_push.x) && !std::isnan(r_to_push.y));

    Edgef p;
    p.from = r_to_push;
    // p.t = (n10 + n20) / norm(n10 + n20);
    p.l = 10 * distance; //!
    return p;
}

//! \brief imagine r_to_push being between r_prev and r_next
//! \brief r_to_push is pushed in the average of normal directions of (r_to_push - r_prev) and (r_next - r_to_push)
//! \param r_to_push point to push away
//! \param r_prev
//! \param r_next
//! \param distance how far away the point is pushed
//! \returns path portal coming from pushed
Edgef createPortal(cdt::Vector2f &r_to_push, const cdt::Vector2f &r_prev, const cdt::Vector2f &r_next, const float distance, bool left)
{

    const auto v0 = r_to_push;
    const auto v1 = r_prev;
    const auto v2 = r_next;
    const auto dv20 = (v2 - v0) / norm(v2 - v0);
    const auto dv10 = (v1 - v0) / norm(v1 - v0);
    auto vertex_normal = (dv10 + dv20) / norm(dv20 + dv10);
    if ((2.f * left - 1.f) * sign(r_next, r_prev, r_to_push) > 0)
    {
        vertex_normal *= -1.f;
    }

    Edgef p;
    p.from = r_to_push + vertex_normal * distance;
    r_to_push += vertex_normal * distance;
    p.t = vertex_normal;
    p.l = 10 * distance; //!
    return p;
}

//! \brief crates shorte.st path inside a Polygon defined by FunnelData
//! \param r_start starting position of the path
//! \param r_end end position of the path
//! \param radius defines how much the path will be pushed away from corners
//! \returns shortest path inside funnel and portals (line segments from path points indicating that I passed the path
//! point)
PathFinder::PathData PathFinder::pathFromFunnel(const cdt::Vector2f r_start, const cdt::Vector2f r_end,
                                                const float radius, Funnel &funnel) const
{

    PathData path_and_portals;

    auto &smoothed_path = path_and_portals.path;
    auto &portals = path_and_portals.portals;
    path_and_portals.path = {r_start};
    path_and_portals.portals = {Edgef()};

    const auto &triangles = m_cdt->m_triangles;
    const auto &vertices = m_cdt->m_vertices;

    cdt::Vector2f right;
    cdt::Vector2f left;
    cdt::Vector2f portal_apex = r_start;
    cdt::Vector2f portal_right = r_start;
    cdt::Vector2f portal_left = r_start;

    int right_index = 0;
    int left_index = 0;
    int apex_index = 0;

    std::vector<Edgef> left_portals;
    std::vector<Edgef> right_portals;

    left_portals.push_back(Edgef());
    right_portals.push_back(Edgef());
    cdt::Vector2f prev_left = r_start;
    cdt::Vector2f prev_right = r_start;
    int i_first_same = 0;
    bool is_first = true;

    std::vector<int> unique_left({0});
    std::vector<int> unique_right({0});
    for (int i = 1; i < funnel.size(); ++i)
    {
        auto &next_r = funnel[i].first;
        auto &next_l = funnel[i].second;
        if (!vequal(next_l, prev_left))
        {
            unique_left.push_back(i);
            prev_left = next_l;
        }
        if (!vequal(next_r, prev_right))
        {
            unique_right.push_back(i);
            prev_right = next_r;
        }
    }

    //! push path points away from walls
    const auto push_distance = radius;
    for (int i = 1; i < unique_left.size() - 1; ++i)
    {
        const auto &prev_unique_left = funnel[unique_left[i - 1]].second;
        const auto &next_unique_left = funnel[unique_left[i + 1]].second;
        for (int j = unique_left[i]; j < unique_left[i + 1]; ++j)
        {
            auto &mid_left = funnel[j].second;
            const auto left_portal = pushAwayFromCorner(mid_left, prev_unique_left, next_unique_left, push_distance, true);
            left_portals.push_back(left_portal);
        }
    }
    for (int i = 1; i < unique_right.size() - 1; ++i)
    {
        const auto &prev_unique_right = funnel[unique_right[i - 1]].first;
        const auto &next_unique_right = funnel[unique_right[i + 1]].first;
        for (int j = unique_right[i]; j < unique_right[i + 1]; ++j)
        {
            auto &mid_right = funnel[j].first;
            const auto right_portal =
                pushAwayFromCorner(mid_right, prev_unique_right, next_unique_right, push_distance, false);
            right_portals.push_back(right_portal);
        }
    }

    path_and_portals.funnel = funnel;

    left_portals.push_back(Edgef());
    right_portals.push_back(Edgef());
    for (int i = 1; i < funnel.size(); ++i)
    {

        right = funnel[i].first;
        left = funnel[i].second;

        if (sign(portal_apex, portal_right, right) <= 0.f)
        { //! if the portal shrank from right
            auto is_same_point = vequal(portal_apex, portal_right);
            if (is_same_point || sign(portal_apex, portal_left, right) >
                                     0.f)
            { //! if the new right segment of the portal crosses the left segment
                portal_right = right;
                right_index = i;
            }
            else if (vequal(portal_right, right))
            {
                right_index = i;
            }
            else
            {
                portals.push_back(left_portals.at(left_index));

                smoothed_path.push_back(portal_left);
                portal_apex = portal_left;
                portal_left = portal_apex;
                portal_right = portal_apex;
                apex_index = left_index;
                right_index = left_index;
                i = left_index;
                continue;
            }
        }

        if (sign(portal_apex, portal_left, left) >= 0.f)
        { //! same as above but we move left portal segment
            auto is_same_point = vequal(portal_apex, portal_left);
            //            if(vequal(portal_left, left)){ continue;}
            if (is_same_point or sign(portal_apex, portal_right, left) < 0.f)
            {
                portal_left = left;
                left_index = i;
            }
            else if (vequal(portal_left, left))
            {
                left_index = i;
            }
            else
            {
                portals.push_back(right_portals.at(right_index));

                smoothed_path.push_back(portal_right);
                portal_apex = portal_right;
                portal_right = portal_apex;
                portal_left = portal_apex;
                apex_index = right_index;
                left_index = right_index;
                i = right_index;
                continue;
            }
        }
    }
    smoothed_path.push_back(r_end);
    portals.push_back(Edgef());
    return path_and_portals;
}

//! TODO: figure out why I get segfault when I use sign(..) function from "core.h"
//!       but not this one with optimisation lvl at least -O2?
//!
float PathFinder::sign(cdt::Vector2f a, cdt::Vector2f b, cdt::Vector2f c) const
{
    const auto ax = b.x - a.x;
    const auto ay = b.y - a.y;
    const auto bx = c.x - a.x;
    const auto by = c.y - a.y;
    const auto area = by * ax - ay * bx;
    return area;
}
