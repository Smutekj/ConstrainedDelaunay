#include "VisibilityField.h"

VisionField::VisionField(cdt::Triangulation<cdt::Vector2i> &cdt) : m_cdt(cdt) {}

bool VisionField::isVisible(cdt::Vector2f query) const
{
    auto angle_query = dir2angle(query - m_center);
    if (dist(query, m_center) > m_vision_dist)
    {
        return false;
    }
    // if (angle_query < m_min_angle || angle_query > m_max_angle)
    // {
    //     return false;
    // }

    for (const auto &[left, right] : m_vision)
    {
        auto l = orient2(m_center, query, left);
        auto r = orient2(m_center, right, query);
        auto x = orient2(m_center, right, left);
        if (l >= 0.f && r >= 0.f && x >= 0.f)
        {
            return true;
        }
    }
    return false;
}

void VisionField::contrstuctField(cdt::Vector2f from, cdt::Vector2f look_dir)
{
    using namespace cdt;

    auto left_limit = from + m_vision_dist * angle2dir(dir2angle(look_dir) + m_min_angle);
    auto right_limit = from + m_vision_dist * angle2dir(dir2angle(look_dir) + m_max_angle);
    assert(orient2(from, right_limit, left_limit) > 0.f);

    auto in_vision_cone = [from, left_limit, right_limit](cdt::Vector2f point)
    {
        return orient2(from, point, left_limit) >= 0.f && orient2(from, right_limit, point) >= 0.f;
    };
    auto lies_on_line = [](cdt::Vector2f point, cdt::Vector2f &left_line, cdt::Vector2f right_line)
    {
        return approx_equal_zero(cross(left_line - point, right_line - point));
    };

    m_center = from;
    m_vision.clear();
    float max_length = 1000.f;

    const auto &triangles = m_cdt.m_triangles;
    auto start_tri_ind = m_cdt.findTriangle(from, false);

    std::vector<Walker> to_visit;
    auto &curr_tri = triangles.at(start_tri_ind);

    //! when standing on some line,
    //! we move the point very little towards triangle center
    //!  so that we don't have to deal with it (dirty but maybe works)
    auto v1 = asFloat(curr_tri.verts[0]);
    auto v2 = asFloat(curr_tri.verts[1]);
    auto v3 = asFloat(curr_tri.verts[2]);
    auto center = cdt::Vector2f(v1 + v2 + v3) / 3.f;
    if (lies_on_line(from, v2, v1) || lies_on_line(from, v3, v2) || lies_on_line(from, v1, v3))
    {
        from = from + (center - from) / norm(center - from) * 0.001f;
    }

    for (int i = 0; i < 3; ++i)
    {
        auto left = asFloat(curr_tri.verts[next(i)]);
        auto right = asFloat(curr_tri.verts[i]);

        // if (!in_vision_cone(left))
        // { //! find where left side of vision cone hit the tri-edge
        //     auto end_left = from + max_length * (left_limit - from) / norm(left_limit - from);
        //     if (!segmentsIntersect(from, end_left, right, left, left))
        //     {
        //         continue;
        //     }
        // }
        // if (!in_vision_cone(right))
        // { //! find where right side of vision cone hit the tri-edge
        //     auto end_right = from + max_length * (right_limit - from) / norm(right_limit - from);
        //     if (!segmentsIntersect(from, end_right, right, left, right))
        //     {
        //         //! if there is no intersection the side is not visible
        //         continue;
        //     }
        // }

        if (!curr_tri.is_constrained[i])
        {
            to_visit.push_back({start_tri_ind, curr_tri.neighbours[i], left, right});
        }
        else
        {
            m_vision.emplace_back(left, right);
        }
    }

    while (!to_visit.empty())
    {

        auto curr_tri_ind = to_visit.back().curr_tri_ind;
        auto prev_tri_ind = to_visit.back().prev_tri_ind;
        auto left = to_visit.back().left;
        auto right = to_visit.back().right;
        auto &curr_tri = triangles.at(curr_tri_ind);
        to_visit.pop_back();
        if (dist(left, from) > max_length && dist(right, from) > max_length)
        {
            auto end_left = from + max_length * (left - from) / norm(left - from);
            auto end_right = from + max_length * (right - from) / norm(right - from);
            m_vision.emplace_back(end_left, end_right);
            continue;
        }

        if (orient2(from, right, left) < 0.f) //! we do not see anything anymore
        {
            continue;
        }

        auto opposite_ind_in_tri = m_cdt.oppositeIndex(prev_tri_ind, curr_tri);
        auto opposite_vert = asFloat(curr_tri.verts[opposite_ind_in_tri]);
        auto left_vert = asFloat(curr_tri.verts[next(opposite_ind_in_tri)]);
        auto right_vert = asFloat(curr_tri.verts[prev(opposite_ind_in_tri)]);

        auto left_neighbour = curr_tri.neighbours[opposite_ind_in_tri];
        auto right_neighbour = curr_tri.neighbours[prev(opposite_ind_in_tri)];

        auto o_left = orient2(from, opposite_vert, left);
        auto o_right = orient2(from, right, opposite_vert);

        bool left_is_transparent = !curr_tri.is_constrained[opposite_ind_in_tri];
        bool right_is_transparent = !curr_tri.is_constrained[prev(opposite_ind_in_tri)];
        bool left_was_added = vequal(left_vert, left);
        bool right_was_added = vequal(right_vert, right);

        if (o_left >= 0.f && o_right >= 0.f) //! opposite vertex is in vision field
        {

            if (left_is_transparent)
            {
                to_visit.push_back({curr_tri_ind, left_neighbour, left, opposite_vert});
            }
            else
            {
                if (left_was_added)
                {
                    m_vision.emplace_back(left, opposite_vert);
                }
                else
                {
                    auto end_left = from + max_length * (left - from) / norm(left - from);
                    cdt::Vector2f new_left = end_left;
                    segmentsIntersect(from, end_left, opposite_vert, left_vert, new_left);
                    m_vision.emplace_back(new_left, opposite_vert);
                }
            }
            if (right_is_transparent)
            {
                to_visit.push_back({curr_tri_ind, right_neighbour, opposite_vert, right});
            }
            else
            {
                if (right_was_added)
                {
                    m_vision.emplace_back(opposite_vert, right);
                }
                else
                {
                    auto end_right = from + max_length * (right - from) / norm(right - from);
                    cdt::Vector2f new_right = end_right;
                    segmentsIntersect(from, end_right, right_vert, opposite_vert, new_right);
                    m_vision.emplace_back(opposite_vert, new_right);
                }
            }
        }
        else if (o_left < 0.f) //! oppposite vert is not visible and above left
        {
            auto end_left = from + max_length * (left - from) / norm(left - from);
            cdt::Vector2f new_left = end_left;
            if (segmentsIntersect(from, end_left, right_vert, opposite_vert, new_left))
            {
            }
            if (right_is_transparent)
            {
                assert(!std::isnan(new_left.x) && !std::isnan(new_left.y));
                to_visit.push_back({curr_tri_ind, right_neighbour, new_left, right});
            }
            else
            {
                if (!right_was_added)
                {
                    auto end_right = from + max_length * (right - from) / norm(right - from);
                    cdt::Vector2f new_right = end_right;
                    segmentsIntersect(from, end_right, right_vert, opposite_vert, new_right);
                    m_vision.emplace_back(new_left, new_right);
                }
                else
                {
                    m_vision.emplace_back(new_left, right);
                }
            }
        }
        else if (o_right < 0.f) //! oppposite vert is not visible and below right
        {
            auto end_right = from + max_length * (right - from) / norm(right - from);
            cdt::Vector2f new_right = end_right;
            if (segmentsIntersect(from, end_right, left_vert, opposite_vert, new_right))
            {
            }
            if (left_is_transparent)
            {
                assert(!std::isnan(new_right.x) && !std::isnan(new_right.y));
                to_visit.push_back({curr_tri_ind, left_neighbour, left, new_right});
            }
            else
            {
                if (!left_was_added)
                {
                    auto end_left = from + max_length * (left - from) / norm(left - from);
                    cdt::Vector2f new_left = end_left;
                    segmentsIntersect(from, end_left, left_vert, opposite_vert, new_left);
                    m_vision.emplace_back(new_left, new_right);
                }
                else
                {
                    m_vision.emplace_back(left, new_right);
                }
            }
        }
        else
        {
            throw std::runtime_error("floating point error in orientation calculation!");
        }
    }
}

sf::VertexArray VisionField::getDrawVertices() const
{

    auto from = m_center;
    auto angle_dist_cond = [from](VisionCone &r1, VisionCone &r2)
    {
        auto a1 = dir2angle(r1.left - from);
        auto a2 = dir2angle(r2.left - from);
        return a1 < a2;
    };
    // std::sort(m_vision.begin(), m_vision.end(), angle_dist_cond);
    int n_rays = m_vision.size();

    sf::VertexArray vertices;
    vertices.setPrimitiveType(sf::Triangles);
    vertices.resize(3 * n_rays);

    sf::Vertex center_vert;
    center_vert.position = {from.x, from.y};
    center_vert.color = sf::Color(255, 255, 255, 255);
    for (int i = 0; i < n_rays; ++i)
    {

        vertices[3 * i + 0] = center_vert;

        vertices[3 * i + 1].position = {m_vision[i].left.x, m_vision[i].left.y};
        vertices[3 * i + 1].color = sf::Color(255, 255, 255, 255);

        vertices[3 * i + 2].position = {m_vision[i].right.x, m_vision[i].right.y};
        vertices[3 * i + 2].color = sf::Color(255, 255, 255, 255);
    }
    return vertices;
}
