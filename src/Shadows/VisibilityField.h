#pragma once

#include "../Triangulation.h"

#include <SFML/Graphics/VertexArray.hpp>


struct VisionCone
{
    cdt::Vector2f left;
    cdt::Vector2f right;
    VisionCone() = default;
    VisionCone(cdt::Vector2f left, cdt::Vector2f right)
        : left(left), right(right) {}
};

class VisionField
{

    struct Walker
    {
        cdt::TriInd prev_tri_ind;
        cdt::TriInd curr_tri_ind;
        cdt::Vector2f left;
        cdt::Vector2f right;
    };

public:
    VisionField(cdt::Triangulation<cdt::Vector2i> &cdt);

    bool            isVisible(cdt::Vector2f query) const;

    void            contrstuctField(cdt::Vector2f from, cdt::Vector2f look_dir);
    
    sf::VertexArray getDrawVertices() const;

private:
    cdt::Vector2f m_center;
    float m_vision_dist = 100.f;
    float m_min_angle = -60;
    float m_max_angle = +60;

    std::vector<VisionCone> m_vision;
    cdt::Triangulation<cdt::Vector2i> &m_cdt;
};


inline cdt::Vector2f angle2dir(float angle)
{
    cdt::Vector2f dir;
    dir.x = std::cos(angle * M_PIf / 180.f);
    dir.y = std::sin(angle * M_PIf / 180.f);
    return dir;
}

inline float dir2angle(const cdt::Vector2f &dir)
{
    return std::atan2(dir.y, dir.x) * 180.f / M_PIf;
}
