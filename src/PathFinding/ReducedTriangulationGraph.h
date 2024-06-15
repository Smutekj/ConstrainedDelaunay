#pragma once

#include <queue>

#include "../core.h"
#include "../Triangulation.h"


struct Portal {
    cdt::Vector2f first; //! left point of portal when looking through the portal
    cdt::Vector2f second;

    Portal(cdt::Vector2f a, cdt::Vector2f b) : first(a), second(b) {}
};

//! \note first and last element of portals should be one singular point with left == right
typedef std::deque<Portal> Funnel;


//! \struct data  held at a vertex in a reduced triangulation graph
struct ReducedVertex {
    std::array<float, 3> widths = {MAXFLOAT, MAXFLOAT, MAXFLOAT};
    std::array<float, 3> lengths = {0, 0, 0};
    std::array<int, 3> neighbours = {-1, -1, -1};
};

//! \class reduced version of full triangulation graph where we store only edges of crossroad triangles as graph vertices.
//! \class distances between vertices are calculated using paths from funnel algorithm connecting midpoints of the
//! \class crossroads edges through triangle corridors. \class triangles with only one uncostrained edge (dead ends) are also
//! \class  considered vertices;
struct ReducedTriangulationGraph {

    using TriInd = cdt::TriInd;
    using Triangulation = cdt::Triangulation<cdt::Triangle>;

    std::vector<int> tri_ind2vertex;
    std::vector<TriInd> vertex2tri_ind;
    std::vector<ReducedVertex> reduced_vertices;
    std::vector<std::array<int, 3>> vertex2edge_inds2;

    struct TriPathData {
        TriInd current;
        int to;
    };

    struct Corridor {
        TriPathData start;
        TriPathData end;

        float length = 0;
        float width = MAXFLOAT;

        std::vector<TriInd> tri_inds;
        std::vector<int> from_start;
        std::vector<int> from_end;
        Funnel corridor_points;

        Corridor() = default;
    };
    std::vector<Corridor> edges;

  private:
    std::vector<std::array<bool, 3>> visited;

    float sign2(const cdt::Vector2f& p1, const cdt::Vector2f& p2, const cdt::Vector2f& p3) const;
    float calcCorridorWidth(const Funnel& funnel) const;

  public:
    float funnelDistance(const cdt::Vector2f r_start, const cdt::Vector2f r_end, Funnel& fd,
                         const Triangulation& cdt) const;
    void constructFromTriangulation(Triangulation& cdt, std::vector<TriInd>& tri_ind2component);
    void constructFromTriangulationCenters(Triangulation& cdt, std::vector<TriInd>& tri_ind2component);
};
