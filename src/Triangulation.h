#ifndef BOIDS_TRIANGULATION_H
#define BOIDS_TRIANGULATION_H

#include <unordered_set>
#include <deque>
#include <cassert>
#include <memory>
#include <algorithm>
#include <vector>
#include <array>

#include "core.h"
#include "Grid.h"

namespace cdt
{

    using TriInd = unsigned int;
    using VertInd = unsigned int;
    using Vertex = cdt::Vector2i;

    struct EdgeVInd
    {
        VertInd from = -1;
        VertInd to = -1;

        EdgeVInd(VertInd from, VertInd to)
            : from(from), to(to) {}
        EdgeVInd() = default;

        bool operator==(const EdgeVInd &e) const
        {
            return (from == e.from and to == e.to) or (from == e.to and to == e.from);
        }
    };

    struct EdgeI
    {
        Vertex from;
        Vertex t;

        EdgeI() = default;
        EdgeI(const Vertex &v1, const Vertex &v2)
            : from(v1), t(v2 - v1) {}

        float length() const { return norm(t); }
        Vertex to() const { return from + t; }
        bool operator==(const EdgeI &e) const { return e.from == from and e.t == t; }
    };


    //! \struct holds data relating to triangle. Ordering is counterclowise (see image)
    struct Triangle
    {
        Vertex verts[3];                    //! vertex coordinates
        std::array<TriInd, 3> neighbours;   //! indices of neighbouring triangles
        std::array<bool, 3> is_constrained; //! whether corresponding edge is constrained (is this needed here?)

        explicit Triangle()
        {
            neighbours = {-1u, -1u, -1u};
            is_constrained = {false, false, false};
        }

        cdt::Vector2f getCenter() const
        {
            return asFloat(verts[0] + verts[1] + verts[2])/3.f;
        }
    };

    //! \struct function object used to convert an edge into a hash
    struct EdgeHash
    {
        std::size_t operator()(const EdgeI &e) const
        {
            return std::hash<VertInd>()(e.from.x) ^ std::hash<VertInd>()(e.t.x) ^ std::hash<VertInd>()(e.from.y) ^
                   std::hash<VertInd>()(e.t.y);
        }
    };

    struct VertexInsertionData
    {
        VertInd overlapping_vertex = -1;
        EdgeVInd overlapping_edge;
    };

    template <class TriangleT = Triangle>
    class Triangulation
    {
    public:
        explicit Triangulation(cdt::Vector2i box_size);
        Triangulation() = default;

        void reset();
        void createBoundaryAndSuperTriangle(cdt::Vector2i box_size);
        void createBoundary(cdt::Vector2i box_size);
        void createSuperTriangle(cdt::Vector2i box_size);

        TriInd findTriangle(cdt::Vector2f query_point, bool start_from_last_found = false);

        VertInd findOverlappingVertex(const Vertex &new_vertex, const TriInd tri_ind) const;
        EdgeVInd findOverlappingEdge(const Vertex &new_vertex, const TriInd tri_ind) const;

        void insertVertex(const Vertex &v, bool = false);
        void insertVertexIntoSpace(const Vertex &v, TriInd, VertInd);
        void insertConstraint(const EdgeVInd edge);

        VertexInsertionData insertVertexAndGetData(const Vertex &v, bool = false);

        int indexOf(const Vertex &v, const Triangle &tri) const;
        int oppositeIndex(const TriInd np, const Triangle &tri);

        void dumpToFile(const std::string filename) const;

        void updateCellGrid();

        bool allAreDelaunay() const;

    private:
        TriInd findTriangle(Vertex query_point, bool start_from_last_found = false);

        bool edgesIntersect(const EdgeVInd e1, const EdgeVInd e2) const noexcept;
        bool edgesIntersect(const EdgeI e1, const EdgeI e2) const noexcept;

        void insertVertexOnEdge(const Vertex &v, TriInd tri_ind_a, TriInd tri_ind_b, const EdgeI &edge);

        bool isConvex(const Vertex v1, const Vertex v2, const Vertex v3, const Vertex v4) const;

        VertInd oppositeOfEdge(const Triangle &tri, const EdgeI &e) const;

        TriInd triangleOppositeOfEdge(const Triangle &tri, const EdgeI &edge) const;

        void swapConnectingEdge(const TriInd &tri_ind_a, const TriInd &tri_ind_b, int v_ind, Vertex v_a, bool inv = false);

        void findIntersectingEdges(const EdgeVInd &e, std::deque<EdgeI> &intersected_edges,
                                   std::deque<TriInd> &intersected_tri_inds);

        bool isCounterClockwise(const Vertex &v_query, const Vertex &v1, const Vertex &v2) const;
        bool needSwap(const Vertex &vp, const Vertex &v1, const Vertex &v2, const Vertex &v3) const;
        
        long long det(const Vertex &v1, const Vertex &v2, const Vertex &v3) const;
        bool hasGoodOrientation(const Triangle &tri) const;
        bool allTrianglesValid() const;
        
        bool isDelaunay(const Triangle &tri) const;
        bool triangulationIsConsistent() const;

    public:
        std::vector<Triangle> m_triangles;
        std::vector<Vertex> m_vertices;
        std::unordered_set<EdgeI, EdgeHash> m_fixed_edges;

        std::vector<std::array<VertInd, 3>> m_tri_ind2vert_inds;
        std::vector<std::array<bool, 3>> m_triedge_constrained;

    private:
        std::vector<TriInd> m_cell2tri_ind;
        cdt::Vector2i m_boundary;

        TriInd m_last_found = 0;      //! cached index of last found triangle (in a lot of cases new searched triangle is
                                      //! near previously found one)
        std::unique_ptr<Grid> m_grid; //! underlying grid that will be used for finding triangles containing query point
    };

    //! \brief used for finding orientation of \p p1 w.r.t. ( \p p3 - \p p2 )
    template <typename VectorType, typename VectorType2>
    inline float sign(const VectorType &p1, const VectorType2 &p2, const VectorType2 &p3)
    {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    }

    inline int next(const int ind_in_tri)
    {
        assert(ind_in_tri <= 2 && ind_in_tri >= 0);
        if (ind_in_tri == 2)
        {
            return 0;
        }
        return ind_in_tri + 1;
    }

    inline int prev(const int ind_in_tri)
    {
        assert(ind_in_tri <= 2 && ind_in_tri >= 0);
        if (ind_in_tri == 0)
        {
            return 2;
        }
        return ind_in_tri - 1;
    }

    inline int indInTriOf(const Triangle &tri, const TriInd neighbour)
    {
        auto neighbour_it = ::std::find(tri.neighbours.begin(), tri.neighbours.end(), neighbour);
        return neighbour_it - tri.neighbours.begin();
    }

    //! \brief checks if point \p r lies inside the triangle \p tri
    //! \tparam VectorType
    //! \param r
    //! \param tri
    //! \returns true if the point lies inside the triangle
    template <typename VectorType>
    inline bool isInTriangle(const VectorType &r, const Triangle &tri)
    {
        float d1, d2, d3;
        bool has_neg, has_pos;

        d1 = sign(r, tri.verts[0], tri.verts[1]);
        d2 = sign(r, tri.verts[1], tri.verts[2]);
        d3 = sign(r, tri.verts[2], tri.verts[0]);

        has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }

} // namespace cdt

#endif // BOIDS_TRIANGULATION_H
