#pragma once

#include "../Triangulation.h"

#include <unordered_map>


using Vertex = cdt::Vector2i;
using Edge = cdt::EdgeI<Vertex>;

class MapGrid : public cdt::Grid
{

public:
    enum class Tile
    {
        Ground,
        Wall,
    };

    enum class Direction
    {
        Up,
        Down,
        Left,
        Right
    };

public:
    MapGrid(cdt::Vector2i box_size, cdt::Vector2i n_cells);

    void changeTiles(Tile new_tile, cdt::Vector2i lower_left, cdt::Vector2i size);

    std::vector<Edge> extractEdges() const;

private:
    bool isAtBoundary(int ix, int iy) const;
    bool isWall(Direction dir, int ix, int iy) const;

private:
    cdt::Vector2i m_size;
    std::vector<Tile> m_tiles;
};

class MapGridDiagonal : public cdt::Grid
{

public:
    enum class Tile
    {
        Ground,
        Wall,
    };

    //! this is where the normal points
    enum class Direction
    {
        Up,
        Down,
        Left,
        Right,
        RightUp,
        RightDown,
        LeftUp,
        LeftDown,
    };

public:
    MapGridDiagonal(cdt::Vector2i box_size, cdt::Vector2i n_cells);

    void changeTiles(Tile new_tile, cdt::Vector2i lower_left, cdt::Vector2i size);

    std::vector<Edge> extractEdges() const;
    
    void transformCorners();
    void extractBoundaries();

private:
    bool isAtBoundary(int ix, int iy) const;
    bool isWall(Direction dir, int ix, int iy) const;


    enum class BoundaryTile
    {
        Wall,
        Water,
    };

    struct BoundaryData
    {
        Direction dir;
        BoundaryTile type;
    };

    int deltaInd(Direction dir)
    {
        return delta_inds.at(dir);
    }

private:
    cdt::Vector2i m_size;
    std::vector<Tile> m_tiles;
    std::unordered_map<int, BoundaryData> m_boundaries;

    std::unordered_map<Direction, int> delta_inds;
};
