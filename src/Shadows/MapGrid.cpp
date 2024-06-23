#include "MapGrid.h"

std::vector<Edge> MapGrid::extractEdges() const
{
    std::vector<Edge> edges;

    for (int iy = 1; iy < m_cell_count.y - 1; ++iy)
    {
        bool is_scanning_up = false;
        bool is_scanning_down = false;
        Vertex vl_up, vr_up;
        Vertex vl_down, vr_down;
        for (int ix = 1; ix < m_cell_count.x; ++ix)
        {
            if (is_scanning_up && !isWall(Direction::Up, ix, iy))
            {
                is_scanning_up = false;
                vr_up = {ix, iy};
                edges.push_back({vl_up, vr_up});
            }
            if (!is_scanning_up && isWall(Direction::Up, ix, iy))
            {
                vl_up = {ix, iy};
                is_scanning_up = true;
            }

            if (is_scanning_down && !isWall(Direction::Down, ix, iy))
            {
                is_scanning_down = false;
                vr_down = {ix, iy};
                edges.push_back({vr_down, vl_down});
            }
            if (!is_scanning_down && isWall(Direction::Down, ix, iy))
            {
                is_scanning_down = true;
                vl_down = {ix, iy};
            }
        }
    }

    //! scan up-down
    for (int ix = 1; ix < m_cell_count.x - 1; ++ix)
    {
        bool is_scanning_left = false;
        bool is_scanning_right = false;
        Vertex vl_up, vr_up;
        Vertex vl_down, vr_down;
        for (int iy = 1; iy < m_cell_count.y; ++iy)
        {
            if (is_scanning_left && !isWall(Direction::Left, ix, iy))
            {
                is_scanning_left = false;
                vr_up = {ix, iy};
                edges.push_back({vl_up, vr_up});
            }
            if (!is_scanning_left && isWall(Direction::Left, ix, iy))
            {
                vl_up = {ix, iy};
                is_scanning_left = true;
            }

            if (is_scanning_right && !isWall(Direction::Right, ix, iy))
            {
                is_scanning_right = false;
                vr_down = {ix, iy};
                edges.push_back({vr_down, vl_down});
            }
            if (!is_scanning_right && isWall(Direction::Right, ix, iy))
            {
                is_scanning_right = true;
                vl_down = {ix, iy};
            }
        }
    }
    return edges;
}

bool MapGrid::isAtBoundary(int ix, int iy) const
{
    return ix >= m_cell_count.x - 1 || iy >= m_cell_count.y - 1 ||
           ix < 1 || iy < 1;
}

bool MapGrid::isWall(Direction dir, int ix, int iy) const
{
    if (isAtBoundary(ix, iy))
    {
        return false;
    }

    auto tile = m_tiles.at(cellIndex(ix, iy));
    auto tile_up = m_tiles.at(cellIndex(ix, iy - 1));
    auto tile_left = m_tiles.at(cellIndex(ix - 1, iy));

    if (dir == Direction::Down)
    {
        return tile == Tile::Ground && tile_up != Tile::Ground;
    }
    else if (dir == Direction::Up)
    {
        return tile != Tile::Ground && tile_up == Tile::Ground;
    }
    else if (dir == Direction::Right)
    {
        return tile == Tile::Ground && tile_left != Tile::Ground;
    }
    else if (dir == Direction::Left)
    {
        return tile != Tile::Ground && tile_left == Tile::Ground;
    }
}

MapGrid::MapGrid(cdt::Vector2i box_size, cdt::Vector2i n_cells) : Grid(n_cells, box_size)
{
    m_tiles.resize(getNCells());
}

void MapGrid::changeTiles(Tile new_tile, cdt::Vector2i lower_left, cdt::Vector2i size)
{
    for (int dy = 0; dy < size.y; ++dy)
    {
        for (int dx = 0; dx < size.x; ++dx)
        {
            auto ix = lower_left.x + dx;
            auto iy = lower_left.y + dy;
            if (isAtBoundary(ix, iy))
            {
                continue;
            }
            auto cell_ind = cellIndex(ix, iy);
            m_tiles.at(cell_ind) = new_tile;
        }
    }
}

bool MapGridDiagonal::isWall(Direction dir, int ix, int iy) const
{
    if (isAtBoundary(ix, iy))
    {
        return false;
    }
    auto cell_ind = cellIndex(ix, iy);
    return m_boundaries.count(cell_ind) != 0 && m_boundaries.at(cell_ind).dir == dir;
}

MapGridDiagonal::MapGridDiagonal(cdt::Vector2i box_size, cdt::Vector2i n_cells) : Grid(n_cells, box_size)
{
    m_tiles.resize(getNCells());

    delta_inds[Direction::Right] = 1;
    delta_inds[Direction::RightDown] = 1 + m_cell_count.x;
    delta_inds[Direction::Down] = m_cell_count.x;
    delta_inds[Direction::LeftDown] = -1 + m_cell_count.x;
    delta_inds[Direction::Left] = -1;
    delta_inds[Direction::LeftUp] = -1 - m_cell_count.x;
    delta_inds[Direction::Up] = -m_cell_count.x;
    delta_inds[Direction::RightUp] = 1 - m_cell_count.x;
}

void MapGridDiagonal::changeTiles(Tile new_tile, cdt::Vector2i lower_left, cdt::Vector2i size)
{
    for (int dy = 0; dy < size.y; ++dy)
    {
        for (int dx = 0; dx < size.x; ++dx)
        {
            auto ix = lower_left.x + dx;
            auto iy = lower_left.y + dy;
            if (isAtBoundary(ix, iy))
            {
                continue;
            }
            auto cell_ind = cellIndex(ix, iy);
            m_tiles.at(cell_ind) = new_tile;
        }
    }
}

std::vector<Edge> MapGridDiagonal::extractEdges() const
{
    std::vector<Edge> edges;

    for (int iy = 1; iy < m_cell_count.y - 1; ++iy)
    {
        bool is_scanning_up = false;
        bool is_scanning_down = false;
        Vertex vl_up, vr_up;
        Vertex vl_down, vr_down;
        for (int ix = 1; ix < m_cell_count.x; ++ix)
        {
            if (is_scanning_up && !isWall(Direction::Up, ix, iy))
            {
                is_scanning_up = false;
                vr_up = {ix, iy};
                edges.push_back({vl_up, vr_up});
            }
            if (!is_scanning_up && isWall(Direction::Up, ix, iy))
            {
                vl_up = {ix, iy};
                is_scanning_up = true;
            }

            if (is_scanning_down && !isWall(Direction::Down, ix, iy))
            {
                is_scanning_down = false;
                vr_down = {ix, iy + 1};
                edges.push_back({vr_down, vl_down});
            }
            if (!is_scanning_down && isWall(Direction::Down, ix, iy))
            {
                is_scanning_down = true;
                vl_down = {ix, iy + 1};
            }
        }
    }

    //! scan up-down
    for (int ix = 1; ix < m_cell_count.x - 1; ++ix)
    {
        bool is_scanning_left = false;
        bool is_scanning_right = false;
        Vertex vl_up, vr_up;
        Vertex vl_down, vr_down;
        for (int iy = 1; iy < m_cell_count.y; ++iy)
        {
            if (is_scanning_left && !isWall(Direction::Left, ix, iy))
            {
                is_scanning_left = false;
                vr_up = {ix, iy};
                edges.push_back({vl_up, vr_up});
            }
            if (!is_scanning_left && isWall(Direction::Left, ix, iy))
            {
                vl_up = {ix, iy};
                is_scanning_left = true;
            }

            if (is_scanning_right && !isWall(Direction::Right, ix, iy))
            {
                is_scanning_right = false;
                vr_down = {ix + 1, iy};
                edges.push_back({vr_down, vl_down});
            }
            if (!is_scanning_right && isWall(Direction::Right, ix, iy))
            {
                is_scanning_right = true;
                vl_down = {ix + 1, iy};
            }
        }
    }

    //! scan diagonally from left up to right down
    const auto i_diag_max = m_cell_count.x + m_cell_count.y - 2;
    for (int i_diag = 0; i_diag < i_diag_max; ++i_diag)
    {
        int ix = std::max(0, i_diag - m_cell_count.x);
        int iy = std::max(0, m_cell_count.y - i_diag);
        bool is_scanning_down = false;
        bool is_scanning_up = false;
        Vertex vl_up, vr_up;
        Vertex vl_down, vr_down;
        while (ix < m_cell_count.x && iy < m_cell_count.y)
        {
            if (is_scanning_down && !isWall(Direction::LeftDown, ix, iy))
            {
                is_scanning_down = false;
                vr_up = {ix, iy};
                edges.push_back({vl_up, vr_up});
            }
            if (!is_scanning_down && isWall(Direction::LeftDown, ix, iy))
            {
                vl_up = {ix, iy};
                is_scanning_down = true;
            }

            if (is_scanning_up && !isWall(Direction::RightUp, ix, iy))
            {
                is_scanning_up = false;
                vr_down = {ix, iy};
                edges.push_back({vr_down, vl_down});
            }
            if (!is_scanning_up && isWall(Direction::RightUp, ix, iy))
            {
                is_scanning_up = true;
                vl_down = {ix, iy};
            }
            ix++;
            iy++;
        }
    }

    //! scan diagonally form left down to right up
    for (int i_diag = 0; i_diag < i_diag_max; ++i_diag)
    {
        int ix = std::max(0, i_diag - m_cell_count.y);
        int iy = std::min(i_diag, m_cell_count.y - 1);

        bool is_scanning_down = false;
        bool is_scanning_up = false;
        Vertex vl_up, vr_up;
        Vertex vl_down, vr_down;
        while (ix < m_cell_count.x && iy > 0)
        {
            if (is_scanning_down && !isWall(Direction::LeftUp, ix, iy))
            {
                is_scanning_down = false;
                vr_up = {ix, iy + 1};
                edges.push_back({vl_up, vr_up});
            }
            if (!is_scanning_down && isWall(Direction::LeftUp, ix, iy))
            {
                vl_up = {ix, iy + 1};
                is_scanning_down = true;
            }

            if (is_scanning_up && !isWall(Direction::RightDown, ix, iy))
            {
                is_scanning_up = false;
                vr_down = {ix, iy + 1};
                edges.push_back({vr_down, vl_down});
            }
            if (!is_scanning_up && isWall(Direction::RightDown, ix, iy))
            {
                is_scanning_up = true;
                vl_down = {ix, iy + 1};
            }
            ix++;
            iy--;
        }
    }

    return edges;
}

bool MapGridDiagonal::isAtBoundary(int ix, int iy) const
{
    return ix >= m_cell_count.x - 1 || iy >= m_cell_count.y - 1 ||
           ix < 1 || iy < 1;
}

void MapGridDiagonal::extractBoundaries()
{
    m_boundaries.clear();
    for (int j = 1; j < m_cell_count.y - 1; ++j)
    {
        for (int i = 1; i < m_cell_count.x - 1; ++i)
        {
            const auto cell_index = j * m_cell_count.x + i;

            const auto t_left = m_tiles[cell_index + deltaInd(Direction::Left)];
            const auto t_right = m_tiles[cell_index + deltaInd(Direction::Right)];
            const auto t_up = m_tiles[cell_index + deltaInd(Direction::Up)];
            const auto t_down = m_tiles[cell_index + deltaInd(Direction::Down)];
            auto &t = m_tiles[cell_index];
            int n_walls_around = (t_left == Tile::Wall) + (t_right == Tile::Wall) +
                                 (t_down == Tile::Wall) + (t_up == Tile::Wall);
            if (t != Tile::Ground)
            {
                if (n_walls_around == 3)
                {
                    bool is_up = t_up == Tile::Ground;
                    bool is_left = t_left == Tile::Ground;
                    bool is_down = t_down == Tile::Ground;
                    bool is_right = t_right == Tile::Ground;
                    if (is_up)
                    {
                        m_boundaries[cell_index].dir = Direction::Up;
                    }
                    if (is_left)
                    {
                        m_boundaries[cell_index].dir = Direction::Left;
                    }
                    if (is_down)
                    {
                        m_boundaries[cell_index].dir = Direction::Down;
                    }
                    if (is_right)
                    {
                        m_boundaries[cell_index].dir = Direction::Right;
                    }
                }
            }
        }
    }
    for (int j = 1; j < m_cell_count.y - 1; ++j)
    {
        for (int i = 1; i < m_cell_count.x - 1; ++i)
        {
            const auto cell_index = j * m_cell_count.x + i;

            const auto t_left = m_tiles[cell_index + deltaInd(Direction::Left)];
            const auto t_right = m_tiles[cell_index + deltaInd(Direction::Right)];
            const auto t_up = m_tiles[cell_index + deltaInd(Direction::Up)];
            const auto t_down = m_tiles[cell_index + deltaInd(Direction::Down)];
            auto &t = m_tiles[cell_index];
            int n_walls_around = (t_left == Tile::Wall) + (t_right == Tile::Wall) +
                                 (t_down == Tile::Wall) + (t_up == Tile::Wall);
            if (t != Tile::Ground)
            {

                if (n_walls_around == 2)
                {
                    bool is_ul_corner = t_left == Tile::Ground and t_up == Tile::Ground;
                    bool is_ur_corner = t_right == Tile::Ground and t_up == Tile::Ground;
                    bool is_dl_corner = t_left == Tile::Ground and t_down == Tile::Ground;
                    bool is_dr_corner = t_right == Tile::Ground and t_down == Tile::Ground;

                    if (is_ul_corner)
                    {
                        m_boundaries[cell_index].dir = Direction::LeftUp;
                    }
                    if (is_ur_corner)
                    {
                        m_boundaries[cell_index].dir = Direction::RightUp;
                    }
                    if (is_dl_corner)
                    {
                        m_boundaries[cell_index].dir = Direction::LeftDown;
                    }
                    if (is_dr_corner)
                    {
                        m_boundaries[cell_index].dir = Direction::RightDown;
                    }
                }
            }
        }
    }

    for (auto [cell_ind, data] : m_boundaries)
    {
    }
}

void MapGridDiagonal::transformCorners()
{
    m_boundaries.clear();

    for (int iy = 1; iy < m_cell_count.y - 1; ++iy)
    {
        for (int ix = 1; ix < m_cell_count.x; ++ix)
        {
            auto cell_index = cellIndex(ix, iy);
            const auto t_left = m_tiles[cell_index + deltaInd(Direction::Left)];
            const auto t_right = m_tiles[cell_index + deltaInd(Direction::Right)];
            const auto t_up = m_tiles[cell_index + deltaInd(Direction::Up)];
            const auto t_down = m_tiles[cell_index + deltaInd(Direction::Down)];
            auto &t = m_tiles[cell_index];
            int n_walls_around = (t_left == Tile::Wall) + (t_right == Tile::Wall) +
                                 (t_down == Tile::Wall) + (t_up == Tile::Wall);
            if (t != Tile::Ground)
            {

                if (n_walls_around == 2)
                {
                    bool is_ul_corner = t_left == Tile::Ground and t_up == Tile::Ground;
                    bool is_ur_corner = t_right == Tile::Ground and t_up == Tile::Ground;
                    bool is_dl_corner = t_left == Tile::Ground and t_down == Tile::Ground;
                    bool is_dr_corner = t_right == Tile::Ground and t_down == Tile::Ground;

                    if (is_ul_corner)
                    {
                        m_boundaries[cell_index].dir = Direction::LeftUp;
                    }
                    if (is_ur_corner)
                    {
                        m_boundaries[cell_index].dir = Direction::RightUp;
                    }
                    if (is_dl_corner)
                    {
                        m_boundaries[cell_index].dir = Direction::LeftDown;
                    }
                    if (is_dr_corner)
                    {
                        m_boundaries[cell_index].dir = Direction::RightDown;
                    }
                }
            }

            if (n_walls_around == 3)
            {
                bool is_up = t_up == Tile::Ground;
                bool is_left = t_left == Tile::Ground;
                bool is_down = t_down == Tile::Ground;
                bool is_right = t_right == Tile::Ground;
                if (is_up)
                {
                    m_boundaries[cell_index].dir = Direction::Up;
                }
                if (is_left)
                {
                    m_boundaries[cell_index].dir = Direction::Left;
                }
                if (is_down)
                {
                    m_boundaries[cell_index].dir = Direction::Down;
                }
                if (is_right)
                {
                    m_boundaries[cell_index].dir = Direction::Right;
                }
            }
        }
    }
}