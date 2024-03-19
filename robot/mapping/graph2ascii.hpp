#pragma once

#include "Graph.hpp"
#include "Tuple.hpp"
#include "utilities.hpp"
#include "LinkedList.hpp"
// #include <iostream>
// #include <tuple>
// #include <variant>

namespace mtrn3100
{

    // COMPLETE THIS FUNCTION.
    template <typename N, typename E>
    char *graph2ascii(Graph<N, E> const &g)
    {
        // 3 x 3 maze.
        int const numRows = 5;
        int const numCols = 9;

        // Helper function to convert between (rows, cols) and number of chars.
        auto row2charPos = [](int const r)
        { return r * 2 + 1; };
        auto col2charPos = [](int const c)
        { return c * 4 + 2; };

        int const numCharRows = row2charPos(numRows);     // 7 characters.
        int const numCharCols = col2charPos(numCols);     // 14 characters.
        char *maze = new char[numCharCols * numCharRows]; // 98 bytes is needed to represent 3 x 3 maze.

        // Helper function to access the maze with a 2D model.
        auto maze2d = [&maze, &numCharCols](unsigned const r, unsigned const c) -> char &
        {
            return maze[r * numCharCols + c];
        };

        // Initialise the maze values.
        for (int i = 0; i < numCharCols * numCharRows; i++)
        {
            maze[i] = ' ';
        }

        // Do new lines.
        for (int r = 0; r < numCharRows; r++)
        {
            maze2d(r, numCharCols - 1) = '\n';
        }

        // Terminate the string.
        maze2d(numCharRows - 1, numCharCols - 1) = '\0';

        // Do external walls top and bottom.
        for (int c = 0; c < numCharCols - 1; c++)
        {
            if (c % 4 != 0)
            {
                maze2d(0, c) = '-';
                maze2d(numCharRows - 1, c) = '-';
            }
        }

        // Do external walls left and right.
        for (int r = 0; r < numCharRows - 1; r++)
        {
            if (r % 2 != 0)
            {
                maze2d(r, 0) = '|';
                maze2d(r, numCharCols - 2) = '|';
            }
        }

        // Do internal vertical walls.
        int node = 1;
        for (int r = 1; r < numCharRows; r = r + 2)
        {
            for (int c = 4; c < 9; c = c + 4)
            {
                if (!g.is_connected(node, node + 1))
                {
                    maze2d(r, c) = '|';
                }
                node++;
            }
            node++;
        }

        // Do internal horizontal walls.
        node = 1;
        for (int r = 2; r < numCharRows; r = r + 2)
        {
            for (int c = 1; c < numCharCols - 1; c = c + 4)
            {
                if (!g.is_connected(node, node + 3))
                {
                    maze2d(r, c) = '-';
                    maze2d(r, c+1) = '-';
                    maze2d(r, c+2) = '-';
                }
                node++;
            }
        }

        // Provided but I don't use it
        // for (auto const& edge : g.edges()) {
        // }

        // For debugging. Don't forget to include iostream.
        // std::cout << maze << std::endl;

        return maze;
    }

} // namespace mtrn3100