#pragma once

#include "Graph.hpp"

namespace mtrn3100 {

// Converts a maze string to a graph that uses indexes for nodes.
Graph<int, int> ascii2graph(char* maze) {
    Graph<int, int> graph;

    // Find height of maze by counting number of \n.
    int numCharRows = 0;
    for (int i = 0; maze[i] != '\0'; i++) {
        if (maze[i] == '\n') {
            numCharRows++;
        }
    }
    numCharRows++;

    // Find width of maze by counting number of chars until \n is found.
    int numCharCols = 0;
    for (int i = 0; maze[i] != '\n'; i++) {
        numCharCols++;
    }
    numCharCols++;

    // Helper function to convert between (rows, cols) and number of chars.
    auto charRows2row = [](int const r) -> int { return ceil((r - 1) / 2.0); };
    auto charCols2col = [](int const c) -> int { return ceil((c - 2) / 4.0); };

    // Get maze size.
    int numRows = charRows2row(numCharRows);
    int numCols = charCols2col(numCharCols);

    // Helper function to access the maze with a 2D model.
    auto maze2d = [&maze, numCharCols](unsigned const row, unsigned const col) -> char& {
        return maze[row * numCharCols + col];
    };

    // Helper function to convert node indexes to node positions.
    auto index2pos = [numRows, numCols](int const index) -> mtrn3100::Tuple<int, int> {
        return {floor((index - 1) / numRows), (index - 1) % numCols};
    };

    // Helper function to convert node positions to node indexes.
    auto pos2index = [numRows, numCols](int const row, int const col) -> int { return row * numCols + col + 1; };

    for (auto charRow = 0; charRow < numCharRows; charRow++) {
        for (auto charCol = 0; charCol < numCharCols; charCol++) {
            // At centre of tile.
            if ((charCol + 2) % 4 == 0 && charRow % 2 != 0) {
                auto const r = charRows2row(charRow);
                auto const c = charCols2col(charCol);
                int mazePos = pos2index(r, c);

                // Check vertical wall to right of centre of tile.
                if (c + 1 < numCols) {
                    if (maze2d(charRow, charCol + 2) == ' ') {
                        int rightCell = pos2index(r, c + 1);
                        graph.insert_node(mazePos);
                        graph.insert_node(rightCell);
                        graph.insert_edge(mazePos, rightCell, 0);
                        graph.insert_edge(rightCell, mazePos, 0);
                    }
                }

                // Check horizontal wall below centre of tile.
                if (r + 1 < numRows) {
                    if (maze2d(charRow + 1, charCol) == ' ') {
                        int belowCell = pos2index(r + 1, c);
                        graph.insert_node(mazePos);
                        graph.insert_node(belowCell);
                        graph.insert_edge(mazePos, belowCell, 0);
                        graph.insert_edge(belowCell, mazePos, 0);
                    }
                }
            }
        }
    }

    return graph;
}

}  // namespace mtrn3100
