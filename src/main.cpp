#include <unordered_set>
#include <vector>
#include <list>
#include <iostream>
#include <format>
#include <fstream>

#include <boost/xpressive/xpressive.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace xp = boost::xpressive;

struct Cell
{
    size_t y;
    size_t x;
    Cell(size_t y, size_t x) : y(y), x(x) {}

    bool operator==(const Cell & c) const {
        return x == c.x && y == c.y;
    }
};

// Hash functor object for us of Cell in std::unordered_set
namespace std {
  template <>
  struct hash<Cell> {
    size_t operator()(const Cell& c) const {
      size_t h1 = std::hash<int>{}(c.x);
      size_t h2 = std::hash<int>{}(c.y);
      return h1 ^ (h2 << 1);
    }
  };
}

typedef std::vector<std::vector<int>> intmatrix_t;
typedef std::vector<Cell> celllist_t;
typedef std::unordered_set<Cell> cellset_t;

// This test function creates a vector of vector of ints, sets all of them to -1, 
// and given an input list of which cells should be positive, sets only those cells
// to a positive value
intmatrix_t generateMatrix(size_t rows, size_t columns, const celllist_t & positiveCells)
{
    intmatrix_t matrix(rows);
    for (auto & row : matrix) {
        row.resize(columns);
        std::fill(row.begin(), row.end(), -1);
    }
    for (auto cell : positiveCells) {
        matrix[cell.y][cell.x] = 1;
    }
    return matrix;
}

// This helper function prints to the console a vector of vector of ints with 
// positive values shown as + and negative values as -
void printMatrix(const intmatrix_t & matrix) 
{
    for (const auto & row : matrix) {
        for (const auto & col : row) {
            if (col > 0) std::cout << "+ ";
            else std::cout << "- ";
        }
        std::cout << std::endl;
    }
}

// This helper function prints to the console a vector of vector of ints with 
// positive values shown as +, negative values as -, and neighbor cells as 0
void printMatrixNeighborOverlay(const intmatrix_t & matrix, const cellset_t & neighbors) 
{
    for (size_t y = 0; y < matrix.size(); y++) {
        for (size_t x = 0; x < matrix[y].size(); x++) {
            if (matrix[y][x] > 0) std::cout << "+ ";
            else if (neighbors.contains(Cell(y,x))) std::cout << "O ";
            else std::cout << "- ";
        }
        std::cout << std::endl;
    }
}

// This function walks through a vector of vector of ints and returns an
// unordered set with the location of only positive cells
cellset_t findPositiveCells(const intmatrix_t & matrix)
{
    cellset_t positiveCells;
    for (size_t y = 0; y < matrix.size(); y++) {
        for (size_t x = 0; x < matrix[y].size(); x++) {
            if (matrix[y][x] > 0) {
                positiveCells.emplace(Cell(y,x));
            }
        }
    }
    return positiveCells;
}

// Given a width and height and starting position, adds to an unordered
// set of neighbor cells any cell within distanceThreshold of the start
// cell.  (Manhattan distance)  The number of visited cells is returned
// for auditing algorithm complexity.
size_t findNeighborCells(size_t width, size_t height, const Cell & start, cellset_t & neighborCells, size_t distanceThreshold)
{
    size_t visits = 0;      // counter for algorithm complexity
    std::list<std::pair<Cell, size_t>> queue;   // queue of remaining cells to check, and its distance from start

    // seed the queue with the starting cell
    queue.push_back(std::make_pair(start, distanceThreshold));
    do {
        // get the first cell and distance from the queue
        auto & cell = queue.front().first;
        int distance = queue.front().second;

        // if this cell has not been visited, and we still have moves left, move up/down/right/left
        if (!neighborCells.contains(cell) && distance >= 0) {
            visits++;
            neighborCells.emplace(cell); // this is a neighbor cell

            // now move to each adjacent cell, assuming that adjacent cell fits within width/height
            if (cell.y > 0) queue.push_back(std::make_pair(Cell(cell.y-1,cell.x), distance-1)); // up
            if (cell.x > 0) queue.push_back(std::make_pair(Cell(cell.y,cell.x-1), distance-1)); // left
            if (cell.y < height-1) queue.push_back(std::make_pair(Cell(cell.y+1,cell.x), distance-1)); // down
            if (cell.x < width-1) queue.push_back(std::make_pair(Cell(cell.y,cell.x+1), distance-1)); // right
        }
        queue.pop_front();
    } while (!queue.empty());
    return visits;
}

int main(int argc, char** argv)
{
    //
    // Parse program options for input file
    //
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Produce help message")
        ("testfile", po::value<std::string>(), "Path to test file")
    ;

    po::positional_options_description p;
    p.add("testfile", -1);
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
              options(desc).positional(p).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    if (vm.count("testfile")) {
        std::cout << "Using test file: " 
     << vm["testfile"].as<std::string>() << ".\n";
    } else {
        std::cout << "No test file specified.\n";
        return 1;
    }

    // Just a helper lambda function to clean up the code
    auto starLineSeparator = []() {
        std::cout << std::setfill('*') << std::setw(80) << "" << std::endl;
    };

    //
    // Parse the input stream
    //
    std::ifstream infile(vm["testfile"].as<std::string>());

    // Line should be of the form:
    // {TEST_IDENTIFIER};{HEIGHT};{WIDTH};{DISTANCE};{POINTS};{EXPECTED_RESULT}
    //
    // {TESTIDENTIFIER - string}
    // {HEIGHT - uint}
    // {WIDTH - uint}
    // {DISTANCE - uint}
    // {POINTS - Comma separated list of (y,x) values}
    // {EXPECTED_RESULT - uint}
    xp::sregex rx = xp::sregex::compile("(?P<identifier>.*);(?P<height>\\d*);(?P<width>\\d*);(?P<distance>\\d*);(?P<points>.*);(?P<expected>\\d*)");
    xp::sregex point_rx = xp::sregex::compile("\\((?P<y>\\d*),(?P<x>\\d*)\\)");
    int linenum = 0; // only needed to display malformed line number
    std::string line;
    while (std::getline(infile, line)) {
        linenum++;
        if (line.empty() || line.starts_with("#")) continue;

        // Get the parameters for this test
        xp::smatch what;
        if (xp::regex_search(line, what, rx)) {
            // We have a valid line, parse it
            std::string identifier = what["identifier"].str();
            std::string points = what["points"].str();
            size_t height = std::stoul(what["height"].str());
            size_t width = std::stoul(what["width"].str());
            size_t distance = std::stoul(what["distance"].str());
            size_t expected = std::stoul(what["expected"].str());

            celllist_t pointList;
            xp::sregex_iterator it(points.begin(), points.end(), point_rx);
            xp::sregex_iterator end;
            while(it != end) {
                pointList.push_back(Cell(std::stoul((*it)["y"]),std::stoul((*it)["x"])));
                it++;
            }

            // Create an height x width vector of vector of ints, setting all cells to -1 
            // except those in pointList
            intmatrix_t matrix = generateMatrix(height, width, pointList);

            // Display to the console the nature of this test
            starLineSeparator();
            std::cout << "TEST NAME: " << identifier << std::endl;
            std::cout << std::format("Size ({}x{}), N={}", height, width, distance) << std::endl;
            std::cout << "Points: [" << points << "]" << std::endl;
            starLineSeparator();

            // Find the positive cells in a given matrix (O(n) complexity)
            cellset_t positiveCells = findPositiveCells(matrix);
            cellset_t neighborCells;
            size_t totalVisits = 0;

            // For each positive cell, gather its neighbors into an unordered set, then transfer
            // the local neighbors to this cell into the master neighbor set, which is a union of
            // all of the sets
            for (const auto & cell : positiveCells) {
                if (matrix.empty()) break;      // no need to do anything!
                cellset_t localNeighborCells;
                totalVisits += findNeighborCells(matrix[0].size(), matrix.size(), cell, localNeighborCells, distance);
                neighborCells.insert(localNeighborCells.begin(), localNeighborCells.end());
            }

            // Display the result on the console
            printMatrixNeighborOverlay(matrix, neighborCells);
            starLineSeparator();
            std::cout << std::format("Expected Count: {}, Actual Count: {}", expected, neighborCells.size()) << std::endl;
            std::cout << std::format("Count of Visited Cells: {}", totalVisits) << std::endl;
            std::cout << std::format("Result: {}", (expected == neighborCells.size() ? "PASS" : "FAIL")) << std::endl;
        }
        else {
            std::cout << "Error: Badly Formatted Line - line " << linenum << std::endl;
        }

        starLineSeparator();
        std::cout << std::endl;
    }

    return 0;
}
