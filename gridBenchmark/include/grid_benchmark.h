#ifndef INFORMATION_GAIN_H
#define GRID_BENCHMARK_H

#include <vector>
#include <cmath>
#include <sstream>

//#include <math.h>
#include <ros/ros.h>
#include <qdbmp.h>
#include <octomap/OcTree.h>
#include <octomap/math/Vector3.h>
#include <chrono>
#include <algorithm>


namespace mapper
{
    enum Directions { twoD=4, threeD=6 };
    enum Scenario {circle, double_circle, s, oil, oil_larger, experimental};
    enum Algorithm {regularGrid_2d, regularGrid_3d, sparseGrid_2d, sparseGrid_3d};

    class Voxel
    {
    public:
        double x, y, z, size;
        Voxel()
            : x(0), y(0), z(0), size(0)
            {}
        Voxel(double x, double y, double z, double size)
            : x(x), y(y), z(z), size(size)
            {}
        bool isInZlevel(float z_level) const
        {
            float max_z = z + (size/2); 
            float min_z = z -(size/2);
            if(max_z >= z_level && min_z <= z_level)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        ///  Operators
        bool operator==(Voxel const& otherVoxel) const
        {
            return (x == otherVoxel.x && y == otherVoxel.y 
                && z == otherVoxel.z && size == otherVoxel.size);
        }

        ///  Display  and  <<
        std::string displayString() const
        {
          return "(" + std::to_string(x) + "; "+ std::to_string(y) + " )";
        }
        std::ostream& displayString(std::ostream& stream_out) const
        {
          stream_out << "(" << x << "; " << y << "; "<< z << " ) x "<<size ;
          stream_out.precision(3);
          return stream_out;
        }
    };

    class ResultSet
    {
    public:
        ResultSet()
            : cellIterations(0), scenarioArea(0), 
              frontierCellsFound(0), 
              frontierArea(0), executionTime(0),
              scenario(circle), algorithm(regularGrid_2d)
        {};
        ResultSet(int cellIterations, float scenarioArea, int frontierCellsFound, 
            float frontierArea, float executionTime,
            Scenario scenario, Algorithm algorithm)
            : cellIterations(cellIterations), scenarioArea(scenarioArea), 
              frontierCellsFound(frontierCellsFound), 
              frontierArea(frontierArea), executionTime(executionTime),
              scenario(scenario), algorithm(algorithm)
        {};
        bool compareResults(ResultSet const& newResults, bool print = true) const;
        const int cellIterations;
        const float scenarioArea;
        const int frontierCellsFound;
        const float frontierArea;
        const float executionTime;
        const Scenario scenario;
        const Algorithm algorithm;
    };


    typedef std::map<Scenario, ResultSet> AlgorithmResults;

    class ResultSecretary
    {
    public:
        ResultSecretary() : resultStorage({}){};
        void storeAlgorithmResults(ResultSet result);
        void exportCsv(std::list<Scenario> scenarioOrder, bool exportTables, std::string fileName) const;
        static std::string translateAlgorithmName(Algorithm algorithm) ;
        ResultSet const& getResult(Scenario scenario, Algorithm algorithm) const;
        void insertIterationCalculations(std::ofstream & myfile , 
            int frontierCells, int totalCells, int sparseCells) const;
    private:
        void exportForTables(std::ofstream & myfile) const;
        std::map<Algorithm, AlgorithmResults> resultStorage;
    };

    std::ostream& operator<<(std::ostream& s, const Voxel& c){
        return c.displayString(s);
    }
    
///  ====================================================================

    class AlgorithmObject
    {
    public:
        AlgorithmObject(std::string algorithmName, Algorithm algorithmEnum, octomap::OcTree& octree, float grid_res);
        ~AlgorithmObject(){};
        virtual void iteratorNext() = 0; 
        virtual bool iteratorInit(octomath::Vector3 min, octomath::Vector3 max) = 0; 
        virtual bool iteratorEndReached() const = 0; 
        virtual octomath::Vector3 calculateNeighbor(octomath::Vector3 grid_coordinates_curr, 
            octomath::Vector3 direction, double voxel_size) const = 0;   
        virtual void getCurrentCell(Voxel& cell) const = 0;     
        void setBoundingBox(octomath::Vector3 min, octomath::Vector3 max);
        octomath::Vector3 const& getBoundingBoxMin() const;
        octomath::Vector3 const& getBoundingBoxMax() const;
        double calculateCoverage(double side, Directions dimensions);
        
        const float grid_res;
        const std::string algorithmName;
        const Algorithm algorithmEnum;
    protected:
        octomap::OcTree& octree;
        octomath::Vector3 bbx_min, bbx_max;
    };

    class RegularGrid : public AlgorithmObject
    {
    public:
        RegularGrid(std::string algorithmName, Algorithm algorithmEnum, octomap::OcTree& octree, float grid_res);
        ~RegularGrid() {};
        void iteratorNext(); 
        bool iteratorInit(octomath::Vector3 min, octomath::Vector3 max); 
        bool iteratorEndReached() const; 
        octomath::Vector3 calculateNeighbor(octomath::Vector3 grid_coordinates_curr, 
            octomath::Vector3 direction, double voxel_size) const;
        void getCurrentCell(Voxel& cell) const;
    private:
        float x, y, z;  // iterator
    };

    class SparseGrid : public AlgorithmObject
    {
    public:
        SparseGrid(std::string algorithmName, Algorithm algorithmEnum, octomap::OcTree& octree, float grid_res);
        ~SparseGrid(){};
        void iteratorNext(); 
        bool iteratorInit(octomath::Vector3 min, octomath::Vector3 max); 
        bool iteratorEndReached() const; 
        octomath::Vector3 calculateNeighbor(octomath::Vector3 grid_coordinates_curr, 
            octomath::Vector3 direction, double voxel_size) const;
        void getCurrentCell(Voxel& cell) const;
    private:
        octomap::OcTree::leaf_bbx_iterator it;
    };


    class GridBenchmark
    {
    public:
        GridBenchmark(Scenario scenario, std::string scenarioName, octomap::OcTree& octree);
        ~GridBenchmark(){};
        ResultSet findFrontierCells(octomath::Vector3  max, octomath::Vector3  min, Directions directions,
            AlgorithmObject & algorithmSpecification);
        bool isExplored(octomath::Vector3 const& grid_coordinates_toTest) const;
        bool isOccupied(octomath::Vector3 const& grid_coordinates_toTest) const;
        int  drawAllLevels(std::string dataset_name, octomath::Vector3 max, 
            octomath::Vector3 real_min);
        void getBounds(std::tuple<double, double, double> & min, std::tuple<double, double, double> & max);
        int getFrontierNumber();
        std::list<Voxel> const& getFrontierCells() const;
        std::list<Voxel> frontierCells;
    private:
        void drawWorld(std::string const& fileName, octomath::Vector3 max, octomath::Vector3 real_min, 
            float z_level, bool printFrontierCells) const;
        octomap::OcTree & octree;
        std::set<float> z_levels;
        const std::array<octomath::Vector3, 6> rayDirections;
        const Scenario scenario;
        const std::string scenarioName;
    };
}


#endif // GRID_BENCHMARK_H