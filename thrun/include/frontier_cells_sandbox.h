#ifndef INFORMATION_GAIN_H
#define FRONTIER_CELLS_SANDBOX_H

#include <point2d_lib.h>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <qdbmp.h>
#include <octomap/OcTree.h>
#include <octomap/math/Vector3.h>
#include <chrono>
#include <algorithm>


namespace mapper
{
    enum Directions { twoD=4, threeD=6 };

    class Voxel
    {
    public:
        double x, y, z, size;
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
        ResultSet(int cellItertions, float scenarioArea, int frontierCellsFound, 
            float frontierArea, float executionTime,
            std::string scenario, std::string algorithm)
            : cellItertions(cellItertions), scenarioArea(scenarioArea), 
              frontierCellsFound(frontierCellsFound), 
              frontierArea(frontierArea), executionTime(executionTime),
              scenario(scenario), algorithm(algorithm)
        {}
        const int cellItertions;
        const float scenarioArea;
        const int frontierCellsFound;
        const float frontierArea;
        const float executionTime;
        const std::string scenario;
        const std::string algorithm;
    };


    enum Scenario {circle, double_circle, s, oil, oil_larger};
    enum Algorithm {regularGrid_2d, regularGrid_3d, sparseGrid_2d, sparseGrid_3d};
    typedef std::map<Scenario, ResultSet> AlgorithmResults;

    class ResultSecretary
    {
    public:
        ResultSecretary() : resultStorage({}){};
        void storeAlgorithmResults(Scenario scenario, Algorithm algorithm, ResultSet);
        void exportCsv() const;
    private:
        std::map<Algorithm, AlgorithmResults> resultStorage;
    };

    std::ostream& operator<<(std::ostream& s, const Voxel& c){
        return c.displayString(s);
    }
    
    class FrontierCellsSandbox
    {
    public:
    	const int width = 0;
        const int height = 0;
        const Scenario scenario;
        const std::string scenarioName;
        FrontierCellsSandbox(Scenario scenario, std::string scenarioName);
        ~FrontierCellsSandbox(){};
        bool findFrontierCells(octomath::Vector3 max, octomath::Vector3 min, Directions directions);
        std::list<Voxel> const& getFrontierCells() const;
        void drawWorld(std::string const& fileName, octomath::Vector3 max, octomath::Vector3 real_min, 
            float z_level, bool printFrontierCells) const;
        virtual bool isOccupied(octomath::Vector3 const& grid_coordinates_toTest) const = 0;
        virtual bool isExplored(octomath::Vector3 const& grid_coordinates_toTest) const = 0;
        void exportCsv() const;
        double calculateCoverage(double side, Directions dimensions);

    protected:
        virtual float  incrementIteration() const = 0;
        std::list<Voxel> frontierCells;
        const std::array<octomath::Vector3, 6> rayDirections;
        std::set<float> z_levels;
        ResultSecretary storage;
    };

    class OctomapFrontierCellsSandbox : public FrontierCellsSandbox
    {
    public:
        OctomapFrontierCellsSandbox(Scenario scenario, std::string scenarioName, std::string data_file); 
        bool isOccupied(octomath::Vector3 const& grid_coordinates_toTest) const;
        bool isExplored(octomath::Vector3 const& grid_coordinates_toTest) const;
        bool findFrontierCells_sparseIteration(octomath::Vector3 & bbxMax, octomath::Vector3 & min, Directions directions);
        void getBounds(std::tuple<double, double, double> & min, std::tuple<double, double, double> & max);
        int  drawAllLevels(std::string dataset_name, octomath::Vector3 max, octomath::Vector3 real_min);
    protected:
        float  incrementIteration() const;
        octomap::OcTree octree;
    };
}


#endif // FRONTIER_CELLS_SANDBOX_H