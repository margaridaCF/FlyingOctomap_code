#ifndef RESULT_SET_H
#define RESULT_SET_H

namespace LazyThetaStarOctree{
class ResultSet
    {
    public:
        // == VARIABLES ==
        std::map <double, int> cellVoxelDistribution;
        int iterations_used;
        ResultSet()
        : iterations_used (0)
            {}
        ///  Operators
        void addOcurrance(const double cellVoxel)
        {
            cellVoxelDistribution[cellVoxel]++;
        }
        double getSizeOfLargestVoxel() const
        {
            auto rit = cellVoxelDistribution.crbegin();
            return rit->first;
        }
        

        ///  Display  and  <<
        std::string displayString() const
        {
            return "Used " + std::to_string(iterations_used) + " iterations finding voxels as big as " + std::to_string(getSizeOfLargestVoxel());
        }
        std::ostream& displayString(std::ostream& stream_out) const
        {
          stream_out << displayString() ;
          return stream_out;
        }
    };
}
#endif // RESULT_SET_H