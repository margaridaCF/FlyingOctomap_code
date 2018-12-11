#ifndef VOXEL_H
#define VOXEL_H

namespace LazyThetaStarOctree{
class Voxel
    {
    public:
        octomath::Vector3 coordinates;
        double size;
        Voxel()
            : size(0)
            {}
        Voxel(octomath::Vector3 coordinates, double size)
            : coordinates(coordinates), size(size)
            {}
        
        ///  Operators
        bool operator==(Voxel const& otherVoxel) const
        {
            return (otherVoxel.coordinates == coordinates && size == otherVoxel.size);
        }


        octomath::Vector3 getCoordinates() const
        {
            return coordinates;
        }
        void setCoordinates(octomath::Vector3 coordinates)
        {
            coordinates = coordinates;
        }
        bool equalCoordinatesWithErrorMargin(Voxel otherVoxel, double error)
        {
            if(coordinates.distance(otherVoxel.coordinates) > error)
            {
                return false;
            }
            return true;
        }

        // ///  Display  and  <<
        // std::string displayString() const
        // {
        //   return "" << coordinates << " size " << size;
        // }
        // std::ostream& displayString(std::ostream& stream_out) const
        // {
        //   stream_out << coordinates ;
        //   stream_out.precision(3);
        //   return stream_out;
        // }
    };
}
#endif // VOXEL_H
