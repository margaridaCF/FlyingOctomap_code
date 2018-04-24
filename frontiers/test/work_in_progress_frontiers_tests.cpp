#include <gtest/gtest.h>
#include <frontiers.h>
#include <neighbors.h>

namespace Frontiers
{

	bool isCenterGoodGoal(double voxel_side, double sensing_distance)
    {
        return (voxel_side/sensing_distance) <= 1;
    }

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}