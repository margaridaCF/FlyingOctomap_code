#include <gtest/gtest.h>
#include <frontiers.h>
#include <neighbors.h>

namespace Frontiers_test
{
	TEST(CirculatorTest, FullIteration)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		octomath::Vector3 min (0, 0, 0);
		octomath::Vector3 max (1, 1, 1);

		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with write_volume_explored_to_csv");
        }
		octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
		double count = 0;
    	while(it != octree.end_leafs_bbx() )
		{
			count++;
			it++;
		}

		Frontiers::Circulator cit (octree, max, min, 0);
		double count_circulator = 0;
		bool failed = false;
		while(!cit.isFinished() && !failed )
		{
			count_circulator++;
			cit.increment();
			failed = (count_circulator > (count+10) );
		}
		ASSERT_TRUE(cit.isFinished());
		ASSERT_FALSE(failed);
		ASSERT_EQ(count, count_circulator);
	}


	TEST(CirculatorTest, FullIteration_FromOffset)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		octomath::Vector3 min (0, 0, 0);
		octomath::Vector3 max (1, 1, 1);

		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with write_volume_explored_to_csv");
        }
		octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
		double count = 0;
    	while(it != octree.end_leafs_bbx() )
		{
			count++;
			it++;
		}

		Frontiers::Circulator cit (octree, max, min, 2);
		double count_circulator = 0;
		bool failed = false;
		while(!cit.isFinished() && !failed )
		{
			count_circulator++;
			cit.increment();
			failed = (count_circulator > (count+3) );
		}
		ASSERT_TRUE(cit.isFinished());
		ASSERT_FALSE(failed);
		ASSERT_EQ(count, count_circulator);
	}

	TEST(CirculatorTest, FullIteration_OffsetGreaterThanVoxelsCount)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		octomath::Vector3 min (0, 0, 0);
		octomath::Vector3 max (1, 1, 1);

		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with write_volume_explored_to_csv");
        }
		octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
		double count = 0;
    	while(it != octree.end_leafs_bbx() )
		{
			count++;
			it++;
		}

		Frontiers::Circulator cit (octree, max, min, 102);
		double count_circulator = 0;
		bool failed = false;
		while(!cit.isFinished() && !failed )
		{
			count_circulator++;
			cit.increment();
			failed = (count_circulator > (count+3) );
		}
		ASSERT_TRUE(cit.isFinished());
		ASSERT_FALSE(failed);
		ASSERT_EQ(count, count_circulator);
	}
	TEST(CirculatorTest, FullIteration_OffsetGreaterThanVoxelsCount_Double)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		octomath::Vector3 min (0, 0, 0);
		octomath::Vector3 max (1, 1, 1);

		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with write_volume_explored_to_csv");
        }
		octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
		double count = 0;
    	while(it != octree.end_leafs_bbx() )
		{
			count++;
			it++;
		}

		Frontiers::Circulator cit (octree, max, min, 300);
		double count_circulator = 0;
		bool failed = false;
		while(!cit.isFinished() && !failed )
		{
			count_circulator++;
			cit.increment();
			failed = (count_circulator > (count+3) );
		}
		ASSERT_TRUE(cit.isFinished());
		ASSERT_FALSE(failed);
		ASSERT_EQ(count, count_circulator);
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}