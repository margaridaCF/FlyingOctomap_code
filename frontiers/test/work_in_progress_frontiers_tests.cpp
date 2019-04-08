#include <gtest/gtest.h>
#include <next_best_view.h>
#include <frontiers.h>
#include <neighbors.h>

namespace Frontiers
{
	void printForMatlab(std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors)
	{
		std::cout << " x_values = [ ];\n";
		std::cout << " y_values = [ ];\n";
		std::cout << " z_values = [ ];\n";
		for (std::shared_ptr<octomath::Vector3> n : neighbors)
		{
			std::cout << " x_values = [x_values, " << n->x() << "];\n ";
			std::cout << " y_values = [y_values, " << n->y() << "];\n ";
			std::cout << " z_values = [z_values, " << n->z() << "];\n ";
		}
	}


}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}