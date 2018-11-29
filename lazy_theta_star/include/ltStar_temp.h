#include <ltStarOctree_common.h>
#include <neighbors.h>
#include <voxel.h>
#include <open.h>
#include <list>
#include <unordered_map>
#include <lazy_theta_star_msgs/LTStarRequest.h>
#include <lazy_theta_star_msgs/LTStarReply.h>
#include <lazy_theta_star_msgs/LTStarNodeStatus.h>

// namespace std
// {
//     template <>
//     struct hash<octomath::Vector3>
//     {
//         size_t operator()( const octomath::Vector3& coordinates ) const
//         {
//             double fractpart, intpart;
//             fractpart = modf (coordinates.x()*10000 , &intpart);
//             return intpart;
//         }
//     };
// }

namespace LazyThetaStarOctree{

	class InputData
	{
	public:
		octomap::OcTree const& octree;
		octomath::Vector3 const& start;
		octomath::Vector3 const& goal; 
		const double margin; 
		InputData(octomap::OcTree const& octree, const octomath::Vector3& start, const octomath::Vector3& goal, const double margin)
			: octree(octree), start(start), goal(goal), margin(margin)
		{}
	};

	class PublishingInput
	{
	public:
		ros::Publisher const& marker_pub;
		const bool publish;
		const std::string dataset_name;
		PublishingInput(ros::Publisher const& marker_pub, bool publish = false, std::string dataset_name = "unamed")
			: marker_pub(marker_pub), dataset_name(dataset_name), publish(publish)
		{}
	};

	class ObstacleAvoidanceInput
	{
		const InputData input;

	};

	enum CellStatus { kFree = 0, kOccupied = 1, kUnknown = 2 };
	
	double scale_float						(float value);
	CellStatus 	getLineStatus 				(InputData const& input);
	CellStatus 	getLineStatusBoundingBox	(InputData const& input);
	bool 		hasLineOfSight				(InputData const& input, bool ignoreUnknown = false);
	bool 		is_flight_corridor_free		(InputData const& input, PublishingInput const& publish_input, bool ignoreUnknown = false);
	float 		weightedDistance			(octomath::Vector3 const& start, octomath::Vector3 const& end);
	bool 		normalizeToVisibleEndCenter (octomap::OcTree const& octree, std::shared_ptr<octomath::Vector3> const& start, std::shared_ptr<octomath::Vector3> & end, double& cell_size, const double safety_margin, PublishingInput const& publish_input, const double sidelength_lookup_table[], bool ignoreUnknownF = false);
	/**
	 * @brief      Set vertex portion of pseudo code, ln 34.
	 *
	 * @param      octree     The octree
	 * @param      s          TThe node in analyzis
	 * @param      closed     The closed
	 * @param      open       The open
	 * @param[in]  neighbors  The neighbors
	 */
	bool setVertex(
		octomap::OcTree 										const& 	octree, 
		std::shared_ptr<ThetaStarNode> 							& 		s, 
		std::unordered_map<octomath::Vector3, std::shared_ptr<ThetaStarNode>, Vector3Hash, VectorComparatorEqual> &  closed,
		Open 													& 		open, 
		unordered_set_pointers									const& 	neighbors,
		std::ofstream 											& log_file,
		PublishingInput 										const& publish_input, 
		const double sidelength_lookup_table[],
		bool ignoreUnknown = false);

	
	bool setVertex_original(
		octomap::OcTree 										& 	octree, 
		std::shared_ptr<ThetaStarNode> 							& 		s, 
		std::unordered_map<octomath::Vector3, std::shared_ptr<ThetaStarNode>, Vector3Hash, VectorComparatorEqual> &  closed,
		Open 													& 		open, 
		std::unordered_set<std::shared_ptr<octomath::Vector3>> 	const& 	neighbors,
		std::ofstream & log_file,
		double safety_margin,
		PublishingInput const& publish_input,
		const double sidelength_lookup_table[],
		bool ignoreUnknown = false);

	/**
	 * @brief      Extracts a sequence of coordinates from the links between nodes starting at the goal node and expanding the connections to the prevuous point through parentNode.
	 *
	 * @param      path   The list object where to store the path
	 * @param      start  The starting node
	 * @param      end    The goal node
	 *
	 * @return     true is a path from goal node until start node was found (under 500 jumps). False otherwise.
	 */
	bool extractPath(std::list<octomath::Vector3> & path, ThetaStarNode const& start, ThetaStarNode & end);

	/**
	 * @brief      Calcute the cost if a link between these to nodes is created. This is not the ComputeCost method of the pseudo code.
	 *
	 * @param      s            node
	 * @param      s_neighbour  The neighbour
	 *
	 * @return     the new cost
	 */
	float CalculateCost(ThetaStarNode const& s, ThetaStarNode const& s_neighbour);

	void UpdateVertex(ThetaStarNode const& s, std::shared_ptr<ThetaStarNode> s_neighbour,
		Open & open);

	/**
	 * @brief      Lazy Theta Star. Decimal precision in 0.001 both in closed and in open (buildKey)
	 *
	 * @param      octree        The octree
	 * @param      disc_initial  The disc initial
	 * @param      disc_final    The disc final
	 *
	 * @return     A list of the ordered waypoints to get from initial to final
	 */
	std::list<octomath::Vector3> lazyThetaStar_(
		InputData const& input,
		ResultSet & resultSet,
		const double sidelength_lookup_table[],
		PublishingInput const& publish_input,
		int const& max_time_secs = 55);

	std::list<octomath::Vector3> lazyThetaStar_original(
		octomap::OcTree   & octree, 
		octomath::Vector3 const& disc_initial, 
		octomath::Vector3 const& disc_final,
		ResultSet & resultSet,
		double safety_margin,
		const double sidelength_lookup_table[],
		PublishingInput const& publish_input,
		int const& max_time_secs = 55,
		bool print_resulting_path = false);

	bool processLTStarRequest_original(octomap::OcTree & octree, lazy_theta_star_msgs::LTStarRequest const& request, lazy_theta_star_msgs::LTStarReply & reply, const double sidelength_lookup_table[], PublishingInput const& publish_input);

	bool processLTStarRequest(octomap::OcTree & octree, lazy_theta_star_msgs::LTStarRequest const& request, lazy_theta_star_msgs::LTStarReply & reply, const double sidelength_lookup_table[], PublishingInput const& publish_input);

    
}