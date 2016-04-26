#include <shakey_utils/MapHelper.h>

using namespace std;
using namespace Eigen;

bool MapHelper::getPushDistances(MatrixXf A, VectorXf b, VectorXf * x) {
	// Adding offset of 0.75
	*x = A.colPivHouseholderQr().solve(b);
	MatrixXf::Index minRow, minCol;
	// Check here if an entry is negative
	float min = x->minCoeff(&minRow, &minCol);
	double relative_error = (A * (*x) - b).norm() / b.norm();
	// TODO: < 0.1 nec?
	return relative_error < 0.01 && (min >= 0 || abs(min) < 0.1);
}

// Additional Functions
float MapHelper::getOccValue(geometry_msgs::Point p) {
	nav_msgs::OccupancyGrid occGrid = _map.response.map;
	nav_msgs::MapMetaData info = _map.response.map.info;
	tf::Point pt;
	tf::pointMsgToTF(p, pt);
	tf::Transform map_to_world;
	tf::poseMsgToTF(info.origin, map_to_world);
	tf::Point p2 = map_to_world.inverse() * pt;
	int16_t grid_x = static_cast<int16_t>(floor(p2.x() / info.resolution));
	int16_t grid_y = static_cast<int16_t>(floor(p2.y() / info.resolution));
	uint32_t cell = static_cast<uint32_t>(grid_x + grid_y * info.width);
	if (grid_x < 0 || grid_y < 0 || grid_x >= (int16_t) info.width
			|| grid_y >= info.height) {
		ROS_ERROR("Cell [%d, %d]: %d out of costmap.", grid_x, grid_y, cell);
		return -1;
	}
	return occGrid.data[cell];
}

bool MapHelper::freeSpace(geometry_msgs::Point p) {
	float occ_value = getOccValue(p);
	if (occ_value > 98) {
		ROS_WARN("OccValue: %f", occ_value);
		return false;
	}
	return true;
}

bool MapHelper::getPushDistances(geometry_msgs::Pose push1,
		geometry_msgs::Pose push2, geometry_msgs::Pose dest, VectorXf * x) {
	// get push vectors
		tf::Quaternion qt;
		tf::quaternionMsgToTF(push1.orientation, qt);
		tf::Matrix3x3 m;
		m.setIdentity();
		m.setRotation(qt);
		tf::Vector3 direction1 = m.getColumn(0).normalized();
		tf::quaternionMsgToTF(push2.orientation, qt);
		m.setRotation(qt);
		tf::Vector3 direction2 = m.getColumn(0).normalized();
		MatrixXf push_vecs(2, 2);
		push_vecs << direction1.x(), direction2.x(), direction1.y(), direction2.y();
		VectorXf b(2, 1);
		float pos_x = push1.position.x + (0.75 + 0.375) * direction1.x();
		float pos_y = push1.position.y + (0.75 + 0.375) * direction1.y();
		// Vector from current pos (push1) to dest
		b << dest.position.x - pos_x, dest.position.y - pos_y;
		return getPushDistances(push_vecs, b, x);

}
