#include "shakey_utils/geometryPoses.h"
#include <fstream>
#include <sstream>

GeometryPoses::GeometryPoses() { }

GeometryPoses::~GeometryPoses() { }

GeometryPoses::NamedPose GeometryPoses::getPoseFromString(const std::string & line)
{
    std::stringstream ss(line);
    std::string name;
    ss >> name;
    uint32_t tsec, tnsec;
    geometry_msgs::PoseStamped pose;
    ss >> tsec;
    ss >> tnsec;
    pose.header.stamp = ros::Time(tsec, tnsec);
    ss >> pose.header.frame_id;
    ss >> pose.pose.position.x;
    ss >> pose.pose.position.y;
    ss >> pose.pose.position.z;
    ss >> pose.pose.orientation.x;
    ss >> pose.pose.orientation.y;
    ss >> pose.pose.orientation.z;
    ss >> pose.pose.orientation.w;
    return std::make_pair(name, pose);
}

bool GeometryPoses::load(const std::string & filename)
{
   std::ifstream f(filename.c_str());
   if(!f.good()) {
      return false;
   }
   std::string line;
   while(f.good() && !f.eof()) {
      getline(f, line);
      size_t pos = line.find_first_not_of(" ");
      if(pos == std::string::npos)    // empty line
         continue;
      if(line[pos] == '#')       // comment line
         continue;
      // parse the line
      NamedPose np = getPoseFromString(line);
      _poses[np.first] = np.second;
   }
   return true;
}

std::string GeometryPoses::getPoseWriteString(const NamedPose & np)
{
    std::stringstream ss;
    ss << np.first << " ";
    ss << np.second.header.stamp.sec << " " << np.second.header.stamp.nsec << " ";
    ss << np.second.header.frame_id << " ";
    ss << np.second.pose.position.x << " ";
    ss << np.second.pose.position.y << " ";
    ss << np.second.pose.position.z << " ";
    ss << np.second.pose.orientation.x << " ";
    ss << np.second.pose.orientation.y << " ";
    ss << np.second.pose.orientation.z << " ";
    ss << np.second.pose.orientation.w;
    return ss.str();
}

std::ostream & operator<<(std::ostream & os, const GeometryPoses & poses)
{
   for(std::map<std::string, geometry_msgs::PoseStamped>::const_iterator it = poses._poses.begin(); it != poses._poses.end(); it++) {
      os << it->first << " = " << std::endl << it->second << std::endl;
   }
   return os;
}
