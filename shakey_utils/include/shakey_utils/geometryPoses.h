#ifndef GEOMETRY_POSES_H
#define GEOMETRY_POSES_H

#include <map>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

/// Named poses
class GeometryPoses
{
   friend std::ostream & operator<<(std::ostream & os, const GeometryPoses & poses);

   public:
      typedef std::pair<std::string, geometry_msgs::PoseStamped> NamedPose;

      GeometryPoses();
      ~GeometryPoses();

      bool load(const std::string & filename);

      const std::map<std::string, geometry_msgs::PoseStamped> & getPoses() const { return _poses; };

      static NamedPose getPoseFromString(const std::string & line);
      static std::string getPoseWriteString(const NamedPose & np);

   protected:
      std::map<std::string, geometry_msgs::PoseStamped> _poses;
};

std::ostream & operator<<(std::ostream & os, const GeometryPoses & poses);

#endif
