#pragma once

#include "pathplanner/lib/PathPlannerTrajectory.h"
#include <units/velocity.h>
#include <units/acceleration.h>
#include <string>
#include <vector>

namespace pathplanner{
    class PathPlanner {
        public:
            static double resolution;

            static pathplanner::PathPlannerTrajectory loadPath(std::string name, units::meters_per_second_t maxVel, units::meters_per_second_squared_t maxAccel, bool reversed);

            static pathplanner::PathPlannerTrajectory loadPath(std::string name, units::meters_per_second_t maxVel, units::meters_per_second_squared_t maxAccel){
                return PathPlanner::loadPath(name, maxVel, maxAccel, false);
            }
            static pathplanner::PathPlannerTrajectory loadPath(std::string name){
                return PathPlanner::loadPath(name,units::meters_per_second_t(0),units::meters_per_second_squared_t(0),false);
            }
        private:
            static pathplanner::PathPlannerTrajectory joinPaths(std::vector<pathplanner::PathPlannerTrajectory> paths);
    };
}