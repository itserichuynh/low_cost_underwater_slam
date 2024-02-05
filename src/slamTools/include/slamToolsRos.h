//
// Created by tim on 26.03.21.
//
#include "rclcpp/rclcpp.hpp"
#include "graphSlamSaveStructure.h"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav_msgs/msg/path.hpp"
#include <random>

#include "softRegistrationClass.h"

#ifndef SIMULATION_BLUEROV_SLAMTOOLSROS_H
#define SIMULATION_BLUEROV_SLAMTOOLSROS_H

struct measurement {
    int keyframe;
    double x;
    double y;
    double z;
    double timeStamp;
};
struct ImuData {
    double ax;//linear acceleration
    double ay;//linear acceleration
    double az;//linear acceleration
    double wx;//angular velocity
    double wy;//angular velocity
    double wz;//angular velocity
    double roll;//from g measured
    double pitch;//from g measured
    double yaw;//mostly useless
    double timeStamp;
};

struct DvlData {
    double vx; // linear body velocity
    double vy; // linear body velocity
    double vz; // linear body velocity
    double height; // above sea
    double timeStamp;
};

struct intensityValues {
    Eigen::Matrix4d transformation;
    intensityMeasurement intensity;
};
struct transformationStamped {
    Eigen::Matrix4d transformation;
    double timeStamp;
};


class slamToolsRos {

public:

    static void visualizeCurrentPoseGraph(graphSlamSaveStructure *graphSaved,
                                          rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &publisherPath,
                                          rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &publisherMarkerArray,
                                          double sigmaScaling,
                                          rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &publisherPoseSlam,
                                          rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &publisherLoopClosures,
                                          rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &publisherPathGT);






    static edge
    calculatePoseDiffByTimeDepOnEKF(double startTimetoAdd, double endTimeToAdd,
                                    std::deque<transformationStamped> &transformationList,
                                    std::mutex &mutexForAccess);


    static double angleBetweenLastKeyframeAndNow(graphSlamSaveStructure *graphSaved);

    static int getLastIntensityKeyframe(graphSlamSaveStructure *graphSaved);

    static double getDatasetFromGraphForMap(std::vector<intensityValues> &dataSet, graphSlamSaveStructure *graphSaved,
                                            std::mutex &graphSlamMutex);

    static void clearSavingsOfPoses(double upToTime,std::deque<transformationStamped> &transformationList,
                                    std::deque<transformationStamped> &currentPositionGTDeque,
                                    std::mutex &stateEstimationMutex);


    static double createVoxelOfGraphStartEndPoint(double voxelData[], int indexStart, int indexEnd,
                                                  int numberOfPoints, graphSlamSaveStructure *usedGraph,
                                                  double ignoreDistanceToRobot, double dimensionOfVoxelData,
                                                  Eigen::Matrix4d transformationInTheEndOfCalculation);

    static bool calculateStartAndEndIndexForVoxelCreation(int indexMiddle, int &indexStart, int &indexEnd,
                                                          graphSlamSaveStructure *usedGraph);


    static bool
    loopDetectionByClosestPath(graphSlamSaveStructure *graphSaved, softRegistrationClass *scanRegistrationObject,
                               int dimensionOfVoxelData,
                               double ignoreDistanceToRobot, double distanceOfVoxelDataLengthSI, bool useInitialTranslation,int ignoreStartLoopClosure,int ignoreEndLoopClosure,
                               double potentialNecessaryForPeak = 0.1, double maxLoopClosure = 100);





    static bool
    calculateEndIndexForVoxelCreationByStartIndex(int indexStart, int &indexEnd, graphSlamSaveStructure *usedGraph);




};

#endif //SIMULATION_BLUEROV_VISUALIZESLAMINROS_H
