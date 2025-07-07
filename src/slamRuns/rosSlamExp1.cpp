//
// Created by jurobotics on 13.09.21.
//

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "ping360_sonar_msgs/msg/sonar_echo.hpp"
#include "generalHelpfulTools.h"
#include "slamToolsRos.h"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

//#define DEBUG_REGISTRATION false


class rosClassSlam : public rclcpp::Node {
public:
    rosClassSlam() : Node("rosSlamTest") {

        this->declare_parameter("registration_number_of_pixels_dimension", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("registration_size_of_scan", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("registration_threshold_translation", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("map_size", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("dimension_size_map", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("distance_ignored_around_robot", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("rotation_sonar_on_robot", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("number_of_loop_closures_per_rotation", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("measurement_noise_ekf_xy", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("measurement_noise_ekf_yaw", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("maximum_loopclosure_distance", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("sonar_forward_looking_down", rclcpp::PARAMETER_BOOL);


        this->registration_number_of_pixels_dimension = (int)this->get_parameter("registration_number_of_pixels_dimension").as_int();
        this->registration_size_of_scan =this->get_parameter("registration_size_of_scan").as_double();
        this->registration_threshold_translation = this->get_parameter("registration_threshold_translation").as_double();
        this->map_size = (int)this->get_parameter("map_size").as_int();
        this->dimension_size_map = this->get_parameter("dimension_size_map").as_double();
        this->distance_ignored_around_robot = this->get_parameter("distance_ignored_around_robot").as_double();
        this->rotation_sonar_on_robot = this->get_parameter("rotation_sonar_on_robot").as_double();
        this->number_of_loop_closures_per_rotation = this->get_parameter("number_of_loop_closures_per_rotation").as_double();
        this->measurement_noise_ekf_xy = this->get_parameter("measurement_noise_ekf_xy").as_double();
        this->measurement_noise_ekf_yaw = this->get_parameter("measurement_noise_ekf_yaw").as_double();
        this->maximum_loopclosure_distance = this->get_parameter("maximum_loopclosure_distance").as_double();
        this->sonar_forward_looking_down = this->get_parameter("sonar_forward_looking_down").as_bool();


        graphSaved = new graphSlamSaveStructure(3, INTENSITY_BASED_GRAPH);

        scanRegistrationObject = new softRegistrationClass(this->registration_number_of_pixels_dimension, this->registration_number_of_pixels_dimension / 2,
                                                           this->registration_number_of_pixels_dimension / 2,
                                                           this->registration_number_of_pixels_dimension / 2 - 1);



        //we have to make sure, to get ALL the data. Therefor we have to change that in the future.
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_system_default);
        qos.history(rmw_qos_history_policy_e::RMW_QOS_POLICY_HISTORY_KEEP_ALL);
        qos.reliability(rmw_qos_reliability_policy_e::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(rmw_qos_durability_policy_e::RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
        qos.liveliness(rmw_qos_liveliness_policy_e::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT);
        qos.deadline(rmw_time_t(RMW_DURATION_INFINITE));
        qos.lifespan(rmw_time_t(RMW_DURATION_INFINITE));
        qos.liveliness_lease_duration(rmw_time_t(RMW_DURATION_INFINITE));
        qos.avoid_ros_namespace_conventions(false);

        // these allow to control how callbacks are scheduled and run, especially in a multi-threaded executor
        // MutuallyExclusive: only one callback in the group can be executing at a time
        this->callback_group_subscriber1_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
        this->callback_group_subscriber2_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = callback_group_subscriber2_;

        this->subscriberEKF = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "publisherPoseEkf", qos,
                std::bind(&rosClassSlam::stateEstimationCallback,
                          this, std::placeholders::_1), sub1_opt);

        this->subscriberIntensitySonar = this->create_subscription<ping360_sonar_msgs::msg::SonarEcho>(
                "sonar/intensity", qos,
                std::bind(&rosClassSlam::scanCallback,
                          this, std::placeholders::_1), sub2_opt);

        this->publisherPositionOverTime = this->create_publisher<nav_msgs::msg::Path>(
                "positionOverTime", qos);

        this->publisherEKF = this->create_publisher<nav_msgs::msg::Path>(
                "positionOverTimeGT", qos);

        this->publisherMarkerArrayCovariance = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "covariance", qos);
        this->publisherMarkerArrayLoopClosures = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "loopClosures", qos);
        this->publisherOccupancyMap = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                "occupancyHilbertMap", qos);
        this->publisherEndPose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "slamEndPose", qos);

        std::chrono::duration<double> my_timer_duration = std::chrono::duration<double>(5.0);
        this->timer_ = this->create_wall_timer(
                my_timer_duration, std::bind(&rosClassSlam::createImageOfAllScans, this));

        this->sigmaScaling = 1.0;


        this->firstSonarInput = true;
        this->firstCompleteSonarScan = true;

        this->numberOfTimesFirstScan = 0;


        // fill in info about the OccupancyGrid
        map.info.height = this->map_size;
        map.info.width = this->map_size;
        map.info.resolution = this->dimension_size_map / this->map_size;
        map.info.origin.position.x = -this->dimension_size_map / 2;
        map.info.origin.position.y = -this->dimension_size_map / 2;
        map.info.origin.position.z = +0.5;

        // fill in occupancy probability
        for (int i = 0; i < this->map_size; i++) {
            for (int j = 0; j < this->map_size; j++) {
                //determine color:
                map.data.push_back(50);
            }
        }
    }


private:
    nav_msgs::msg::OccupancyGrid map;


    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriberEKF;
    rclcpp::Subscription<ping360_sonar_msgs::msg::SonarEcho>::SharedPtr subscriberIntensitySonar;


    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisherEndPose;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisherOccupancyMap;

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
    rclcpp::TimerBase::SharedPtr timer_;


    std::mutex stateEstimationMutex;
    std::mutex groundTruthMutex;
    std::mutex graphSlamMutex;
    //GraphSlam things
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherPositionOverTime;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherEKF;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisherMarkerArrayCovariance;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisherMarkerArrayLoopClosures;


    //Matrices:
    Eigen::Matrix4d currentEstimatedTransformation;
    Eigen::Matrix4d initialGuessTransformation;


    //EKF savings
    std::deque<edge> posDiffOverTimeEdges;
    std::deque<transformationStamped> ekfTransformationList;

    // GT savings
    std::deque<transformationStamped> currentPositionGTDeque;
    Eigen::Matrix4d currentGTPosition;


    double sigmaScaling;


    graphSlamSaveStructure *graphSaved;
    softRegistrationClass *scanRegistrationObject;
    bool firstSonarInput, firstCompleteSonarScan;
    std::string saveStringGraph;
    int numberOfTimesFirstScan;


    int registration_number_of_pixels_dimension;
    double registration_size_of_scan;
    double registration_threshold_translation;
    int map_size;
    double dimension_size_map;
    double distance_ignored_around_robot;
    double rotation_sonar_on_robot;
    double number_of_loop_closures_per_rotation;
    double measurement_noise_ekf_xy;
    double measurement_noise_ekf_yaw;
    double maximum_loopclosure_distance;
    bool sonar_forward_looking_down;

    void scanCallback(const ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);

        // from gradian to radian
        intensityMeasurement intensityTMP;
        if (this->sonar_forward_looking_down) {
            intensityTMP.angle = std::fmod(-msg->angle / 400.0 * M_PI * 2.0 + this->rotation_sonar_on_robot,
                                           M_PI * 2);// TEST TRYING OUT -
        } else {
            // gradian to radian
            intensityTMP.angle = std::fmod(msg->angle / 400.0 * M_PI * 2.0 + this->rotation_sonar_on_robot,
                                           M_PI * 2);// TEST TRYING OUT -
        }

        intensityTMP.time = rclcpp::Time(msg->header.stamp).seconds();
        intensityTMP.range = msg->range;
        intensityTMP.increment = msg->step_size;
        std::vector<double> intensitiesVector;
        for (int i = 0; i < msg->intensities.size(); i++) {
            intensitiesVector.push_back(msg->intensities[i]);
        }
        intensityTMP.intensities = intensitiesVector;

        // initialize the graph with first scan
        if (firstSonarInput) {
            this->graphSaved->addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                                        Eigen::Matrix3d::Zero(), intensityTMP,
                                        rclcpp::Time(msg->header.stamp).seconds(),
                                        FIRST_ENTRY);
            firstSonarInput = false;
            return;
        }

        //add a new edge and vertex to the graph defined by EKF and Intensity Measurement
        bool messageReceived = waitForEKFMessagesToArrive(rclcpp::Time(msg->header.stamp).seconds());
        if (!messageReceived) {
            std::cout << "return no message found: " << rclcpp::Time(msg->header.stamp).seconds() << "    "
                      << rclcpp::Clock(RCL_ROS_TIME).now().seconds() << std::endl;
            return;
        }

        // calculate the pose difference for an edge
        edge differenceOfEdge = slamToolsRos::calculatePoseDiffByTimeDepOnEKF(
                this->graphSaved->getVertexList()->back().getTimeStamp(), rclcpp::Time(msg->header.stamp).seconds(),
                this->ekfTransformationList, this->stateEstimationMutex);
        slamToolsRos::clearSavingsOfPoses(this->graphSaved->getVertexList()->back().getTimeStamp() - 2.0,
                                          this->ekfTransformationList, this->currentPositionGTDeque,
                                          this->stateEstimationMutex);
        
        // get the last vertex and apply the recently calculated transformation -> pose of the new scan line
        Eigen::Matrix4d tmpTransformation = this->graphSaved->getVertexList()->back().getTransformation();
        tmpTransformation = tmpTransformation * differenceOfEdge.getTransformation();
        Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
        Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
        Eigen::Quaterniond rot(rotM);


        this->graphSaved->addVertex(this->graphSaved->getVertexList()->back().getKey() + 1, pos, rot,
                                    this->graphSaved->getVertexList()->back().getCovarianceMatrix(),
                                    intensityTMP,
                                    rclcpp::Time(msg->header.stamp).seconds(),
                                    INTENSITY_SAVED);


        Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
        covarianceMatrix(0, 0) = this->measurement_noise_ekf_xy;
        covarianceMatrix(1, 1) = this->measurement_noise_ekf_xy;
        covarianceMatrix(2, 2) = this->measurement_noise_ekf_yaw;
        this->graphSaved->addEdge(this->graphSaved->getVertexList()->back().getKey() - 1,
                                  this->graphSaved->getVertexList()->back().getKey(),
                                  differenceOfEdge.getPositionDifference(), differenceOfEdge.getRotationDifference(),
                                  covarianceMatrix, INTEGRATED_POSE);

        // check local loop closure
        // check if enough rotation has happened to consider this a keyframe
        double angleDiff = slamToolsRos::angleBetweenLastKeyframeAndNow(this->graphSaved);
        
        // best would be scan matching between this angle and transformation based last angle( i think this is currently done)
        if (abs(angleDiff) > 2 * M_PI / this->number_of_loop_closures_per_rotation) {

            // a node is considered a keyframe if it is far enough from the last keyframe
            // given the specification of number_of_loop_closures_per_rotation
            // this is for local loop closure
            this->graphSaved->getVertexList()->back().setTypeOfVertex(INTENSITY_SAVED_AND_KEYFRAME);

            // we want to wait until we have enough coverage before attempting loop closures during the first scan
            if (firstCompleteSonarScan) {
                numberOfTimesFirstScan++;
                if (numberOfTimesFirstScan > 2 * this->number_of_loop_closures_per_rotation - 1) {
                    firstCompleteSonarScan = false;
                }
                return;
            }

            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

            // Loop closure process
            int indexStart1, indexEnd1, indexStart2, indexEnd2;

            // go back in time 5 nodes and build a voxel
            // I do not like this
            // @TODO about the -5
            slamToolsRos::calculateStartAndEndIndexForVoxelCreation(
                    this->graphSaved->getVertexList()->back().getKey() - 5, indexStart1, indexEnd1, this->graphSaved);
            indexStart2 = indexEnd1;

            //
            slamToolsRos::calculateEndIndexForVoxelCreationByStartIndex(indexStart2, indexEnd2, this->graphSaved);


            std::cout << "scanAcusitionTime: " << this->graphSaved->getVertexList()->at(indexStart2).getTimeStamp() -
                                                  this->graphSaved->getVertexList()->at(indexEnd2).getTimeStamp()
                      << std::endl;

            // we inverse the initial guess, because the registration creates a T from scan 1 to scan 2.
            // But the graph creates a transformation from 1 -> 2 by the robot, therefore inverse.
            this->initialGuessTransformation =
                    (this->graphSaved->getVertexList()->at(indexStart2).getTransformation().inverse() *
                     this->graphSaved->getVertexList()->at(indexStart1).getTransformation());
            
            // @TODO
            // I don't like this next line
            // we assume that the robot stays level the whole time that the rotational matrix only contains rotation around Z axis
            // the ideal way to to calculate rpy from the rotational matrix and then extract yaw like this
            // Eigen::Vector3d rpy = generalHelpfulTools::getRollPitchYaw(this->initialGuessTransformation);
            // double initialGuessAngle = rpy(2);
            double initialGuessAngle = std::atan2(this->initialGuessTransformation(1, 0),
                                                  this->initialGuessTransformation(0, 0));

            double *voxelData1;
            double *voxelData2;
            voxelData1 = (double *) malloc(sizeof(double) * this->registration_number_of_pixels_dimension * this->registration_number_of_pixels_dimension);
            voxelData2 = (double *) malloc(sizeof(double) * this->registration_number_of_pixels_dimension * this->registration_number_of_pixels_dimension);

            double maximumVoxel1 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData1, indexStart1, indexEnd1,
                                                                                 this->registration_number_of_pixels_dimension,
                                                                                 this->graphSaved,
                                                                                 this->distance_ignored_around_robot,
                                                                                 this->registration_size_of_scan,
                                                                                 Eigen::Matrix4d::Identity());//get voxel


            double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData2, indexStart2, indexEnd2,
                                                                                 this->registration_number_of_pixels_dimension,
                                                                                 this->graphSaved,
                                                                                 this->distance_ignored_around_robot,
                                                                                 this->registration_size_of_scan,
                                                                                 Eigen::Matrix4d::Identity());//get voxel

            // Normalize the Voxels
            double overallMax = std::max(maximumVoxel1, maximumVoxel2);
            for (int i = 0; i < this->registration_number_of_pixels_dimension * this->registration_number_of_pixels_dimension; i++) {
                voxelData1[i] = voxelData1[i] / overallMax;
                voxelData2[i] = voxelData2[i] / overallMax;
            }


            Eigen::Matrix3d covarianceEstimation = Eigen::Matrix3d::Zero();
            std::cout << "direct matching consecutive: " << std::endl;
            // result is matrix to transform scan 1 to scan 2 therefore later inversed + initial guess inversed

            this->currentEstimatedTransformation = this->scanRegistrationObject->registrationOfTwoVoxelsSOFFTFast(
                    voxelData1, voxelData2,
                    this->initialGuessTransformation,
                    covarianceEstimation, true, true, (double) this->registration_size_of_scan /
                                                      (double) this->registration_number_of_pixels_dimension, false,
                    0.1);

//            slamToolsRos::saveResultingRegistrationTMPCOPY(indexStart1, indexEnd1, indexStart2, indexEnd2,
//                                                           this->graphSaved, this->registration_number_of_pixels_dimension,
//                                                           this->distance_ignored_around_robot,
//                                                           this->registration_size_of_scan,
//                                                           DEBUG_REGISTRATION, this->currentEstimatedTransformation,
//                                                           initialGuessTransformation);
            std::cout << "Printing Initial guess and estimated transformation:" << std::endl;
            std::cout << this->initialGuessTransformation << std::endl;
            std::cout << this->currentEstimatedTransformation << std::endl;

            double differenceAngleBeforeAfter = generalHelpfulTools::angleDiff(
                    std::atan2(this->currentEstimatedTransformation(1, 0), this->currentEstimatedTransformation(0, 0)),
                    initialGuessAngle);



            // only if angle diff is smaller than 40 degreece its ok
            if (abs(differenceAngleBeforeAfter) < 40.0 / 180.0 * M_PI) {
                //inverse the transformation because we want the robot transformation, not the scan transformation
                Eigen::Matrix4d transformationEstimationRobot1_2 = this->currentEstimatedTransformation;
                Eigen::Quaterniond qTMP(transformationEstimationRobot1_2.block<3, 3>(0, 0));

                // now you add an edge between the two found node for loop closure
                graphSaved->addEdge(indexStart2,
                                    indexStart1,
                                    transformationEstimationRobot1_2.block<3, 1>(0, 3), qTMP,
                                    covarianceEstimation,
                                    LOOP_CLOSURE);//@TODO still not sure about size

            } else {
                std::cout << "we just skipped that registration because its to far away to be true" << std::endl;
            }
            std::cout << "loopClosure: " << std::endl;

            ////////////// look for loop closure  //////////////
            // @TODO I guess this is for global loop closure?
            bool foundGlobalLoopClosure = slamToolsRos::loopDetectionByClosestPath(this->graphSaved, this->scanRegistrationObject,
                                                     this->registration_number_of_pixels_dimension, this->distance_ignored_around_robot,
                                                     this->registration_size_of_scan,
                                                     true, 250, 500,
                                                     this->registration_threshold_translation, this->maximum_loopclosure_distance);
            
            std::cout << "Found a global loop closure" << std::endl;

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            std::cout << "Loop closure computation time: " << timeToCalculate << std::endl;

            this->graphSaved->isam2OptimizeGraph(true, 2);

//            slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherPositionOverTime,
//                                                    this->publisherMarkerArrayCovariance, this->sigmaScaling,
//                                                    this->publisherEndPose, this->publisherMarkerArrayLoopClosures,
//                                                    this->publisherEKF);
            std::cout << "next: " << std::endl;


        }

        // @TODO
        slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherPositionOverTime,
                                                this->publisherMarkerArrayCovariance, this->sigmaScaling,
                                                this->publisherEndPose, this->publisherMarkerArrayLoopClosures,
                                                this->publisherEKF);

    }

    // this produces a list of transformations wrt initial pos
    void stateEstimationCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

        std::lock_guard<std::mutex> lock(this->stateEstimationMutex);


        double currentTimeStamp = rclcpp::Time(msg->header.stamp).seconds();

        // calculate where to put the current new message
        int i = 0;
        if (!this->ekfTransformationList.empty()) {
            i = this->ekfTransformationList.size();
            while (this->ekfTransformationList[i - 1].timeStamp > currentTimeStamp) { // this means current msg should be earlier in the list
                i--;
            }
        }

        if (i == this->ekfTransformationList.size() || i == 0) { // only allows to insert at the end or at the beginning
            Eigen::Quaterniond tmpQuad(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
            Eigen::Vector3d tmpVec(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            Eigen::Matrix4d transformationMatrix = generalHelpfulTools::getTransformationMatrix(tmpVec, tmpQuad);

            transformationStamped tmpTransformationStamped;
            tmpTransformationStamped.timeStamp = rclcpp::Time(msg->header.stamp).seconds();
            tmpTransformationStamped.transformation = transformationMatrix;

            this->ekfTransformationList.push_back(tmpTransformationStamped);

        } else {
            std::cout << "should mean an EKF message came in different order" << std::endl;
            exit(0);
        }
    }

    bool waitForEKFMessagesToArrive(double timeUntilWait) {

        // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        // double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        // while (this->ekfTransformationList.empty() && timeToCalculate < 2000) {
        //     rclcpp::sleep_for(std::chrono::nanoseconds(2000000));
        //     end = std::chrono::steady_clock::now();
        //     timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        // }
        // while (timeUntilWait > ekfTransformationList.back().timeStamp) {
        //     rclcpp::sleep_for(std::chrono::nanoseconds(2000000));
        //     end = std::chrono::steady_clock::now();
        //     timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        //     double timeToWait = 4000;
        //     if (timeToCalculate > timeToWait) {
        //         std::cout << "we break" << std::endl;
        //         break;
        //     }
        // }

        // rclcpp::sleep_for(std::chrono::nanoseconds(2000000));
        // end = std::chrono::steady_clock::now();
        // timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        // double timeToWait = 40;
        // if (timeToCalculate > timeToWait) {
        //     return false;
        // } else {
        //     return true;
        // }

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::chrono::seconds timeout = std::chrono::seconds(2);

        while (this->ekfTransformationList.empty()) {
            if (std::chrono::steady_clock::now() - begin > timeout)
                return false;
            rclcpp::sleep_for(std::chrono::nanoseconds(2000000));
        }

        timeout = std::chrono::seconds(4);

        while (timeUntilWait > ekfTransformationList.back().timeStamp) {
            if (std::chrono::steady_clock::now() - begin > timeout)
                return false;

            rclcpp::sleep_for(std::chrono::nanoseconds(2000000));
        }

        return true;
    }

public:
    // this function is used to create an occupancy map based on the pose graph
    void createImageOfAllScans() {
        std::vector<intensityValues> dataSet;
        double maximumIntensity = slamToolsRos::getDatasetFromGraphForMap(dataSet, this->graphSaved,
                                                                          this->graphSlamMutex);
        
        // @TODO
        //voxeldata is incremented whenever we add new data into mapData to later normalize mapData
        // hence, I should come up with better name
        int *voxelDataIndex;
        voxelDataIndex = (int *) malloc(sizeof(int) * this->map_size * this->map_size);

        double *mapData;
        mapData = (double *) malloc(sizeof(double) * this->map_size * this->map_size);
        //set zero voxel and index
        for (int i = 0; i < this->map_size * this->map_size; i++) {
            voxelDataIndex[i] = 0;
            mapData[i] = 0;
        }

        // iterate thru all the vertexes
        for (int currentPosition = 0; currentPosition < dataSet.size(); currentPosition++) {
            // calculate the position of each intensity and create an index in two arrays.
            // First in voxel data, and second save number of intensities.
            // was 90 yaw and 180 roll

            Eigen::Matrix4d transformationOfIntensityRay =
                    generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 0.0 / 180.0 * M_PI) *
                    generalHelpfulTools::getTransformationMatrixFromRPY(0.0 / 180.0 * M_PI, 0, 0) *
                    dataSet[currentPosition].transformation;
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             dataSet[currentPosition].intensity.angle);
            
            // determine how many index we have to ignore
            int ignoreDistance = (int) (this->distance_ignored_around_robot / (dataSet[currentPosition].intensity.range /
                                                                    ((double) dataSet[currentPosition].intensity.intensities.size())));

            
            // iterate thru all the intensities and ignore the ignored area
            for (int j = ignoreDistance; j < dataSet[currentPosition].intensity.intensities.size(); j++) {
                // find the distance in meters at this index
                double distanceOfIntensity =
                        (j / ((double) dataSet[currentPosition].intensity.intensities.size())) *
                        ((double) dataSet[currentPosition].intensity.range);

                int incrementOfScan = dataSet[currentPosition].intensity.increment; // angle step size

                // @TODO I dont really understand what l is
                // we smear the scan line for better approximate of the true sonar beam width and improve map coverage
                // this is due to the assumption that one scan line never truly scan only one angle section 
                for (int l = -incrementOfScan - 5; l <= incrementOfScan + 5; l++) {
                    Eigen::Vector4d positionOfIntensity(
                            distanceOfIntensity,
                            0,
                            0,
                            1);
                    double rotationOfPoint = (l / 400.0) * 2.0 * M_PI;
                    Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(0,
                                                                                                                0,
                                                                                                                rotationOfPoint);
                    positionOfIntensity = rotationForBetterView * positionOfIntensity;

                    positionOfIntensity =
                            transformationOfIntensityRay * rotationOfSonarAngleMatrix * positionOfIntensity;
                    //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
                    int indexX =
                            (int) (positionOfIntensity.x() / (this->dimension_size_map / 2) * this->map_size /
                                   2) +
                            this->map_size / 2;
                    int indexY =
                            (int) (positionOfIntensity.y() / (this->dimension_size_map / 2) * this->map_size /
                                   2) +
                            this->map_size / 2;


                    if (indexX < this->map_size && indexY < this->map_size && indexY >= 0 &&
                        indexX >= 0) {
                        //if index fits inside of our data, add that data. Else Ignore
                        voxelDataIndex[indexX + this->map_size * indexY] =
                                voxelDataIndex[indexX + this->map_size * indexY] + 1;
                        mapData[indexX + this->map_size * indexY] =
                                mapData[indexX + this->map_size * indexY] +
                                dataSet[currentPosition].intensity.intensities[j];
                    }
                }
            }

        }


        //make sure next iteration the correct registrationis calculated
        //TO THE END
        //NOW: TO THE BEGINNING

        // normalize data
        double maximumOfVoxelData = 0;
        double minimumOfVoxelData = INFINITY;

        for (int i = 0; i < this->map_size * this->map_size; i++) {
            if (voxelDataIndex[i] > 0) {
                mapData[i] = mapData[i] / voxelDataIndex[i];
                if (maximumOfVoxelData < mapData[i]) {
                    maximumOfVoxelData = mapData[i];
                }
                if (minimumOfVoxelData > mapData[i]) {
                    minimumOfVoxelData = mapData[i];
                }
            }
        }

        // Normalize map to values [0, 250] (grayscale image values)
        for (int i = 0; i < this->map_size * this->map_size; i++) {

            mapData[i] = (mapData[i] - minimumOfVoxelData) / (maximumOfVoxelData - minimumOfVoxelData) * 250;
        }

        // build and publish occupancy map
        nav_msgs::msg::OccupancyGrid occupanyMap;
        occupanyMap.header.frame_id = "map_ned";
        occupanyMap.info.height = this->map_size;
        occupanyMap.info.width = this->map_size;
        occupanyMap.info.resolution = this->dimension_size_map / this->map_size;
        occupanyMap.info.origin.position.x = -this->dimension_size_map / 2;
        occupanyMap.info.origin.position.y = -this->dimension_size_map / 2;
        occupanyMap.info.origin.position.z = +0.5;
        for (int i = 0; i < this->map_size; i++) {
            for (int j = 0; j < this->map_size; j++) {
                //determine color:
                occupanyMap.data.push_back((int) (mapData[j + this->map_size * i]));
            }
        }

        this->publisherOccupancyMap->publish(occupanyMap);
        free(voxelDataIndex);
        free(mapData);
    }


};


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rosClassSlam>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();


    return (0);
}
