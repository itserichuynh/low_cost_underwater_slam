//
// Created by auvatjacobs on 30.11.21.
//

#ifndef UNDERWATERSLAM_EKFDVL_H
#define UNDERWATERSLAM_EKFDVL_H

#include <deque>
//#include "../src/slamTools/generalHelpfulTools.h"
#include "generalHelpfulTools.h"
#include "pose.h"
//#include "../src/slamTools/slamToolsRos.h"
//asynchronous EKF with reset of POS correction

/*
* EKF logic interface
*/
class ekfClassDVL {
public:
    ekfClassDVL(rclcpp::Time timeRos) {
        this->stateOfEKF.position = Eigen::Vector3f(0, 0, 0);
        this->stateOfEKF.rotation = Eigen::Vector3f(0, 0, 0);
        this->stateOfEKF.velocity = Eigen::Vector3f(0, 0, 0);
        this->stateOfEKF.angleVelocity = Eigen::Vector3f(0, 0, 0);
        this->stateOfEKF.timeLastPrediction = timeRos;

        this->processNoise = Eigen::MatrixXd::Identity(12, 12); // this tells us how much we believe in the model dynamics
        this->processNoise(0, 0) = 0.1;//x 0.1 to 0.5
        this->processNoise(1, 1) = 0.1;//y 0.1 to 0.5
        this->processNoise(2, 2) = 0.005;//z 0.1 to 0.5
        this->processNoise(3, 3) = 100.0;//vx // we aren't predicting nor updating velocity with USBL
        this->processNoise(4, 4) = 100.0;//vy
        this->processNoise(5, 5) = 100.0;//vz
        this->processNoise(6, 6) = 0.01;//r
        this->processNoise(7, 7) = 0.01;//p
        this->processNoise(8, 8) = 0.01;//y
        this->processNoise(9, 9) = 0.001;//vr
        this->processNoise(10, 10) = 0.001;//vp
        this->processNoise(11, 11) = 0.001;//vy

        this->measurementNoiseDepth = Eigen::MatrixXd::Identity(12, 12);
        this->measurementNoiseDepth(2, 2) = 0.5;//z 0.1 to 0.5

        this->measurementNoiseDVL = Eigen::MatrixXd::Identity(12, 12);
        this->measurementNoiseDVL(3, 3) = 0.2;//vx 0.25 to 0.5
        this->measurementNoiseDVL(4, 4) = 0.2;//vy
        this->measurementNoiseDVL(5, 5) = 0.2;//vz


        this->measurementImuVelocity = Eigen::MatrixXd::Identity(12, 12);
        this->measurementImuVelocity(3, 3) = 1e6;//vx
        this->measurementImuVelocity(4, 4) = 1e6;//vy
        this->measurementImuVelocity(5, 5) = 1e6;//vz
        this->measurementImuVelocity(6, 6) = 100.0;//r
        this->measurementImuVelocity(7, 7) = 100.0;//p
        this->measurementImuVelocity(9, 9) = 0.1;//vr
        this->measurementImuVelocity(10, 10) = 0.1;//vp
        this->measurementImuVelocity(11, 11) = 0.1;//vy


        // @TODO I dont know what to set this to just yet...
        this->measurementNoiseUSBL = Eigen::MatrixXd::Identity(12, 12);
        this->measurementNoiseUSBL(0, 0) = 0.8;//x in meters
        this->measurementNoiseUSBL(1, 1) = 0.8;//y in meters

        // @TODO I dont know what to set this to just yet...
        this->measurementMagnetometer = Eigen::MatrixXd::Identity(12, 12);
        this->measurementMagnetometer(8, 8) = 100; //yaw 0.5 to 1.0



        this->stateOfEKF.covariance = processNoise;

        this->recentPoses.push_back(stateOfEKF);
    }

    void predictionImu(double xAccel, double yAccel, double zAccel, Eigen::Quaterniond currentRotation,Eigen::Vector3d positionIMU, rclcpp::Time timeStamp);

    void predictionImuNoVelocity(double xAccel, double yAccel, double zAccel, Eigen::Quaterniond currentRotation,Eigen::Vector3d positionIMU, rclcpp::Time timeStamp);

    void updateDVL(double xVel, double yVel, double zVel, Eigen::Quaterniond rotationOfDVL,Eigen::Vector3d positionDVL, rclcpp::Time timeStamp);

    void updateUSBL(double x, double y, double z, rclcpp::Time timeStamp);

    void updateIMU(double roll, double pitch, double xAngularVel, double yAngularVel, double zAngularVel,
                   Eigen::Quaterniond currentRotation, rclcpp::Time timeStamp);

    void updateIMUNoVelocity(double roll, double pitch, double xAngularVel, double yAngularVel, double zAngularVel,
                   Eigen::Quaterniond currentRotation, rclcpp::Time timeStamp);
    
    void updateMagnetometer(double x_mag, double y_mag, double z_mag, rclcpp::Time timeStamp);

    void updateHeight(double depth, rclcpp::Time timeStamp);

    pose getState();

    Eigen::Quaterniond getRotationVector();

    Eigen::Quaterniond getRotationVectorWithoutYaw();


    Eigen::VectorXd innovationStateDiff(Eigen::VectorXd z, Eigen::MatrixXd H, Eigen::VectorXd currentStateBeforeUpdate);

private:
    // internal pose object to store an estimate
    pose stateOfEKF;
    //std::deque<edge> lastPositionDifferences;
    std::deque<pose> recentPoses;
    
    // noise matrix
    Eigen::MatrixXd processNoise, measurementNoiseDepth, measurementNoiseDVL, measurementImuVelocity, measurementNoiseSlam, measurementNoiseUSBL, measurementMagnetometer;
    rclcpp::Time lastUpdateTime;

    bool firstMagMeasurement = true;
    double firstYawFromMag;
};


#endif //UNDERWATERSLAM_EKFDVL_H
