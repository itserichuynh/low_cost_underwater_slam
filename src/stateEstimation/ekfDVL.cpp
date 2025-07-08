//
// Created by jurobotics on 13.09.21.
//
#include "ekfDVL.h"

/*
* Prediction state of EKF
* This uses acceleration from imu to update pos and vel
*/
void ekfClassDVL::predictionImu(double xAccel, double yAccel, double zAccel, Eigen::Quaterniond currentRotation,
                                Eigen::Vector3d positionIMU,
                                rclcpp::Time timeStamp) {
    // for saving the current EKF pose difference in
    // currentStateBeforeUpdate is 12x1 [xpos, ypos, zpos, vx, vy, xz, r, p, y, rvel, pvel, yvel]
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();

    // raw acceleration
    Eigen::Vector3d bodyAcceleration(xAccel, yAccel, zAccel);

    
    Eigen::Vector3d angularVelocity(currentStateBeforeUpdate[9], currentStateBeforeUpdate[10],
                                    currentStateBeforeUpdate[11]);
    

    // @TODO I dont know if i should add or subtract 9.81 here bc from imu topic, linear acc in z is +9.81
    // I decided to go with negative since our reading from imu is +9.81

    // Compensate for gravity and centrifugal acceleration
    // + gravity to cancel it out
    // - w x (w x r): if imu is not at the center, it feels extra force when the robot rotates
    Eigen::Vector3d bodyAccelerationReal = (bodyAcceleration +
                                            this->getRotationVector().inverse() * Eigen::Vector3d(0, 0, -9.81) -
                                            angularVelocity.cross(angularVelocity.cross(positionIMU)));


    // bodyAcceleration has to be changed to correct rotation(body acceleration)
    // this converts it to the world frame (NED)
    Eigen::Vector3d localAcceleration = this->getRotationVector() *
                                        bodyAccelerationReal;


    double timeDiff = (timeStamp - this->stateOfEKF.timeLastPrediction).seconds();

    if (timeDiff > 0.4 || timeDiff < 0) {
        timeDiff = 0.4;
    }
    
    // A-Matrix is identity, except for the entries of transition between velocity and position.(there time diff since last prediction)
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(12, 12);
    // State Transition Matrix
    // Position updates from velocity
    A(0, 3) = timeDiff;
    A(1, 4) = timeDiff;
    A(2, 5) = timeDiff;
    // Orientation update from angular velocity
    A(6, 9) = timeDiff;
    A(7, 10) = timeDiff;
    A(8, 11) = timeDiff;
    Eigen::VectorXd state = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();
    Eigen::VectorXd inputMatrix = Eigen::VectorXd::Zero(12);
    inputMatrix(3) = localAcceleration(0) * timeDiff;
    inputMatrix(4) = localAcceleration(1) * timeDiff;
    inputMatrix(5) = localAcceleration(2) * timeDiff;

    // + inputMatrix is to update linear velocity
    state = A * state + inputMatrix;
    //look for wrap around
    if (state(8) > M_PI) {
        state(8) = state(8) - 2 * M_PI;
    }
    if (state(8) < -M_PI) {
        state(8) = state(8) + 2 * M_PI;
    }

    this->stateOfEKF.applyState(state);
    //update covariance
    this->stateOfEKF.covariance = A * this->stateOfEKF.covariance * A.transpose() + processNoise;
    this->stateOfEKF.timeLastPrediction = timeStamp;

}

void ekfClassDVL::updateIMU(double roll, double pitch, double xAngularVel, double yAngularVel, double zAngularVel,
                            Eigen::Quaterniond currentRotation,
                            rclcpp::Time timeStamp) {
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();

    Eigen::VectorXd innovation;
    //change from system to body system
    Eigen::Vector3d velocityBodyAngular(xAngularVel, yAngularVel, zAngularVel);
    // velocityAngular has to be changed to correct rotation(world velocityAngular)
    Eigen::Vector3d velocityLocalAngular = this->getRotationVectorWithoutYaw() * velocityBodyAngular;

    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(6) = roll;
    z(7) = pitch;
    z(9) = velocityLocalAngular(0);
    z(10) = velocityLocalAngular(1);
    z(11) = velocityLocalAngular(2);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(6, 6) = 1;
    H(7, 7) = 1;
    H(9, 9) = 1;
    H(10, 10) = 1;
    H(11, 11) = 1;

    // make sure roll angles arent passed the +- pi boundary
    if (roll > 0 && currentStateBeforeUpdate[6] < 0) {
        currentStateBeforeUpdate[6] = currentStateBeforeUpdate[6] + M_PI * 2;
    }
    if (roll < 0 && currentStateBeforeUpdate[6] > 0) {
        currentStateBeforeUpdate[6] = currentStateBeforeUpdate[6] - M_PI * 2;
    }
    innovation = this->innovationStateDiff(z, H, currentStateBeforeUpdate);//also called y
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementImuVelocity;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = currentStateBeforeUpdate + K * innovation;

    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;

}

void ekfClassDVL::updateMagnetometer(double x_mag, double y_mag, double z_mag, rclcpp::Time timeStamp) {
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();

    if (this->firstMagMeasurement) {
        this->firstYawFromMag = std::atan2(y_mag, x_mag);
        this->firstMagMeasurement = false;
    }

    Eigen::VectorXd innovation;
    // //change from system to body system
    // Eigen::Vector3d velocityBodyAngular(xAngularVel, yAngularVel, zAngularVel);
    // // velocityAngular has to be changed to correct rotation(world velocityAngular)
    // Eigen::Vector3d velocityLocalAngular = this->getRotationVectorWithoutYaw() * velocityBodyAngular;

    // Compute yaw from magnetometer
    // double yaw_measured = std::atan2(y_mag, x_mag) - this->firstYawFromMag;

    // if (yaw_measured > M_PI) yaw_measured -= 2*M_PI;
    // if (yaw_measured < -M_PI) yaw_measured += 2*M_PI;
    double yaw_measured = generalHelpfulTools::angleDiff(std::atan2(-y_mag, x_mag), this->firstYawFromMag);

    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(8) = yaw_measured;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(8, 8) = 1;

    // // make sure roll angles arent passed the +- pi boundary
    // if (roll > 0 && currentStateBeforeUpdate[6] < 0) {
    //     currentStateBeforeUpdate[6] = currentStateBeforeUpdate[6] + M_PI * 2;
    // }
    // if (roll < 0 && currentStateBeforeUpdate[6] > 0) {
    //     currentStateBeforeUpdate[6] = currentStateBeforeUpdate[6] - M_PI * 2;
    // }
    // innovation = this->innovationStateDiff(z, H, currentStateBeforeUpdate);//also called y

    innovation = z - H * this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementMagnetometer;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = currentStateBeforeUpdate + K * innovation;

    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;

}

void
ekfClassDVL::updateDVL(double xVel, double yVel, double zVel, Eigen::Quaterniond rotationOfDVL,
                       Eigen::Vector3d positionDVL, rclcpp::Time timeStamp) {
    //for saving the current EKF pose difference in

    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();

    Eigen::Vector3d velocityBodyLinear(xVel, yVel, zVel);
    velocityBodyLinear = rotationOfDVL * velocityBodyLinear;

    //reduce velocity dependent on rotation and position of dvl
    // if DVl is not mounted at the center of the rotation, when the robot rotates,
    // the dvl will sense extra vel 
    Eigen::Vector3d angularVelocity(currentStateBeforeUpdate[9], currentStateBeforeUpdate[10],
                                    currentStateBeforeUpdate[11]);
    velocityBodyLinear = velocityBodyLinear - angularVelocity.cross(positionDVL);
    
    // velocityAngular has to be changed to correct rotation(world velocityAngular)
    Eigen::Vector3d velocityLocalLinear = this->getRotationVector() * velocityBodyLinear;

    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(3) = velocityLocalLinear(0);
    z(4) = velocityLocalLinear(1);
    z(5) = velocityLocalLinear(2);// removed because z is not important. velocityLocalLinear(2);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(3, 3) = 1;
    H(4, 4) = 1;
    H(5, 5) = 1;
    innovation = z - H * this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();//also y
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementNoiseDVL;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel() + K * innovation;
    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;
}

void ekfClassDVL::updateUSBL(double xPos, double yPos, double zPos, rclcpp::Time timeStampp) {
    //for saving the current EKF pose difference in

    // Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();

    // Eigen::Vector3d velocityBodyLinear(xVel, yVel, zVel);
    // velocityBodyLinear = rotationOfDVL * velocityBodyLinear;

    // //reduce velocity dependent on rotation and position of dvl
    // // if DVl is not mounted at the center of the rotation, when the robot rotates,
    // // the dvl will sense extra vel 
    // Eigen::Vector3d angularVelocity(currentStateBeforeUpdate[9], currentStateBeforeUpdate[10],
    //                                 currentStateBeforeUpdate[11]);
    // velocityBodyLinear = velocityBodyLinear - angularVelocity.cross(positionDVL);
    
    // // velocityAngular has to be changed to correct rotation(world velocityAngular)
    // Eigen::Vector3d velocityLocalLinear = this->getRotationVector() * velocityBodyLinear;

    Eigen::Vector3d position(xPos, yPos, zPos);
    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12); // matrix to hold measurements
    z(0) = position(0);
    z(1) = position(1);
    //z(2) = position(2);// removed because our usbl can only give x and y info
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(0, 0) = 1;
    H(1, 1) = 1;
    // H(2, 2) = 1;
    innovation = z - H * this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();//also y
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementNoiseUSBL;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel() + K * innovation;
    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;
}

void
ekfClassDVL::updateHeight(double depth, rclcpp::Time timeStamp) {
    if (isnan(depth)) {
        depth = 0;
    }
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();


    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(2) = depth;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(2, 2) = 1;
    innovation = z - H * this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();//also y
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementNoiseDepth;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel() + K * innovation;
    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;
}

pose ekfClassDVL::getState() {
    return this->stateOfEKF;
}

/*
* returns robot's orientation from body to world or wrt world coor
*/
Eigen::Quaterniond ekfClassDVL::getRotationVector() {
    double rotationX = this->stateOfEKF.rotation.x();
    double rotationY = this->stateOfEKF.rotation.y();
    double rotationZ = this->stateOfEKF.rotation.z();

    Eigen::Quaterniond rotDiff = generalHelpfulTools::getQuaternionFromRPY(rotationX, rotationY, rotationZ);
    return rotDiff;
}

Eigen::Quaterniond ekfClassDVL::getRotationVectorWithoutYaw() {
    double rotationX = this->stateOfEKF.rotation.x();
    double rotationY = this->stateOfEKF.rotation.y();
    Eigen::Quaterniond rotDiff = generalHelpfulTools::getQuaternionFromRPY(rotationX, rotationY, 0);
    return rotDiff;
}

Eigen::VectorXd ekfClassDVL::innovationStateDiff(Eigen::VectorXd z, Eigen::MatrixXd H,
                                                 Eigen::VectorXd currentStateBeforeUpdate) {//xyz vxvyvz rpy rvel pvel yvel
    Eigen::VectorXd innovation;
    innovation = z - H * currentStateBeforeUpdate;//also y

    // 
    innovation(6) = generalHelpfulTools::angleDiff(z(6), H(6, 6) * currentStateBeforeUpdate(6));
    innovation(7) = generalHelpfulTools::angleDiff(z(7), H(7, 7) * currentStateBeforeUpdate(7));
    innovation(8) = generalHelpfulTools::angleDiff(z(8), H(8, 8) * currentStateBeforeUpdate(8));

    return innovation;
}
