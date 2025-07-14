//
// Created by jurobotics on 13.09.21.

#include "ekfDVL.h"
#include "rclcpp/rclcpp.hpp"

// just for tricking compiler
#include "waterlinked_a50/msg/transducer_report_stamped.hpp"
#include "usbl_seatrac_msgs/msg/position_stamped.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "px4_msgs/msg/sensor_combined.hpp"


#include <thread>

static constexpr double CONSTANTS_ONE_G = 9.80665;

class RosClassEKF : public rclcpp::Node {
public:
    RosClassEKF() : Node("ekfNode"), currentEkf(rclcpp::Clock(RCL_ROS_TIME).now()) {
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        this->declare_parameter("xPositionDVL", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("yPositionDVL", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("zPositionDVL", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("yawRotationDVL", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("xPositionIMU", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("yPositionIMU", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("zPositionIMU", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("yawRotationIMU", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("zPositionDepthSensor", rclcpp::PARAMETER_DOUBLE);



        this->firstMessage = true;
        this->rotationOfDVL = Eigen::AngleAxisd(this->get_parameter("yawRotationDVL").as_double(),
                                                Eigen::Vector3d::UnitZ());//yaw rotation for correct alignment of DVL data; quick fix set to default
        this->positionIMU = Eigen::Vector3d(this->get_parameter("xPositionIMU").as_double(),
                                            this->get_parameter("yPositionIMU").as_double(),
                                            this->get_parameter("zPositionIMU").as_double());
        this->positionDVL = Eigen::Vector3d(this->get_parameter("xPositionDVL").as_double(),
                                            this->get_parameter("yPositionDVL").as_double(),
                                            this->get_parameter("zPositionDVL").as_double());


        this->subscriberIMU = this->create_subscription<sensor_msgs::msg::Imu>("mavros/imu/data", qos,
                                                                               std::bind(&RosClassEKF::imuCallback,
                                                                                         this, std::placeholders::_1));

        this->subscriberMagnetometer = this->create_subscription<sensor_msgs::msg::MagneticField>("mavros/imu/mag", qos,
                                                                               std::bind(&RosClassEKF::magnetometerCallback,
                                                                                         this, std::placeholders::_1));

        this->subscriberDVL = this->create_subscription<waterlinked_a50::msg::TransducerReportStamped>(
                "/velocity_estimate", qos, std::bind(&RosClassEKF::DVLCallbackDVL, this, std::placeholders::_1));

        this->subscriberUSBL = this->create_subscription<usbl_seatrac_msgs::msg::PositionStamped>("/usbl/data", qos, std::bind(&RosClassEKF::USBLCallback, this, std::placeholders::_1));

        this->subscriberDepthSensorBaroSensorTube = this->create_subscription<sensor_msgs::msg::FluidPressure>(
                "/pressure/data", qos,
                std::bind(
                        &RosClassEKF::depthSensorBaroSensorTubeCallback,
                        this,
                        std::placeholders::_1));

        this->publisherPoseEkf = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "publisherPoseEkf", qos);
        this->publisherTwistEkf = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
                "publisherTwistEkf", qos);
    }

private:
    ekfClassDVL currentEkf;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriberIMU;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscriberPX4IMU;

    rclcpp::Subscription<usbl_seatrac_msgs::msg::PositionStamped>::SharedPtr subscriberUSBL;

    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr subscriberMagnetometer;

    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr subscriberDepthSensorBaroSensorTube;
    rclcpp::Subscription<waterlinked_a50::msg::TransducerReportStamped>::SharedPtr subscriberDVL;


    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisherPoseEkf;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisherTwistEkf;
    std::mutex updateEKFMutex;
    Eigen::Quaterniond rotationOfDVL;
    Eigen::Vector3d positionIMU, positionDVL;

    double pressureWhenStarted;
    bool firstMessage;

    void imuCallbackHelper(const sensor_msgs::msg::Imu::SharedPtr msg) {

        // Rotate 90 CCW
        // @TODO Do I need this at all or is it already handled in mavros?
        // Well looks like mavros has it in NED already! So we dont need to do this anymore
        Eigen::Matrix3d transformationX180DegreeRotationMatrix;
        Eigen::AngleAxisd rotation_vector2(0.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));

        transformationX180DegreeRotationMatrix = rotation_vector2.toRotationMatrix();

        Eigen::Vector3d acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                     msg->linear_acceleration.z);
        acceleration = transformationX180DegreeRotationMatrix * acceleration;

        Eigen::Vector3d rotationVel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        rotationVel = transformationX180DegreeRotationMatrix * rotationVel;

        // reconstruct a new imu message
        sensor_msgs::msg::Imu newMsg;
        newMsg.header = msg->header;
        newMsg.angular_velocity.x = rotationVel.x();
        newMsg.angular_velocity.y = rotationVel.y();
        newMsg.angular_velocity.z = rotationVel.z();

        newMsg.linear_acceleration.x = acceleration.x();
        newMsg.linear_acceleration.y = acceleration.y();
        newMsg.linear_acceleration.z = acceleration.z();

        // estimate roll and pitch from accelerometer 

        // we kinda ignore the next line tbh
        Eigen::Quaterniond rotationRP(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        // Eigen::Vector3d rollPitchYaw = this->getRollPitchYaw(rotationRP.inverse());
        double rollIMUACCEL = atan2(-msg->linear_acceleration.y, msg->linear_acceleration.z);
        double pitchIMUACCEL = atan2(msg->linear_acceleration.x,
                                     sqrt(msg->linear_acceleration.y * msg->linear_acceleration.y +
                                          msg->linear_acceleration.z * msg->linear_acceleration.z));

        // we cant really get yaw from accelerometer as gravity always points down
        rotationRP = getQuaternionFromRPY(rollIMUACCEL, pitchIMUACCEL, 0);

        newMsg.orientation.x = rotationRP.x();
        newMsg.orientation.y = rotationRP.y();
        newMsg.orientation.z = rotationRP.z();
        newMsg.orientation.w = rotationRP.w();

        Eigen::Quaterniond tmpRot;
        tmpRot.x() = newMsg.orientation.x;
        tmpRot.y() = newMsg.orientation.y;
        tmpRot.z() = newMsg.orientation.z;
        tmpRot.w() = newMsg.orientation.w;

        currentEkf.predictionImuNoVelocity(newMsg.linear_acceleration.x, newMsg.linear_acceleration.y,
                                 newMsg.linear_acceleration.z,
                                 tmpRot, this->positionIMU,
                                 newMsg.header.stamp);

        Eigen::Vector3d euler = generalHelpfulTools::getRollPitchYaw(tmpRot);// roll pitch yaw


        currentEkf.updateIMUNoVelocity(euler.x(), euler.y(), newMsg.angular_velocity.x, newMsg.angular_velocity.y,
                             newMsg.angular_velocity.z, tmpRot, newMsg.header.stamp);
        pose currentStateEkf = currentEkf.getState();
        geometry_msgs::msg::PoseWithCovarianceStamped poseMsg;
        poseMsg.header.frame_id = "map_ned";
        poseMsg.pose.pose.position.x = currentStateEkf.position.x();
        poseMsg.pose.pose.position.y = currentStateEkf.position.y();
        poseMsg.pose.pose.position.z = currentStateEkf.position.z();
        Eigen::Quaterniond rotDiff = currentEkf.getRotationVector();
        poseMsg.pose.pose.orientation.x = rotDiff.x();
        poseMsg.pose.pose.orientation.y = rotDiff.y();
        poseMsg.pose.pose.orientation.z = rotDiff.z();
        poseMsg.pose.pose.orientation.w = rotDiff.w();
        poseMsg.header.stamp = msg->header.stamp;
        this->publisherPoseEkf->publish(poseMsg);
        geometry_msgs::msg::TwistWithCovarianceStamped twistMsg;
        twistMsg.header.frame_id = "map_ned";
        twistMsg.twist.twist.linear.x = currentStateEkf.velocity.x();
        twistMsg.twist.twist.linear.y = currentStateEkf.velocity.y();
        twistMsg.twist.twist.linear.z = currentStateEkf.velocity.z();
        twistMsg.twist.twist.angular.x = currentStateEkf.angleVelocity.x();
        twistMsg.twist.twist.angular.y = currentStateEkf.angleVelocity.y();
        twistMsg.twist.twist.angular.z = currentStateEkf.angleVelocity.z();
        twistMsg.header.stamp = newMsg.header.stamp;
        this->publisherTwistEkf->publish(twistMsg);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        //change the orientation of the IMU message

        this->updateEKFMutex.lock();
        this->imuCallbackHelper(msg);
        this->updateEKFMutex.unlock();
    }

    void magnetometerCallbackHelper(const sensor_msgs::msg::MagneticField msg) {
        this->currentEkf.updateMagnetometer(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z, msg.header.stamp);
    }

    void magnetometerCallback(const sensor_msgs::msg::MagneticField msg) {
        this->updateEKFMutex.lock();
        this->magnetometerCallbackHelper(msg);
        this->updateEKFMutex.unlock();
    }

    void DVLCallbackDVLHelper(const waterlinked_a50::msg::TransducerReportStamped::SharedPtr msg) {
        if (!msg->report.velocity_valid || msg->report.status != 0) {
            //if we don't know anything, the speed of the ekf should just go to 0, else the IMU gives direction. But should not happen anyway.
            this->currentEkf.updateDVL(0, 0, 0, this->rotationOfDVL, this->positionDVL, rclcpp::Time(msg->timestamp));
        } else {
            this->currentEkf.updateDVL(msg->report.vx, msg->report.vy, msg->report.vz, this->rotationOfDVL,
                                       this->positionDVL,
                                       rclcpp::Time(msg->timestamp));
        }
        return;
    }

    void DVLCallbackDVL(const waterlinked_a50::msg::TransducerReportStamped::SharedPtr msg) {
        this->updateEKFMutex.lock();
        this->DVLCallbackDVLHelper(msg);
        this->updateEKFMutex.unlock();
    }

    void usblCallbackHelper(const usbl_seatrac_msgs::msg::PositionStamped::SharedPtr msg) {
        if (!msg->pos_valid) {
            //if we don't know anything, the position of the ekf should just go to 0. But should not happen anyway.
            this->currentEkf.updateUSBL(0, 0, 0, msg->header.stamp);
        } else {
            this->currentEkf.updateUSBL(msg->x, msg->y, msg->z, msg->header.stamp);
        }
        return;
    }

    void USBLCallback(const usbl_seatrac_msgs::msg::PositionStamped::SharedPtr msg) {
        this->updateEKFMutex.lock();
        this->usblCallbackHelper(msg);
        this->updateEKFMutex.unlock();
    }

    void depthSensorBaroSensorTubeCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {

        if (this->firstMessage) {
            this->pressureWhenStarted = msg->fluid_pressure;
            this->firstMessage = false;
            return;
        }


        double currentHeight =
                ((msg->fluid_pressure - this->pressureWhenStarted) * 0.01f) / (CONSTANTS_ONE_G * 1000.0f);

        this->updateEKFMutex.lock();
        this->depthSensorHelper(currentHeight, msg->header.stamp);
        this->updateEKFMutex.unlock();
    }



    void depthSensorHelper(double height, rclcpp::Time rosTime) {
        this->currentEkf.updateHeight(height, rosTime);
    }


    Eigen::Vector3d getRollPitchYaw(Eigen::Quaterniond quat) {
        tf2::Quaternion tmp(quat.x(), quat.y(), quat.z(), quat.w());
        tf2::Matrix3x3 m(tmp);
        double r, p, y;
        m.getRPY(r, p, y);
        Eigen::Vector3d returnVector(r, p, y);
        return returnVector;
    }

    Eigen::Quaterniond getQuaternionFromRPY(double roll, double pitch, double yaw) {
        tf2::Quaternion qtf2;
        qtf2.setRPY(roll, pitch, yaw);
        Eigen::Quaterniond q;
        q.x() = qtf2.x();
        q.y() = qtf2.y();
        q.z() = qtf2.z();
        q.w() = qtf2.w();
        return q;
    };

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosClassEKF>());


    rclcpp::shutdown();

    return (0);
}
