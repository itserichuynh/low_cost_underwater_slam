//
// Created by tim on 23.02.21.
//

#ifndef SIMULATION_BLUEROV_EDGE_H
#define SIMULATION_BLUEROV_EDGE_H

#include <Eigen/Geometry>
#include <iostream>

class edge {
public:
    edge(const int fromVertex, const int toVertex, const Eigen::Vector3d &positionDifference,
         const Eigen::Quaterniond &rotationDifference, const Eigen::Matrix3d &covarianceMatrix,
         int degreeOfFreedom, int typeOfEdge, int keyOfEdgeInGraph) {
        if (degreeOfFreedom == 3) {
            this->fromKey = fromVertex;
            this->toKey = toVertex;
            this->positionDifference = positionDifference;
            this->rotationDifference = rotationDifference;
            this->rotationDifference.normalize();
            this->covariancePosition = covariancePosition;
            this->covarianceQuaternion = covarianceQuaternion;
            this->keyOfEdgeInGraph = keyOfEdgeInGraph;

        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
        this->typeOfEdge = typeOfEdge;

    }

    edge(const edge &edgeToCopy) {
        this->fromKey = edgeToCopy.getFromKey();
        this->toKey = edgeToCopy.getToKey();
        this->covariancePosition = edgeToCopy.getCovariancePosition();
        this->covarianceQuaternion = edgeToCopy.getCovarianceQuaternion();
        this->positionDifference = edgeToCopy.getPositionDifference();
        this->rotationDifference = edgeToCopy.getRotationDifference();
        this->rotationDifference.normalize();
        this->typeOfEdge = edgeToCopy.getTypeOfEdge();
        this->keyOfEdgeInGraph = edgeToCopy.getKeyOfEdgeInGraph();
    }

    Eigen::Vector3d getCovariancePosition() const;


    double getCovarianceQuaternion() const;


    Eigen::Vector3d getPositionDifference() const;


    Eigen::Quaterniond getRotationDifference() const;


    int getFromKey() const;


    int getToKey() const;


    int getTypeOfEdge() const;


    int getKeyOfEdgeInGraph() const;


    Eigen::Matrix4d getTransformation() const;

private:
    int fromKey;
    int toKey;
    Eigen::Vector3d covariancePosition;//estimated covariance for this measurement in x y z
    double covarianceQuaternion;//estimated covariance for this measurement in q = w x y z (rotation)
    Eigen::Vector3d positionDifference;
    Eigen::Quaterniond rotationDifference;
    int typeOfEdge;
    int keyOfEdgeInGraph;
};


#endif //SIMULATION_BLUEROV_EDGE_H
