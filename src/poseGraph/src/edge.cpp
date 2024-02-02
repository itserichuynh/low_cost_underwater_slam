
//
// Created by tim on 23.02.21.
//

#include "edge.h"

int edge::getFromKey() const {
    return this->fromKey;
}

int edge::getToKey() const {
    return this->toKey;
}


Eigen::Vector3d edge::getCovariancePosition() const {
    return covariancePosition;
}


double edge::getCovarianceQuaternion() const {
    return covarianceQuaternion;
}


Eigen::Vector3d edge::getPositionDifference() const {
    return positionDifference;
}

Eigen::Quaterniond edge::getRotationDifference() const {
    return rotationDifference;
}

int edge::getTypeOfEdge() const {
    return typeOfEdge;
}



Eigen::Matrix4d edge::getTransformation() const{
    Eigen::Matrix4d transformation;

    transformation << 1, 0, 0, this->positionDifference.x(),
            0, 1, 0, this->positionDifference.y(),
            0, 0, 1, this->positionDifference.z(),
            0, 0, 0, 1;//transformation missing currently
    Eigen::Matrix3d m(this->rotationDifference.toRotationMatrix());
    transformation.block<3, 3>(0, 0) = m;
    return transformation;
}

int edge::getKeyOfEdgeInGraph() const{
    return(this->keyOfEdgeInGraph);
}