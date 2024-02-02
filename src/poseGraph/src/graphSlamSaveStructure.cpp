//
// Created by tim on 23.02.21.
//

#include "graphSlamSaveStructure.h"
#include "random"

void
graphSlamSaveStructure::addEdge(int fromKey, int toKey, Eigen::Vector3d positionDifference,
                                Eigen::Quaterniond rotationDifference, Eigen::Matrix3d covarianceMatrix,
                                int typeOfEdge) {


    edge edgeToAdd(fromKey, toKey, positionDifference, rotationDifference, covarianceMatrix,
                   this->degreeOfFreedom, typeOfEdge, this->graph.size());

    auto model = gtsam::noiseModel::Gaussian::Covariance(covarianceMatrix);
    // add a factor between the keys
    if (abs(toKey - fromKey) > 2) {
        this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(fromKey, toKey, gtsam::Pose2(
                                                                                edgeToAdd.getPositionDifference().x(), edgeToAdd.getPositionDifference().y(),
                                                                                generalHelpfulTools::getRollPitchYaw(edgeToAdd.getRotationDifference())[2]),
                                                                        model);//@TODO different Noise Model

    } else {
        this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(fromKey, toKey, gtsam::Pose2(
                                                                                edgeToAdd.getPositionDifference().x(), edgeToAdd.getPositionDifference().y(),
                                                                                generalHelpfulTools::getRollPitchYaw(edgeToAdd.getRotationDifference())[2]),
                                                                        model);

    }
    // add edge to edge list
    this->edgeList.push_back(edgeToAdd);

}

void graphSlamSaveStructure::addVertex(int key, const Eigen::Vector3d &positionVertex,
                                       const Eigen::Quaterniond &rotationVertex,
                                       const Eigen::Matrix3d &covarianceMatrix,
                                       intensityMeasurement intensityInput, double timeStamp,
                                       int typeOfVertex) {

    vertex vertexToAdd(key, positionVertex, rotationVertex, this->degreeOfFreedom,
                       intensityInput, covarianceMatrix, timeStamp, typeOfVertex);
    this->vertexList.push_back(vertexToAdd);
    //ADD BETTER INITIAL STATE
    double tmpYaw = generalHelpfulTools::getRollPitchYaw(rotationVertex)[2];
    this->currentEstimate.insert(key, gtsam::Pose2(positionVertex[0], positionVertex[1], tmpYaw));
}


void graphSlamSaveStructure::isam2OptimizeGraph(bool verbose, int numberOfUpdates) {



    this->isam->update(this->graph, this->currentEstimate);


    for (int i = 0; i < numberOfUpdates; i++) {
        this->isam->update();
    }


    this->currentEstimate = this->isam->calculateEstimate();

    for (int i = 1; i < this->vertexList.size(); i++) {
        gtsam::Pose2 iterativePose = this->currentEstimate.at(this->vertexList[i].getKey()).cast<gtsam::Pose2>();
        this->vertexList.at(i).setPositionVertex(Eigen::Vector3d(iterativePose.x(), iterativePose.y(), 0));
        this->vertexList.at(i).setRotationVertex(
                generalHelpfulTools::getQuaternionFromRPY(0, 0, iterativePose.theta()));
        this->vertexList.at(i).setCovarianceMatrix(this->isam->marginalCovariance(this->vertexList.at(i).getKey()));
    }

    this->graph.resize(0);
    this->currentEstimate.clear();
}

std::vector<vertex> *graphSlamSaveStructure::getVertexList() {
    return &this->vertexList;
}

std::vector<edge> *graphSlamSaveStructure::getEdgeList() {
    return &this->edgeList;
}