//
// Created by tim on 23.02.21.
//

// This file defines the graph building, updating, and optimization logic in SLAM.
// It connects robot pose and sonar scan data into pose graph, adds constraints (edges),
// and optimizes it with iSAM2

#include "graphSlamSaveStructure.h"
#include "random"

/* This function creates a graph edge, which represents a relative pose constraint
* meaning how you move from prev node to this one
* General idea: "From A to B, I moved like this, with this much uncertainty."
* @param fromKey, toKey are indices of the two nodes
* @param positionDifference, rotationDifference are the relative transform
* @param covarianceMatrix: how uncertain this transform is
* @param typeOfEdge: odometry edges, loop closures, etc.
*/
void graphSlamSaveStructure::addEdge(int fromKey, int toKey, Eigen::Vector3d positionDifference,
                                Eigen::Quaterniond rotationDifference, Eigen::Matrix3d covarianceMatrix,
                                int typeOfEdge) {


    edge edgeToAdd(fromKey, toKey, positionDifference, rotationDifference, covarianceMatrix,
                   this->degreeOfFreedom, typeOfEdge, this->graph.size());

    // create a noise model to tell the optimizer how much trust to place in the measurement
    auto model = gtsam::noiseModel::Gaussian::Covariance(covarianceMatrix);
    // add a factor/edge between the keys
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

// This function adds a new robot pose to the graph (node/vertex)
// The vertex stores position, rotation, covariance, sonar intensities and timestamp
void graphSlamSaveStructure::addVertex(int key, const Eigen::Vector3d &positionVertex,
                                       const Eigen::Quaterniond &rotationVertex,
                                       const Eigen::Matrix3d &covarianceMatrix,
                                       intensityMeasurement intensityInput, double timeStamp,
                                       int typeOfVertex) {

    vertex vertexToAdd(key, positionVertex, rotationVertex, this->degreeOfFreedom,
                       intensityInput, covarianceMatrix, timeStamp, typeOfVertex);
    this->vertexList.push_back(vertexToAdd);

    // ADD BETTER INITIAL STATE
    // The graph must have initial guesses for all poses. This is needed for the nonlinear
    // optimization
    double tmpYaw = generalHelpfulTools::getRollPitchYaw(rotationVertex)[2];
    this->currentEstimate.insert(key, gtsam::Pose2(positionVertex[0], positionVertex[1], tmpYaw));
}

// graph optimization
void graphSlamSaveStructure::isam2OptimizeGraph(bool verbose, int numberOfUpdates) {

    // feed the newly added factors (edges) and guesses (nodes) into iSAM2
    this->isam->update(this->graph, this->currentEstimate);

    // Run additional optimization steps if required
    for (int i = 0; i < numberOfUpdates; i++) {
        this->isam->update();
    }

    // after convergence this holds the new optimized pose estimates
    this->currentEstimate = this->isam->calculateEstimate();

    // pose update loop
    // each vertex is updated with the new optimized position
    // given a new covariance, showing how uncertain the estimate is
    for (int i = 1; i < this->vertexList.size(); i++) {
        gtsam::Pose2 iterativePose = this->currentEstimate.at(this->vertexList[i].getKey()).cast<gtsam::Pose2>();
        this->vertexList.at(i).setPositionVertex(Eigen::Vector3d(iterativePose.x(), iterativePose.y(), 0));
        this->vertexList.at(i).setRotationVertex(
                generalHelpfulTools::getQuaternionFromRPY(0, 0, iterativePose.theta()));
        this->vertexList.at(i).setCovarianceMatrix(this->isam->marginalCovariance(this->vertexList.at(i).getKey()));
    }

    // the graph and factors are reset
    this->graph.resize(0);
    this->currentEstimate.clear();
}

std::vector<vertex> *graphSlamSaveStructure::getVertexList() {
    return &this->vertexList;
}

std::vector<edge> *graphSlamSaveStructure::getEdgeList() {
    return &this->edgeList;
}
