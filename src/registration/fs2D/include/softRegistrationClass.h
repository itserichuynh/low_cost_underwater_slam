//
// Created by tim-linux on 01.03.22.
//

#ifndef FS2D_softRegistrationClass_H
#define FS2D_softRegistrationClass_H

//#include "softRegistrationClass.h"
#include "softCorrelationClass.h"
#include "PeakFinder.h"
#include "generalHelpfulTools.h"
//#include "slamToolsRos.h"

//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/common/transforms.h>
//#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/core.hpp>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_points_d_traits_2.h>
#include <CGAL/Random.h>

#include <iostream>
#include <fstream>
#include <findpeaks/mask.hpp>
#include <findpeaks/persistence.hpp>
#include "fftw3.h"

struct rotationPeakfs2D {
    double angle;
    double peakCorrelation;
    double covariance;
};

struct translationPeakfs2D {
    Eigen::Vector2d translationSI;
    Eigen::Vector2i translationVoxel;
    double peakHeight;
    double persistenceValue;
    Eigen::Matrix2d covariance;
};

//struct transformationPeakfs2D {
//    std::vector<translationPeakfs2D> potentialTranslations;
//    rotationPeakfs2D potentialRotation;
//};

class softRegistrationClass {
public:
    softRegistrationClass(int N, int bwOut, int bwIn, int degLim) : sofftCorrelationObject(N, bwOut, bwIn,
                                                                                                degLim) {
        this->N = N;
        this->correlationN = N * 2 - 1;
        this->bwOut = bwOut;
        this->bwIn = bwIn;
        this->degLim = degLim;
        this->resultingCorrelationDouble = (double *) malloc(sizeof(double) * this->correlationN * this->correlationN);
        this->resultingCorrelationComplex = fftw_alloc_complex(8 * bwOut * bwOut * bwOut);
//        this->resultingPhaseDiff2D = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N);
        this->resultingPhaseDiff2DCorrelation = (fftw_complex *) fftw_malloc(
                sizeof(fftw_complex) * this->correlationN * this->correlationN);
//        this->resultingShiftPeaks2D = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N);
        this->resultingShiftPeaks2DCorrelation = (fftw_complex *) fftw_malloc(
                sizeof(fftw_complex) * this->correlationN * this->correlationN);

        this->magnitude1Shifted = (double *) malloc(sizeof(double) * N * N * N);
        this->magnitude2Shifted = (double *) malloc(sizeof(double) * N * N * N);
        this->voxelData1 = (double *) malloc(sizeof(double) * N * N * N);
        this->voxelData2 = (double *) malloc(sizeof(double) * N * N * N);
        this->spectrumOut = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
        this->spectrumOutCorrelation = (fftw_complex *) fftw_malloc(
                sizeof(fftw_complex) * this->correlationN * this->correlationN * this->correlationN);
        this->phase1 = (double *) malloc(sizeof(double) * N * N * N);
        this->phase2 = (double *) malloc(sizeof(double) * N * N * N);
        this->magnitude1 = (double *) malloc(sizeof(double) * N * N * N);
        this->magnitude2 = (double *) malloc(sizeof(double) * N * N * N);
        this->phase1Correlation = (double *) malloc(
                sizeof(double) * this->correlationN * this->correlationN * this->correlationN);
        this->phase2Correlation = (double *) malloc(
                sizeof(double) * this->correlationN * this->correlationN * this->correlationN);
        this->magnitude1Correlation = (double *) malloc(
                sizeof(double) * this->correlationN * this->correlationN * this->correlationN);
        this->magnitude2Correlation = (double *) malloc(
                sizeof(double) * this->correlationN * this->correlationN * this->correlationN);
        resampledMagnitudeSO3_1 = (double *) malloc(sizeof(double) * N * N);
        resampledMagnitudeSO3_2 = (double *) malloc(sizeof(double) * N * N);
        resampledMagnitudeSO3_1TMP = (double *) malloc(sizeof(double) * N * N);
        resampledMagnitudeSO3_2TMP = (double *) malloc(sizeof(double) * N * N);
        inputSpacialData = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
        inputSpacialDataCorrelation = (fftw_complex *) fftw_malloc(
                sizeof(fftw_complex) * this->correlationN * this->correlationN * this->correlationN);

        planFourierToVoxel2DCorrelation = fftw_plan_dft_2d(this->correlationN, this->correlationN,
                                                           resultingPhaseDiff2DCorrelation,
                                                           resultingShiftPeaks2DCorrelation, FFTW_BACKWARD,
                                                           FFTW_ESTIMATE);



        planVoxelToFourier2D = fftw_plan_dft_2d(N, N, inputSpacialData,
                                                spectrumOut, FFTW_FORWARD, FFTW_ESTIMATE);
        planVoxelToFourier2DCorrelation = fftw_plan_dft_2d(this->correlationN, this->correlationN,
                                                           inputSpacialDataCorrelation,
                                                           spectrumOutCorrelation, FFTW_FORWARD, FFTW_ESTIMATE);
    }

        ~softRegistrationClass() {
        sofftCorrelationObject.~softCorrelationClass();

    }


    double
    getSpectrumFromVoxelData2D(double voxelData[], double magnitude[], double phase[], bool gaussianBlur = false);

    double
    sofftRegistrationVoxel2DRotationOnly(double voxelData1Input[], double voxelData2Input[], double goodGuessAlpha,double &covariance);

    std::vector<rotationPeakfs2D>
    sofftRegistrationVoxel2DListOfPossibleRotations(double voxelData1Input[], double voxelData2Input[], bool multipleRadii = false,
                                                    bool useClahe = true, bool useHamming = true);

    Eigen::Matrix4d registrationOfTwoVoxelsSOFFTFast(double voxelData1Input[],
                                                     double voxelData2Input[],
                                                     Eigen::Matrix4d &initialGuess,Eigen::Matrix3d &covarianceMatrix,
                                                     bool useInitialAngle, bool useInitialTranslation,
                                                     double cellSize,
                                                     bool useGauss,
                                                     double potentialNecessaryForPeak = 0.1);

//    std::vector<transformationPeakfs2D> registrationOfTwoVoxelsSOFFTAllSoluations(double voxelData1Input[],
//                                                                                  double voxelData2Input[],
//                                                                                  double cellSize,
//                                                                                  bool useGauss,
//                                                                                  bool debug = false,
//                                                                                  double potentialNecessaryForPeak = 0.1,
//                                                                                  bool multipleRadii = false,
//                                                                                  bool useClahe = true,
//                                                                                  bool useHamming = true);

    double getSpectrumFromVoxelData2DCorrelation(double voxelData[], double magnitude[], double phase[],
                                                 bool gaussianBlur, double normalizationFactor);

    std::vector<translationPeakfs2D> sofftRegistrationVoxel2DTranslationAllPossibleSolutions(double voxelData1Input[],
                                                                                             double voxelData2Input[],
                                                                                             double cellSize,
                                                                                             double normalizationFactor,
                                                                                             int numberOfRotationForDebug = 0,
                                                                                             double potentialNecessaryForPeak = 0.1);

//    std::vector<translationPeakfs2D>
//    peakDetectionOf2DCorrelationSimpleDouble1D(double maximumCorrelation, double cellSize, int impactOfNoiseFactor = 2,
//                                               double percentageOfMaxCorrelationIgnored = 0.10);
//
//    std::vector<translationPeakfs2D>
//    peakDetectionOf2DCorrelationOpenCVHoughTransform(double maximumCorrelation, double cellSize,
//                                                     int impactOfNoiseFactor = 2,
//                                                     double percentageOfMaxCorrelationIgnored = 0.10);

    bool isPeak(cv::Mat mx[], std::vector<cv::Point> &conn_points);

//    cv::Mat imregionalmax(cv::Mat &src);

    double normalizationFactorCalculation(int x, int y);

//    cv::Mat opencv_imextendedmax(cv::Mat &inputMatrix, double hParam);

    void imextendedmax_imreconstruct(cv::Mat g, cv::Mat f, cv::Mat &dest);

    std::vector<translationPeakfs2D>
    peakDetectionOf2DCorrelationFindPeaksLibrary(double cellSize, double potentialNecessaryForPeak = 0.1,
                                                 double ignoreSidesPercentage = 0.05);

//    int getSizeOfRegistration();


private://here everything is created. malloc is done in the constructor




    int N, correlationN;//describes the size of the overall voxel system + correlation N
    int bwOut, bwIn, degLim;
    double *voxelData1;
    double *voxelData2;
    fftw_complex *spectrumOut;
    fftw_complex *spectrumOutCorrelation;
    double *magnitude1;
    double *magnitude2;
    double *magnitude1Correlation;
    double *magnitude2Correlation;
    double *phase1;
    double *phase2;
    double *phase1Correlation;
    double *phase2Correlation;
    double *magnitude1Shifted;
    double *magnitude2Shifted;
    double *resampledMagnitudeSO3_1;
    double *resampledMagnitudeSO3_2;
    double *resampledMagnitudeSO3_1TMP;
    double *resampledMagnitudeSO3_2TMP;
    softCorrelationClass sofftCorrelationObject;
    fftw_complex *resultingCorrelationComplex;
//    fftw_complex *resultingPhaseDiff2D;
    fftw_complex *resultingPhaseDiff2DCorrelation;
//    fftw_complex *resultingShiftPeaks2D;
    fftw_complex *resultingShiftPeaks2DCorrelation;
    double *resultingCorrelationDouble;
    fftw_complex *inputSpacialData;
    fftw_complex *inputSpacialDataCorrelation;
//    fftw_plan planVoxelToFourier3D;
    fftw_plan planVoxelToFourier2D;
    fftw_plan planVoxelToFourier2DCorrelation;
//    fftw_plan planFourierToVoxel2D;
    fftw_plan planFourierToVoxel2DCorrelation;
};


#endif //UNDERWATERSLAM_softRegistrationClass_H
