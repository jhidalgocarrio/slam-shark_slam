#ifndef _SHARK_SLAM_ISHRAK_HPP_
#define _SHARK_SLAM_ISHARK_HPP_

/** GTSAM factor types **/
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

/** GTSAM optimizers **/
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>

/** Base types **/
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/IMUSensors.hpp>

/** Standard library **/
#include <memory>

namespace shark_slam
{
    class iShark
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Structures having Eigen members

    protected:

        /******************************/
        /*** Control Flow Variables ***/
        /******************************/

        /** Initialization flag **/
        bool initialize;

        /** Orientation is available from IMU **/
        bool orientation_available;

        /** Needs optimization **/
        bool needs_optimization;

        /** Indices to identify estimates **/
        unsigned long int idx;

        /**************************/
        /***    Input Ports     ***/
        /**************************/
        base::samples::RigidBodyState gps_pose_samples;
        base::samples::IMUSensors imu_samples;
        base::samples::RigidBodyState orientation_samples;

        /**************************/
        /*** Property Variables ***/
        /**************************/

        /** IMU covariance matrices **/
        gtsam::Matrix33 measured_acc_cov;
        gtsam::Matrix33 measured_omega_cov; 
        gtsam::Matrix33 integration_error_cov;
        gtsam::Matrix33 bias_acc_cov;
        gtsam::Matrix33 bias_omega_cov;
        gtsam::Matrix66 bias_acc_omega_int;

        /** Initial pose uncertainties **/
        gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model;
        gtsam::noiseModel::Diagonal::shared_ptr velocity_noise_model;
        gtsam::noiseModel::Diagonal::shared_ptr bias_noise_model;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        Eigen::Affine3d tf_init, tf_init_inverse; /** initial pose **/

        gtsam::NavState prev_state, prop_state;
        gtsam::imuBias::ConstantBias prev_bias;
        std::pair<gtsam::Matrix66, gtsam::Matrix66> prev_cov;

        /** GTSAM Factor graph **/
        std::shared_ptr< gtsam::NonlinearFactorGraph > factor_graph;

        /** iSAM2 variable **/
        std::shared_ptr< gtsam::ISAM2 > isam;

        /** Values of the estimated quantities **/
        std::shared_ptr< gtsam::Values > initial_values;

        /** IMU preintegrated measurements (Imufactor or CombinedImufactor) **/
        std::shared_ptr< gtsam::PreintegrationType > imu_preintegrated;

        /**************************/
        /***   Output Ports     ***/
        /**************************/
        base::samples::RigidBodyState output_pose;

    public:

        /** Default constructor **/
        iShark();

        /** Default destructor **/
        ~iShark();

        /** initialization of the SLAM **/
        void initialization(Eigen::Affine3d &tf);

        /** Optimization step **/
        void optimize();

        void gps_pose_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_pose_samples_sample);

        void imu_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample);

        void orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample);

        const base::samples::RigidBodyState& getPose();

        void updatePose(const base::Time &time, gtsam::NavState &state, const gtsam::Matrix66 &cov_pose, const gtsam::Matrix66 &cov_velo);
    };

} // end namespace shark_slam

#endif
