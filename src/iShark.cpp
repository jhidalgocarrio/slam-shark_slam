#include "iShark.hpp"
#include <iostream>

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

//#define DEBUG_PRINTS 1

using namespace shark_slam;

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

iShark::iShark()
{
    /******************************/
    /*** Control Flow Variables ***/
    /******************************/

    this->initialize = false; //No initialization
    this->needs_optimization = false; //No optimize until we receive an IMU measurement
    this->orientation_available = false; //No orientation until we receive a orientation measurement
    this->idx = 0;

    /***************************/
    /**    IMU Noise Init     **/
    /***************************/

    /** TODO: change from config values **/
    double accel_noise_sigma = 0.3924;
    double gyro_noise_sigma = 0.205689024915;
    double accel_bias_rw_sigma = 0.04905;
    double gyro_bias_rw_sigma = 0.001454441043;

    /** Covariance matrices **/
    this->measured_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
    this->measured_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
    this->integration_error_cov = gtsam::Matrix33::Identity(3,3) * 1e-8; // error committed in integrating position from velocities
    this->bias_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
    this->bias_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
    this->bias_acc_omega_int = gtsam::Matrix::Identity(6,6) * 1e-5; // error in the bias used for preintegration

    /** Set initial time to zero for input port variables**/
    this->gps_pose_samples.time = base::Time::fromSeconds(0);
    this->imu_samples.time = base::Time::fromSeconds(0);
    this->orientation_samples.time = base::Time::fromSeconds(0);


    #ifdef DEBUG_PRINTS
    std::cout<<"** iSHARK WELCOME!! **\n";
    #endif
}

iShark::~iShark()
{
    /** Reset GTSAM **/
    this->factor_graph.reset();
    this->initial_values.reset();
    this->imu_preintegrated.reset();
    this->isam.reset();

    /** Reset estimation **/
    this->idx = 0;

}

void iShark::initialization(Eigen::Affine3d &tf)
{

    /***************************/
    /**  GTSAM Initialization **/
    /***************************/

    std::cout<<"INITIAL POSE:\n";
    std::cout<<tf.matrix()<<"\n";

    /** Initial idx **/
    this->idx = 0;

    /** Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps to maintain proper linearization
    and efficient variable ordering, iSAM2 performs partial relinearization/reordering at each step. A parameter
    structure is available that allows the user to set various properties, such as the relinearization threshold
    and type of linear solver. For this example, we we set the relinearization threshold small so the iSAM2 result
    will approach the batch result. **/
    gtsam::ISAM2Params isam_parameters;
    isam_parameters.relinearizeThreshold = 0.1;
    isam_parameters.relinearizeSkip = 10;
    this->isam.reset(new gtsam::ISAM2(isam_parameters));

    /** Initial prior pose **/
    gtsam::Pose3 prior_pose(gtsam::Rot3(tf.rotation()), gtsam::Point3(tf.translation()));

    /** Initial prior velocity (assume zero initial velocity)**/
    gtsam::Vector3 prior_velocity = Eigen::Vector3d::Zero();

    /** Initial prior bias (assume zero initial bias)**/
    gtsam::imuBias::ConstantBias prior_imu_bias;

    /** Create the estimated values **/
    this->initial_values.reset(new gtsam::Values());

    /** Insert the initial values (pose, velocity and bias) **/
    this->initial_values->insert(X(this->idx), prior_pose);
    this->initial_values->insert(V(this->idx), prior_velocity);
    this->initial_values->insert(B(this->idx), prior_imu_bias);

    /** Assemble prior noise model and insert it to the graph. **/
    this->pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
    this->velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3,0.1); // m/s
    this->bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6,1e-3);


    /** Create the factor graph **/
    this->factor_graph.reset(new gtsam::NonlinearFactorGraph());

    /** Insert all prior factors (pose, velocity and bias) to the graph **/
    this->factor_graph->add(gtsam::PriorFactor<gtsam::Pose3> (X(this->idx), prior_pose, pose_noise_model));
    this->factor_graph->add(gtsam::PriorFactor<gtsam::Vector3> (V(this->idx), prior_velocity, velocity_noise_model));
    this->factor_graph->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias> (B(this->idx), prior_imu_bias, this->bias_noise_model));

    /** Initialize the measurement noise **/
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);

    /** PreintegrationBase params: **/
    p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous

    /** should be using 2nd order integration **/
    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous

    /** PreintegrationCombinedMeasurements params: **/
    p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
    p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    p->biasAccOmegaInt = bias_acc_omega_int;

    this->imu_preintegrated.reset(new gtsam::PreintegratedCombinedMeasurements(p, prior_imu_bias));

    /** Store previous state for the imu integration and the latest predicted outcome. **/
    this->prev_state = gtsam::NavState(prior_pose, prior_velocity);
    this->prop_state = this->prev_state;
    this->prev_bias = prior_imu_bias;

    /** Set the covariance to zero **/
    this->prev_cov.first.setZero();
    this->prev_cov.second.setZero();

    return;
}

void iShark::optimize()
{
    /** Store the state in the values **/
    this->initial_values->insert(X(this->idx), this->prop_state.pose());
    this->initial_values->insert(V(this->idx), this->prop_state.v());
    this->initial_values->insert(B(this->idx), this->prev_bias);

    /** Update iSAM with the new factors **/
    this->isam->update(*this->factor_graph, *this->initial_values);

    /** Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
    If accuracy is desired at the expense of time, update(*) can be called additional times
    to perform multiple optimizer iterations every step. **/
    this->isam->update();

    /** Optimize **/
    //gtsam::LevenbergMarquardtOptimizer optimizer (*(this->factor_graph), *(this->initial_values));
    std::cout<<"[SHARK_SLAM OPTIMIZE] INITIAL_VALUES WITH: "<< this->initial_values->size()<<std::endl;

    /** Store in the values **/
    //gtsam::Values result = optimizer.optimize();
    gtsam::Values result = this->isam->calculateEstimate();

    /** Clear the factor graph and values for the next iteration **/
    this->factor_graph->resize(0);
    this->initial_values->clear();

    /** Overwrite the beginning of the preintegration for the next step. **/
    this->prev_state = gtsam::NavState(result.at<gtsam::Pose3>(X(this->idx)),
                                       result.at<gtsam::Vector3>(V(this->idx)));

    /** Get the marginals covariance, first for the pose and second for the twist **/
    this->prev_cov.first = this->isam->marginalCovariance(X(this->idx));
    this->prev_cov.second.block<3,3>(0,0) = this->isam->marginalCovariance(V(this->idx));

    /** Get the imu bias estimation **/
    this->prev_bias = result.at<gtsam::imuBias::ConstantBias>(B(this->idx));

    /** Reset the preintegration object. **/
    this->imu_preintegrated->resetIntegrationAndSetBias(prev_bias);

    /** Mark as optimized **/
    this->needs_optimization = false;

    return;
}

void iShark::gps_pose_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_pose_samples_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<<"[SHARK_SLAM GPS_POSE_SAMPLES] Received time-stamp: "<<gps_pose_samples_sample.time.toMicroseconds()<<std::endl;
    #endif

    if (!this->initialize && this->orientation_available)
    {
        /***********************
        * SLAM INITIALIZATION  *
        ***********************/
        #ifdef DEBUG_PRINTS
        std::cout<<"[SHARK_SLAM GPS_POSE_SAMPLES] - Initializing..."<<std::endl;
        #endif

        /** Initial orientation from IMU **/
        this->tf_init = this->orientation_samples.getTransform();

        /** Initial position from GPS**/
        this->tf_init.translation() = gps_pose_samples_sample.getTransform().translation();

        /** Initialization **/
        this->initialization(tf_init);

        /** Initialization succeeded **/
        this->initialize = true;

        /** Store the inverse of the initial transformation tf_gps_world **/
        this->tf_init_inverse = this->tf_init.inverse();

        /** Initialize output port **/
        this->output_pose.sourceFrame = gps_pose_samples_sample.sourceFrame;
        this->output_pose.targetFrame = gps_pose_samples_sample.targetFrame;

        #ifdef DEBUG_PRINTS
        std::cout<<"[DONE]"<<std::endl;
        #endif
    }
    else if (this->initialize && this->needs_optimization)
    {
        /** Store the gps samples **/
        this->gps_pose_samples = gps_pose_samples_sample;
        //std::cout<<"position:\n"<<this->gps_pose_samples.position<<"\n";

        /** New GPS sample: increase index **/
        this->idx++;

        /** Add ImuFactor **/
        gtsam::PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<gtsam::PreintegratedCombinedMeasurements*>(this->imu_preintegrated.get());

        gtsam::CombinedImuFactor imu_factor(X(this->idx-1), V(this->idx-1),
                                    X(this->idx), V(this->idx),
                                    B(this->idx-1), B(this->idx),
                                    *preint_imu_combined);
        this->factor_graph->add(imu_factor);

        /** Add GPS factor **/
        gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Isotropic::Sigma(3,0.1); //noise in meters
        gtsam::GPSFactor gps_factor (X(this->idx),
                                    gtsam::Point3(this->gps_pose_samples.position),
                                    gps_noise);
        this->factor_graph->add(gps_factor);

        /***********
        * Optimize *
        ************/
        this->optimize();

        /** Update output pose**/
        this->updatePose(this->gps_pose_samples.time, this->prev_state, this->prev_cov.first, this->prev_cov.second);
    }
}

void iShark::imu_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<<"[SHARK_SLAM IMU_SAMPLES] Received time-stamp: "<<imu_samples_sample.time.toMicroseconds()<<std::endl;
    #endif

    /** Check if this is the first sample **/
    if (this->imu_samples.time.toSeconds() == 0)
    {
        this->imu_samples.time = imu_samples_sample.time;
    }

    /** The delta time from the timestamps **/
    double imu_samples_period = (imu_samples_sample.time-this->imu_samples.time).toSeconds();
    #ifdef DEBUG_PRINTS
    std::cout<<"[SHARK_SLAM IMU_SAMPLES] delta_time "<< imu_samples_period <<"\n";
    #endif

    /** Store the imu samples in body frame **/
    this->imu_samples = imu_samples_sample;
    //std::cout<<"gyros: "<<this->imu_samples.gyro[0]<<","<<this->imu_samples.gyro[1]<<","<<this->imu_samples.gyro[2]<<"\n";
    //std::cout<<"acc:\n"<<this->imu_samples.acc<<"\n";

    if (this->initialize)
    {
        /** Integrate the IMU samples in the preintegration **/
        this->imu_preintegrated->integrateMeasurement(this->imu_samples.acc, this->imu_samples.gyro, imu_samples_period);

        /** It can optimize after integrating an IMU measurement **/
        this->needs_optimization = true;

        /** Propagate the state in the values **/
        this->prop_state = this->imu_preintegrated->predict(this->prev_state, this->prev_bias);
    }
}

void iShark::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<<"[SHARK_SLAM ORIENTATION_SAMPLES] Received time-stamp: "<<orientation_samples_sample.time.toMicroseconds()<<std::endl;
    #endif

    if (!this->orientation_available)
    {
        /** Initial orientation  **/
        this->orientation_samples = orientation_samples_sample;
        this->orientation_available = true;
    }
    else
    {
        /** Store the sample for the next iteration **/
        this->orientation_samples = orientation_samples_sample;

        /** Update the propagated state with the orientation **/
        this->prop_state = gtsam::NavState(gtsam::Rot3(this->orientation_samples.orientation),
                                            this->prop_state.position(),
                                            this->prop_state.velocity());

        /** Update output pose**/
        this->updatePose(this->orientation_samples.time, this->prop_state, this->prev_cov.first, this->prev_cov.second);

    }
}


const base::samples::RigidBodyState& iShark::getPose()
{
    return this->output_pose;
}

void iShark::updatePose(const base::Time &time, gtsam::NavState &state, const gtsam::Matrix66 &cov_pose, const gtsam::Matrix66 &cov_velo)
{
    this->output_pose.time = time;
    this->output_pose.position = state.pose().translation();
    Eigen::Quaterniond delta_q = this->output_pose.orientation.inverse() * state.pose().rotation().toQuaternion();
    this->output_pose.orientation = state.pose().rotation().toQuaternion();
    this->output_pose.velocity = state.velocity();
    this->output_pose.angular_velocity = ::base::getEuler(delta_q);
    this->output_pose.cov_position = cov_pose.block<3, 3>(0,0);
    this->output_pose.cov_orientation = cov_pose.block<3, 3>(3,3);
    this->output_pose.cov_velocity = cov_velo.block<3, 3>(0,0);
    this->output_pose.cov_angular_velocity = cov_velo.block<3, 3>(3,3);

}



