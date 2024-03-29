#include "iShark.hpp"
#include <iostream>

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

#define DEBUG_PRINTS 1

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
    this->needs_optimization = false; //No optimize until we receive an IMU measurement in between two GPS measurements
    this->orientation_available = false; //No orientation until we receive a orientation measurement
    this->idx = 0;

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

void iShark::configuration(double &accel_noise_sigma,  double &gyro_noise_sigma,
                            double &accel_bias_rw_sigma, double &gyro_bias_rw_sigma,
                            double &gps_noise_sigma, std::string &source_frame,
                            std::string &target_frame, bool &use_gps_heading)
{

    /***************************/
    /**    IMU Noise Init     **/
    /***************************/

    /** Covariance matrices **/
    this->measured_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
    this->measured_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
    this->integration_error_cov = gtsam::Matrix33::Identity(3,3) * 1e-6; // error committed in integrating position from velocities
    this->bias_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
    this->bias_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
    this->bias_acc_omega_int = gtsam::Matrix::Identity(6,6) * 1e-5; // error in the bias used for preintegration

    this->gps_noise_model = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(gps_noise_sigma),
                                                                gtsam::Vector3::Constant(0.1)).finished()); //noise in meters and radians

    /** Frame names of the output pose **/
    this->output_pose.sourceFrame = source_frame;
    this->output_pose.targetFrame = target_frame;

    /** Set the use of GPS as source for heading corrections **/
    this->use_gps_heading = use_gps_heading;

    return;
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
    isam_parameters.setFactorization("CHOLESKY");
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
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);

    /** PreintegrationBase params: **/
    params->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    params->integrationCovariance = integration_error_cov; // integration uncertainty continuous

    /** should be using 2nd order integration **/
    params->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous

    /** PreintegrationCombinedMeasurements params: **/
    params->biasAccCovariance = bias_acc_cov; // acc bias in continuous
    params->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    params->biasAccOmegaInt = bias_acc_omega_int;

    params->setUse2ndOrderCoriolis(false);
    params->setOmegaCoriolis(gtsam::Vector3(0, 0, 0));

    this->imu_preintegrated.reset(new gtsam::PreintegratedCombinedMeasurements(params, prior_imu_bias));

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
    /** Propagate the state in the integration of the imu values **/
    this->prop_state = this->imu_preintegrated->predict(this->prev_state, this->prev_bias);

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
    #ifdef DEBUG_PRINTS
    std::cout<<"[SHARK_SLAM OPTIMIZE] INITIAL_VALUES WITH: "<< this->initial_values->size()<<std::endl;
    #endif

    /** Store in the values **/
    //gtsam::Values result = optimizer.optimize();
    gtsam::Values result = this->isam->calculateEstimate();

    /** Clear the factor graph and values for the next iteration **/
    this->factor_graph->resize(0);
    this->initial_values->clear();

    /** Overwrite the beginning of the preintegration for the next step. **/
    this->prev_state = gtsam::NavState(result.at<gtsam::Pose3>(X(this->idx)),
                                       result.at<gtsam::Vector3>(V(this->idx)));

    /** Get the imu bias estimation **/
    this->prev_bias = result.at<gtsam::imuBias::ConstantBias>(B(this->idx));

    /** Get the marginals covariance, first for the pose and second for the twist **/
    this->prev_cov.first = this->isam->marginalCovariance(X(this->idx));
    this->prev_cov.second.block<3,3>(0,0) = this->isam->marginalCovariance(V(this->idx));

    /** Reset the preintegration object. **/
    this->imu_preintegrated->resetIntegrationAndSetBias(this->prev_bias);

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
        std::cout<<"FIRST ROLL: "<<this->orientation_samples.getRoll() * R2D <<" PITCH: "<< this->orientation_samples.getPitch() * R2D <<" YAW: "<<this->orientation_samples.getYaw() * R2D<<std::endl;
        #endif

        /** Initial orientation **/
        Eigen::Quaterniond attitude;
        if (this->use_gps_heading)
        {
            attitude= Eigen::Quaternion <double>(
                    Eigen::AngleAxisd(gps_pose_samples_sample.getYaw(), Eigen::Vector3d::UnitZ())*
                    Eigen::AngleAxisd(this->orientation_samples.getPitch(), Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(this->orientation_samples.getRoll(), Eigen::Vector3d::UnitX()));
        }
        else
        {
            attitude = this->orientation_samples.orientation;
        }

        this->tf_init = attitude; //this->orientation_samples.getTransform();

        /** Initial position from GPS**/
        this->tf_init.translation() = gps_pose_samples_sample.getTransform().translation();

        /** Initialization **/
        this->initialization(this->tf_init);

        /** Initialization succeeded **/
        this->initialize = true;

        /** Store the inverse of the initial transformation tf_gps_world **/
        this->tf_init_inverse = this->tf_init.inverse();

        /** Initialize output port **/
        this->output_pose.setPose(this->tf_init);

        /** Store the gps samples **/
        this->gps_pose_samples = gps_pose_samples_sample;

        #ifdef DEBUG_PRINTS
        std::cout<<"INITIAL POSITION: "<<this->tf_init.translation()[0]<<", "<<this->tf_init.translation()[1]<<", "<<this->tf_init.translation()[2]<<std::endl;
        Eigen::Matrix <double,3,1> euler = base::getEuler(Eigen::Quaterniond(this->tf_init.rotation()));
        std::cout<<"INITIAL ROLL: "<<euler[2] * R2D <<" PITCH: "<< euler[1] * R2D <<" YAW: "<<euler[0] * R2D<<std::endl;
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

        #ifdef DEBUG_PRINTS
        Eigen::Matrix <double,3,1> euler = base::getEuler(this->orientation_samples.orientation);
        std::cout<<"ORIENT ROLL: "<<euler[2] * R2D <<" PITCH: "<< euler[1] * R2D <<" YAW: "<<euler[0] * R2D<<std::endl;
        euler = base::getEuler(this->gps_pose_samples.orientation);
        std::cout<<"GPS ROLL: "<<euler[2] * R2D <<" PITCH: "<< euler[1] * R2D <<" YAW: "<<euler[0] * R2D<<std::endl;
        #endif

        /** Orientation for the GPS factor **/
        Eigen::Quaterniond attitude;
        if (this->use_gps_heading)
        {
            attitude = Eigen::Quaternion <double>(
                    Eigen::AngleAxisd(this->gps_pose_samples.getYaw(), Eigen::Vector3d::UnitZ())*
                    Eigen::AngleAxisd(this->orientation_samples.getPitch(), Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(this->orientation_samples.getRoll(), Eigen::Vector3d::UnitX()));
        }
        else
        {
            attitude = this->orientation_samples.orientation;
        }

        /** GPS pose for the factor **/
        gtsam::Pose3 gps_measurement = gtsam::Pose3(gtsam::Rot3(attitude), gtsam::Point3(this->gps_pose_samples.position));

        #ifdef DEBUG_PRINTS
        euler = base::getEuler(attitude);
        std::cout<<"FACTOR ROLL: "<<euler[2] * R2D <<" PITCH: "<< euler[1] * R2D <<" YAW: "<<euler[0] * R2D<<std::endl;
        #endif

        /** Add GPS factor **/
        gtsam::PriorFactor<gtsam::Pose3> gps_factor (X(this->idx), gps_measurement, this->gps_noise_model);
        this->factor_graph->add(gps_factor);

        /***********
        * Optimize *
        ************/
        this->optimize();

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

    /** Store the imu samples **/
    this->imu_samples = imu_samples_sample;

    if (this->initialize)
    {
        /** Integrate the IMU samples in the preintegration **/
        this->imu_preintegrated->integrateMeasurement(this->imu_samples.acc, this->imu_samples.gyro, imu_samples_period);

        /** It can optimize after integrating an IMU measurement **/
        this->needs_optimization = true;
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
        this->orientation_available = true;
    }

    this->orientation_samples = orientation_samples_sample;
}

const base::samples::RigidBodyState& iShark::getPose(const base::Time &ts)
{
    if (this->initialize)
    {
        /** Propagate the state in the integration of the imu values **/
        gtsam::NavState state = this->imu_preintegrated->predict(this->prev_state, this->prev_bias);

        /** Update output pose**/
        this->updatePose(ts, state, this->prev_cov.first, this->prev_cov.second);
    }

    return this->output_pose;
}

void iShark::updatePose(const base::Time &time, gtsam::NavState &state, const gtsam::Matrix66 &cov_pose, const gtsam::Matrix66 &cov_velo)
{
    this->output_pose.time = time;
    this->output_pose.position = state.pose().translation();
    Eigen::Quaterniond delta_q = this->output_pose.orientation.inverse() * state.pose().rotation().toQuaternion();
    this->output_pose.orientation = state.pose().rotation().toQuaternion();
    Eigen::Matrix <double,3,1> euler = base::getEuler(state.pose().rotation().toQuaternion());
    std::cout<<"OUTPUT ROLL: "<<euler[2] * R2D <<" PITCH: "<< euler[1] * R2D <<" YAW: "<<euler[0] * R2D<<std::endl;
    this->output_pose.velocity = state.velocity();
    this->output_pose.angular_velocity = ::base::getEuler(delta_q);
    this->output_pose.cov_position = cov_pose.block<3, 3>(0,0);
    this->output_pose.cov_orientation = cov_pose.block<3, 3>(3,3);
    this->output_pose.cov_velocity = cov_velo.block<3, 3>(0,0);
    this->output_pose.cov_angular_velocity = cov_velo.block<3, 3>(3,3);

}



