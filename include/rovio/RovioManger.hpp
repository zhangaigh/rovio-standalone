/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_ROVIONODE_HPP_
#define ROVIO_ROVIONODE_HPP_

#include <memory>
#include <mutex>
#include <queue>
#include <iomanip>
#include <iostream>

#include "rovio/RovioFilter.hpp"
#include "rovio/CoordinateTransform/RovioOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutputReadable.hpp"
#include "rovio/CoordinateTransform/YprOutput.hpp"
#include "rovio/CoordinateTransform/LandmarkOutput.hpp"

namespace rovio
{

/** \brief Class, defining the Rovio Node
 *
 *  @tparam FILTER  - \ref rovio::RovioFilter
 */
template<typename FILTER>
class RovioManger
{
public:
    // Filter Stuff
    typedef FILTER mtFilter;
    std::shared_ptr<mtFilter> mpFilter_;
    typedef typename mtFilter::mtFilterState mtFilterState;
    typedef typename mtFilterState::mtState mtState;
    typedef typename mtFilter::mtPrediction::mtMeas mtPredictionMeas;
    mtPredictionMeas predictionMeas_;//预测输入
    typedef typename std::tuple_element<0,typename mtFilter::mtUpdates>::type mtImgUpdate;
    typedef typename mtImgUpdate::mtMeas mtImgMeas;
    mtImgMeas imgUpdateMeas_;//图像输入
    mtImgUpdate* mpImgUpdate_;//图像更新类
    typedef typename std::tuple_element<1,typename mtFilter::mtUpdates>::type mtPoseUpdate;
    typedef typename mtPoseUpdate::mtMeas mtPoseMeas;
    mtPoseMeas poseUpdateMeas_;//groundtruth 
    mtPoseUpdate* mpPoseUpdate_;//位置更新类

    struct FilterInitializationState
    {
        FilterInitializationState()
            : WrWM_ ( V3D::Zero() ),
              state_ ( State::WaitForInitUsingAccel ) {}

        enum class State
        {
            // Initialize the filter using accelerometer measurement on the next
            // opportunity.
            WaitForInitUsingAccel,
            // Initialize the filter using an external pose on the next opportunity.
            WaitForInitExternalPose,
            // The filter is initialized.
            Initialized
        } state_;

        // Buffer to hold the initial pose that should be set during initialization
        // with the state WaitForInitExternalPose.
        V3D WrWM_;
        QPD qMW_;

        explicit operator bool() const
        {
            return isInitialized();
        }

        bool isInitialized() const
        {
            return ( state_ == State::Initialized );
        }
    };
    FilterInitializationState init_state_;//初始化状态

//     bool forceOdometryPublishing_;
//     bool forceTransformPublishing_;
//     bool forceExtrinsicsPublishing_;
//     bool forceImuBiasPublishing_;
//     bool forcePclPublishing_;
//     bool forceMarkersPublishing_;
//     bool forcePatchPublishing_;
//     bool gotFirstMessages_;
    std::mutex m_filter_;

//  // Nodes, Subscriber, Publishers
//  ros::NodeHandle nh_;
//  ros::NodeHandle nh_private_;
//  ros::Subscriber subImu_;
//  ros::Subscriber subImg0_;
//  ros::Subscriber subImg1_;
//  ros::Subscriber subGroundtruth_;
//  ros::Subscriber subGroundtruthOdometry_;
//  ros::ServiceServer srvResetFilter_;
//  ros::ServiceServer srvResetToPoseFilter_;
//  ros::Publisher pubOdometry_;
//  ros::Publisher pubTransform_;
//  tf::TransformBroadcaster tb_;
//  ros::Publisher pubPcl_;            /**<Publisher: Ros point cloud, visualizing the landmarks.*/
//  ros::Publisher pubPatch_;            /**<Publisher: Patch data.*/
//  ros::Publisher pubMarkers_;          /**<Publisher: Ros line marker, indicating the depth uncertainty of a landmark.*/
//  ros::Publisher pubExtrinsics_[mtState::nCam_];
//  ros::Publisher pubImuBias_;

//  // Ros Messages
//  geometry_msgs::TransformStamped transformMsg_;
//  nav_msgs::Odometry odometryMsg_;
//  geometry_msgs::PoseWithCovarianceStamped extrinsicsMsg_[mtState::nCam_];
//  sensor_msgs::PointCloud2 pclMsg_;
//  sensor_msgs::PointCloud2 patchMsg_;
//  visualization_msgs::Marker markerMsg_;
//  sensor_msgs::Imu imuBiasMsg_;
//  int msgSeq_;

    // Rovio outputs and coordinate transformations
    typedef StandardOutput mtOutput;
    mtOutput cameraOutput_;
    MXD cameraOutputCov_;
    mtOutput imuOutput_;
    MXD imuOutputCov_;
    CameraOutputCT<mtState> cameraOutputCT_;
    ImuOutputCT<mtState> imuOutputCT_;
    rovio::TransformFeatureOutputCT<mtState> transformFeatureOutputCT_;
    rovio::LandmarkOutputImuCT<mtState> landmarkOutputImuCT_;
    rovio::FeatureOutput featureOutput_;
    rovio::LandmarkOutput landmarkOutput_;
    MXD featureOutputCov_;
    MXD landmarkOutputCov_;
    rovio::FeatureOutputReadableCT featureOutputReadableCT_;
    rovio::FeatureOutputReadable featureOutputReadable_;
    MXD featureOutputReadableCov_;

    // ROS names for output tf frames.
    std::string map_frame_;
    std::string world_frame_;
    std::string camera_frame_;
    std::string imu_frame_;

    /** \brief Constructor
     */
    RovioManger (
        //ros::NodeHandle& nh, ros::NodeHandle& nh_private,
        std::shared_ptr<mtFilter> mpFilter )
        :
        //nh_(nh), nh_private_(nh_private),
        mpFilter_ ( mpFilter ), transformFeatureOutputCT_ ( &mpFilter->multiCamera_ ), landmarkOutputImuCT_ ( &mpFilter->multiCamera_ ),
        cameraOutputCov_ ( ( int ) ( mtOutput::D_ ), ( int ) ( mtOutput::D_ ) ), featureOutputCov_ ( ( int ) ( FeatureOutput::D_ ), ( int ) ( FeatureOutput::D_ ) ), landmarkOutputCov_ ( 3,3 ),
        featureOutputReadableCov_ ( ( int ) ( FeatureOutputReadable::D_ ), ( int ) ( FeatureOutputReadable::D_ ) )
    {
#ifndef NDEBUG
//      ROS_WARN("====================== Debug Mode ======================");
#endif
        mpImgUpdate_ = &std::get<0> ( mpFilter_->mUpdates_ );
        mpPoseUpdate_ = &std::get<1> ( mpFilter_->mUpdates_ );
//         forceOdometryPublishing_ = false;
//         forceTransformPublishing_ = false;
//         forceExtrinsicsPublishing_ = false;
//         forceImuBiasPublishing_ = false;
//         forcePclPublishing_ = false;
//         forceMarkersPublishing_ = false;
//         forcePatchPublishing_ = false;
//         gotFirstMessages_ = false;
    }

    /** \brief Destructor
     */
    virtual ~RovioManger() {}

    /** \brief Tests the functionality of the rovio node.
     *
     *  @todo debug with   doVECalibration = false and depthType = 0
     */
    void makeTest()
    {
        mtFilterState* mpTestFilterState = new mtFilterState();
        *mpTestFilterState = mpFilter_->init_;
        mpTestFilterState->setCamera ( &mpFilter_->multiCamera_ );
        mtState& testState = mpTestFilterState->state_;
        unsigned int s = 2;
        testState.setRandom ( s );
        predictionMeas_.setRandom ( s );
        imgUpdateMeas_.setRandom ( s );

        LWF::NormalVectorElement tempNor;
        for ( int i=0; i<mtState::nMax_; i++ )
        {
            testState.CfP ( i ).camID_ = 0;
            tempNor.setRandom ( s );
            if ( tempNor.getVec() ( 2 ) < 0 )
            {
                tempNor.boxPlus ( Eigen::Vector2d ( 3.14,0 ),tempNor );
            }
            testState.CfP ( i ).set_nor ( tempNor );
            testState.CfP ( i ).trackWarping_ = false;
            tempNor.setRandom ( s );
            if ( tempNor.getVec() ( 2 ) < 0 )
            {
                tempNor.boxPlus ( Eigen::Vector2d ( 3.14,0 ),tempNor );
            }
            testState.aux().feaCoorMeas_[i].set_nor ( tempNor,true );
            testState.aux().feaCoorMeas_[i].mpCamera_ = &mpFilter_->multiCamera_.cameras_[0];
            testState.aux().feaCoorMeas_[i].camID_ = 0;
        }
        testState.CfP ( 0 ).camID_ = mtState::nCam_-1;
        mpTestFilterState->fsm_.setAllCameraPointers();

        // Prediction
        std::cout << "Testing Prediction" << std::endl;
        mpFilter_->mPrediction_.testPredictionJacs ( testState,predictionMeas_,1e-8,1e-6,0.1 );

        // Update
        if ( !mpImgUpdate_->useDirectMethod_ )
        {
            std::cout << "Testing Update (can sometimes exhibit large absolut errors due to the float precision)" << std::endl;
            for ( int i=0; i< ( std::min ( ( int ) mtState::nMax_,2 ) ); i++ )
            {
                testState.aux().activeFeature_ = i;
                testState.aux().activeCameraCounter_ = 0;
                mpImgUpdate_->testUpdateJacs ( testState,imgUpdateMeas_,1e-4,1e-5 );
                testState.aux().activeCameraCounter_ = mtState::nCam_-1;
                mpImgUpdate_->testUpdateJacs ( testState,imgUpdateMeas_,1e-4,1e-5 );
            }
        }

        // Testing CameraOutputCF and CameraOutputCF
        std::cout << "Testing cameraOutputCF" << std::endl;
        cameraOutputCT_.testTransformJac ( testState,1e-8,1e-6 );
        std::cout << "Testing imuOutputCF" << std::endl;
        imuOutputCT_.testTransformJac ( testState,1e-8,1e-6 );
        std::cout << "Testing attitudeToYprCF" << std::endl;
        rovio::AttitudeToYprCT attitudeToYprCF;
        attitudeToYprCF.testTransformJac ( 1e-8,1e-6 );

        // Testing TransformFeatureOutputCT
        std::cout << "Testing transformFeatureOutputCT" << std::endl;
        transformFeatureOutputCT_.setFeatureID ( 0 );
        if ( mtState::nCam_>1 )
        {
            transformFeatureOutputCT_.setOutputCameraID ( 1 );
            transformFeatureOutputCT_.testTransformJac ( testState,1e-8,1e-5 );
        }
        transformFeatureOutputCT_.setOutputCameraID ( 0 );
        transformFeatureOutputCT_.testTransformJac ( testState,1e-8,1e-5 );

        // Testing LandmarkOutputImuCT
        std::cout << "Testing LandmarkOutputImuCT" << std::endl;
        landmarkOutputImuCT_.setFeatureID ( 0 );
        landmarkOutputImuCT_.testTransformJac ( testState,1e-8,1e-5 );

        // Getting featureOutput for next tests
        transformFeatureOutputCT_.transformState ( testState,featureOutput_ );
        if ( !featureOutput_.c().isInFront() )
        {
            featureOutput_.c().set_nor ( featureOutput_.c().get_nor().rotated ( QPD ( 0.0,1.0,0.0,0.0 ) ),false );
        }

        // Testing FeatureOutputReadableCT
        std::cout << "Testing FeatureOutputReadableCT" << std::endl;
        featureOutputReadableCT_.testTransformJac ( featureOutput_,1e-8,1e-5 );

        // Testing pixelOutputCT
        rovio::PixelOutputCT pixelOutputCT;
        std::cout << "Testing pixelOutputCT (can sometimes exhibit large absolut errors due to the float precision)" << std::endl;
        pixelOutputCT.testTransformJac ( featureOutput_,1e-4,1.0 ); // Reduces accuracy due to float and strong camera distortion

        // Testing ZeroVelocityUpdate_
        std::cout << "Testing zero velocity update" << std::endl;
        mpImgUpdate_->zeroVelocityUpdate_.testJacs();

        // Testing PoseUpdate
        if ( !mpPoseUpdate_->noFeedbackToRovio_ )
        {
            std::cout << "Testing pose update" << std::endl;
            mpPoseUpdate_->testUpdateJacs ( 1e-8,1e-5 );
        }

        delete mpTestFilterState;
    }

    /** \brief IMU 数据接口.
     *  \gyr gyr xyz
     *  \acc acc xyz
     */
    void imuCallback (
        Eigen::Vector3d& gyr,
        Eigen::Vector3d& acc,
        double dTime
    )
    {
        std::lock_guard<std::mutex> lock ( m_filter_ );
        static unsigned nCount = 0;//调用计数
        std::cout << std::setprecision ( 19 );
        std::cout << "IMU input = "<<nCount++<<std::endl<<" gyr ="<<std::endl<<gyr<<";\r\n acc ="<<std::endl<<acc<<"; \r\nTime = "<<dTime<<std::endl<<std::endl;
        
        predictionMeas_.template get<mtPredictionMeas::_gyr>() = gyr;
        predictionMeas_.template get<mtPredictionMeas::_acc>() = acc;
        if ( init_state_.isInitialized() )
        {
            mpFilter_->addPredictionMeas ( predictionMeas_,dTime );
            updateAndPublish();
        }
        else
        {
            switch ( init_state_.state_ )
            {
            case FilterInitializationState::State::WaitForInitExternalPose:
            {
                std::cout << "-- Filter: Initializing using external pose ..." << std::endl;
                mpFilter_->resetWithPose ( init_state_.WrWM_, init_state_.qMW_, dTime );
                break;
            }
            case FilterInitializationState::State::WaitForInitUsingAccel:
            {
                std::cout << "-- Filter: Initializing using accel. measurement ..." << std::endl;
                mpFilter_->resetWithAccelerometer ( predictionMeas_.template get<mtPredictionMeas::_acc>(),dTime );
                break;
            }
            default:
            {
                std::cout << "Unhandeld initialization type." << std::endl;
                abort();
                break;
            }
            }

            std::cout << std::setprecision ( 19 );
            std::cout << "-- Filter: Initialized at t = " << dTime << std::endl;
            init_state_.state_ = FilterInitializationState::State::Initialized;
        }

    }

    /** \brief Image callback. Adds images (as filteredupdate measurements) to the filter.
     *
     *   @param img   - Image message.
     *   @param camID - Camera ID.
     */
    void imgCallback ( const cv::Mat& cv_img,double msgTime, const int camID = 0 )
    {
        static unsigned nCount = 0;
        std::cout << std::setprecision ( 19 );                
        std::cout<<"Image input = "<<nCount++<<" camID = "<<camID<<"; msgTime = " << std::setprecision ( 19 )<<msgTime<<std::endl<<std::endl;
        
        if ( init_state_.isInitialized() && !cv_img.empty() )
        {
            if ( msgTime != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_ )
            {
                for ( int i=0; i<mtState::nCam_; i++ )
                {
                    if ( imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[i] )
                    {
                        std::cout << "    \033[31mFailed Synchronization of Camera Frames, t = " << msgTime << "\033[0m" << std::endl;
                    }
                }
                imgUpdateMeas_.template get<mtImgMeas::_aux>().reset ( msgTime );
            }
            imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_[camID].computeFromImage ( cv_img,true );
            imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[camID] = true;

            if ( imgUpdateMeas_.template get<mtImgMeas::_aux>().areAllValid() )
            {
                mpFilter_->template addUpdateMeas<0> ( imgUpdateMeas_,msgTime );
                imgUpdateMeas_.template get<mtImgMeas::_aux>().reset ( msgTime );
                updateAndPublish();
            }
        }
    }

    /** \brief Callback for external groundtruth as TransformStamped
     *
     *  @param transform - Groundtruth message.
     */
    void groundtruthCallback (
        //const geometry_msgs::TransformStamped::ConstPtr& transform
    )
    {
        /*
        std::lock_guard<std::mutex> lock(m_filter_);
        if(init_state_.isInitialized()){
        Eigen::Vector3d JrJV(transform->transform.translation.x,transform->transform.translation.y,transform->transform.translation.z);
        poseUpdateMeas_.pos() = JrJV;
        QPD qJV(transform->transform.rotation.w,transform->transform.rotation.x,transform->transform.rotation.y,transform->transform.rotation.z);
        poseUpdateMeas_.att() = qJV.inverted();
        mpFilter_->template addUpdateMeas<1>(poseUpdateMeas_,transform->header.stamp.toSec()+mpPoseUpdate_->timeOffset_);
        updateAndPublish();
        }
        */
    }

    /** \brief Callback for external groundtruth as Odometry
     *
     * @param odometry - Groundtruth message.
     */
    void groundtruthOdometryCallback (
        //const nav_msgs::Odometry::ConstPtr& odometry
    )
    {
        /*
        std::lock_guard<std::mutex> lock(m_filter_);
        if(init_state_.isInitialized()) {
        Eigen::Vector3d JrJV(odometry->pose.pose.position.x,odometry->pose.pose.position.y,odometry->pose.pose.position.z);
        poseUpdateMeas_.pos() = JrJV;

        QPD qJV(odometry->pose.pose.orientation.w,odometry->pose.pose.orientation.x,odometry->pose.pose.orientation.y,odometry->pose.pose.orientation.z);
        poseUpdateMeas_.att() = qJV.inverted();

        const Eigen::Matrix<double,6,6> measuredCov = Eigen::Map<const Eigen::Matrix<double,6,6,Eigen::RowMajor>>(odometry->pose.covariance.data());
        poseUpdateMeas_.measuredCov() = measuredCov;

        mpFilter_->template addUpdateMeas<1>(poseUpdateMeas_,odometry->header.stamp.toSec()+mpPoseUpdate_->timeOffset_);
        updateAndPublish();
        }
        */
    }

    /** \brief ROS service handler for resetting the filter.
     */
    bool resetServiceCallback (
    )
    {
        requestReset();
        return true;
    }

    /** \brief ROS service handler for resetting the filter to a given pose.
     */
    bool resetToPoseServiceCallback (
//          rovio::SrvResetToPose::Request& request,
//          rovio::SrvResetToPose::Response& /*response*/
    )
    {
        /*
        V3D WrWM(request.T_IW.position.x, request.T_IW.position.y,
               request.T_IW.position.z);
        QPD qWM(request.T_IW.orientation.w, request.T_IW.orientation.x,
              request.T_IW.orientation.y, request.T_IW.orientation.z);
        requestResetToPose(WrWM, qWM.inverted());
        */
        return true;
    }

    /** \brief Reset the filter when the next IMU measurement is received.
     *         The orientaetion is initialized using an accel. measurement.
     */
    void requestReset()
    {
        std::lock_guard<std::mutex> lock ( m_filter_ );
        if ( !init_state_.isInitialized() )
        {
            std::cout << "Reinitialization already triggered. Ignoring request...";
            return;
        }

        init_state_.state_ = FilterInitializationState::State::WaitForInitUsingAccel;
    }

    /** \brief Reset the filter when the next IMU measurement is received.
     *         The pose is initialized to the passed pose.
     *  @param WrWM - Position Vector, pointing from the World-Frame to the IMU-Frame, expressed in World-Coordinates.
     *  @param qMW  - Quaternion, expressing World-Frame in IMU-Coordinates (World Coordinates->IMU Coordinates)
     */
    void requestResetToPose ( const V3D& WrWM, const QPD& qMW )
    {
        std::lock_guard<std::mutex> lock ( m_filter_ );
        if ( !init_state_.isInitialized() )
        {
            std::cout << "Reinitialization already triggered. Ignoring request...";
            return;
        }

        init_state_.WrWM_ = WrWM;
        init_state_.qMW_ = qMW;
        init_state_.state_ = FilterInitializationState::State::WaitForInitExternalPose;
    }

    /** \brief Executes the update step of the filter and publishes the updated data.
     */
    void updateAndPublish()
    {
        if ( init_state_.isInitialized() )
        {
            // Execute the filter update.
            const double t1 = ( double ) cv::getTickCount();
            static double timing_T = 0;
            static int timing_C = 0;
            const double oldSafeTime = mpFilter_->safe_.t_;
            int c1 = std::get<0> ( mpFilter_->updateTimelineTuple_ ).measMap_.size();
            double lastImageTime;
            if ( std::get<0> ( mpFilter_->updateTimelineTuple_ ).getLastTime ( lastImageTime ) )
            {
                mpFilter_->updateSafe ( &lastImageTime );
            }
            const double t2 = ( double ) cv::getTickCount();
            int c2 = std::get<0> ( mpFilter_->updateTimelineTuple_ ).measMap_.size();
            timing_T += ( t2-t1 ) /cv::getTickFrequency() *1000;
            timing_C += c1-c2;
            bool plotTiming = true;
            if ( plotTiming )
            {
                std::cout << std::setprecision ( 12 );
                std::cout<<" == Filter Update: " << ( t2-t1 ) /cv::getTickFrequency() *1000 << " ms for processing " << c1-c2 << " images, average: " << timing_T/timing_C<<std::endl<<std::endl<<std::endl;
            }

            if(mpFilter_->safe_.t_ > oldSafeTime)
            { // Publish only if something changed
                for(int i=0;i<mtState::nCam_;i++)
                {
                    if(!mpFilter_->safe_.img_[i].empty() && mpImgUpdate_->doFrameVisualisation_)
                    {
                        cv::imshow("Tracker" + std::to_string(i), mpFilter_->safe_.img_[i]);
                        cv::waitKey(3);
                    }
                }

                if(!mpFilter_->safe_.patchDrawing_.empty() && mpImgUpdate_->visualizePatches_)
                {
                    cv::imshow("Patches", mpFilter_->safe_.patchDrawing_);
                    cv::waitKey(3);
                }
            }
        }

    }
};

}
#endif /* ROVIO_ROVIONODE_HPP_ */
