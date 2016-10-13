
#include <boost/filesystem.hpp>

#include <memory>
#include <cstdlib>
#include <iostream>
#include <string>

#include "rovio/RovioFilter.hpp"

#include "rovio/RovioManger.hpp"

#include "rovio/Time.hpp"
#include "rovio/RovioScene.hpp"


#ifdef ROVIO_NMAXFEATURE
static constexpr int nMax_ = ROVIO_NMAXFEATURE;
#else
static constexpr int nMax_ = 25; // Maximal number of considered features in the filter state.
#endif

#ifdef ROVIO_NLEVELS
static constexpr int nLevels_ = ROVIO_NLEVELS;
#else
static constexpr int nLevels_ = 4; // // Total number of pyramid levels considered.
#endif

#ifdef ROVIO_PATCHSIZE
static constexpr int patchSize_ = ROVIO_PATCHSIZE;
#else
static constexpr int patchSize_ = 6; // Edge length of the patches (in pixel). Must be a multiple of 2!
#endif

#ifdef ROVIO_NCAM
static constexpr int nCam_ = ROVIO_NCAM;
#else
static constexpr int nCam_ = 1; // Used total number of cameras.
#endif

#ifdef ROVIO_NPOSE
static constexpr int nPose_ = ROVIO_NPOSE;
#else
static constexpr int nPose_ = 0; // Additional pose states.
#endif


typedef rovio::RovioFilter<rovio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;


int main ( int argc, char** argv )
{



    std::string Config_Path = argv[1];
    std::string path = argv[2];


    // Filter
    std::shared_ptr<mtFilter> mpFilter ( new mtFilter );

    mpFilter->readFromInfo ( Config_Path +"/rovio.info" );
    // Force the camera calibration paths to the ones from ROS parameters.
    mpFilter->cameraCalibrationFile_[0] = Config_Path + "/euroc_cam0.yaml";
    //mpFilter->cameraCalibrationFile_[1] = Config_Path + "/euroc_cam1.yaml";
    mpFilter->refreshProperties();

    // Node
    rovio::RovioManger<mtFilter> rovioNode ( mpFilter );
    rovioNode.makeTest();

    // open the IMU file
    std::string line;
    std::ifstream imu_file ( path + "/imu0/data.csv" );
    if ( !imu_file.good() )
    {
        std::cout<< "no imu file found at " << path+"/imu0/data.csv";
        return -1;
    }
    int number_of_lines = 0;
    while ( std::getline ( imu_file, line ) )
    {
        ++number_of_lines;
    }
    std::cout<< "No. IMU measurements: " << number_of_lines-1;
    if ( number_of_lines - 1 <= 0 )
    {
        std::cout<< "no imu messages present in " << path+"/imu0/data.csv";
        return -1;
    }
    // set reading position to second line
    imu_file.clear();
    imu_file.seekg ( 0, std::ios::beg );
    std::getline ( imu_file, line );
    const unsigned int numCameras = nCam_;
    int num_camera_images = 0;
    std::vector < std::vector < std::string >> image_names ( numCameras );
    for ( size_t i = 0; i < numCameras; ++i )
    {
        num_camera_images = 0;
        std::string folder ( path + "/cam" + std::to_string ( i ) + "/data" );

        for ( auto it = boost::filesystem::directory_iterator ( folder );
                it != boost::filesystem::directory_iterator(); it++ )
        {
            if ( !boost::filesystem::is_directory ( it->path() ) ) //we eliminate directories
            {
                num_camera_images++;
                image_names.at ( i ).push_back ( it->path().filename().string() );
            }
            else
            {
                continue;
            }
        }

        if ( num_camera_images == 0 )
        {
            std::cout<< "no images at " << folder;
            return 1;
        }

        std::cout << "No. cam " << i << " images: " << num_camera_images;
        // the filenames are not going to be sorted. So do this here
        std::sort ( image_names.at ( i ).begin(), image_names.at ( i ).end() );
    }

    std::vector < std::vector < std::string > ::iterator
    > cam_iterators ( numCameras );
    for ( size_t i = 0; i < numCameras; ++i )
    {
        cam_iterators.at ( i ) = image_names.at ( i ).begin();
    }
    okvis::Duration deltaT ( 0.0 );
    okvis::Time start ( 0.0 );
    int counter = 0;


          //glutMainLoop();

    while ( true )
    {
        // check if at the end
        for ( size_t i = 0; i < numCameras; ++i )
        {
            if ( cam_iterators[i] == image_names[i].end() )
            {
                std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
                cv::waitKey();
                return 0;
            }
        }
        /// add images
        okvis::Time t;

        for ( size_t i = 0; i < numCameras; ++i )
        {
            cv::Mat filtered = cv::imread (
                                   path + "/cam" + std::to_string ( i ) + "/data/" + *cam_iterators.at ( i ),
                                   cv::IMREAD_GRAYSCALE );
            std::string nanoseconds = cam_iterators.at ( i )->substr (
                                          cam_iterators.at ( i )->size() - 13, 9 );
            std::string seconds = cam_iterators.at ( i )->substr (
                                      0, cam_iterators.at ( i )->size() - 13 );
            t = okvis::Time ( atoi ( seconds.c_str() ), atoi ( nanoseconds.c_str() ) );
            if ( start == okvis::Time ( 0.0 ) )
            {
                start = t;
            }

            // get all IMU measurements till then
            okvis::Time t_imu = start;
            Eigen::Vector3d gyr;
            Eigen::Vector3d acc;
                
            do
            {
                if ( !std::getline ( imu_file, line ) )
                {
                    std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
                    cv::waitKey();
                    return 0;
                }

                std::stringstream stream ( line );
                std::string s;
                std::getline ( stream, s, ',' );
                std::string nanoseconds = s.substr ( s.size() - 9, 9 );
                std::string seconds = s.substr ( 0, s.size() - 9 );

                for ( int j = 0; j < 3; ++j )
                {
                    std::getline ( stream, s, ',' );
                    gyr[j] = std::atof ( s.c_str() );
                }

                for ( int j = 0; j < 3; ++j )
                {
                    std::getline ( stream, s, ',' );
                    acc[j] = std::atof ( s.c_str() );
                }

                t_imu = okvis::Time ( std::atoi ( seconds.c_str() ), std::atoi ( nanoseconds.c_str() ) );

                // add the IMU measurement for (blocking) processing
            //    if ( t_imu - start + okvis::Duration ( 1.0 ) > deltaT )
            //     {
            //        rovioNode.imuCallback ( gyr,acc,t_imu.toSec() );
            //    }


                if (t > t_imu)
                {
                      std::cout << "IMU update ...... Time = "<<std::setprecision ( 19 )<<t_imu.toSec()<<std::endl<<std::endl;
                    rovioNode.imuCallback ( gyr,acc,t_imu.toSec() );
                }
                else if (t == t_imu)
                {
                    rovioNode.imuCallback ( gyr,acc,t_imu.toSec() );
                    rovioNode.imgCallback ( filtered,t.toSec(), i );
                }
                else
                {
                    rovioNode.imgCallback ( filtered,t.toSec(), i );
                    rovioNode.imuCallback ( gyr,acc,t_imu.toSec() );
                }

            }
            while ( t_imu < t );//( t_imu +okvis::Duration ( 0.004 ) <= t );

            //std::cout << "IMU update ...... Time = "<<std::setprecision ( 19 )<<t_imu.toSec()<<std::endl<<std::endl;

            // add the image to the frontend for (blocking) processing
            if ( t - start > deltaT )
            {
               // rovioNode.imgCallback ( filtered,t.toSec(), i );
            }

            cam_iterators[i]++;

         // mRovioScene.addKeyboardCB('r',[&rovioNode]() mutable {rovioNode.requestReset();});
          //glutMainLoop();
        }
        ++counter;

        // display progress
        if ( counter % 20 == 0 )
        {
            std::cout << "\rProgress: "
                      << int ( double ( counter ) / double ( num_camera_images ) * 100 ) << "%  "
                      << std::flush;
        }

          /*
          // Scene

          mRovioScene.setIdleFunction(idleFunc);
          mRovioScene.addKeyboardCB('r',[&rovioNode]() mutable {rovioNode.requestReset();});
         // glutMainLoop();
         */
    }

    return 0;
}
