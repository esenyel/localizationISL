#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include "bubbleprocess.h"
#include "imageprocess.h"
#include "databasemanager.h"
#include "localization.h"
#include "Utility.h"
#include "qstring.h"
#include <fstream>
#include <iostream>
#include <time.h>
#include <sys/time.h>

//Includes for database
#include <QObject>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlError>
#include <QFile>
#include <QDir>
#include <QtSql/QSqlQuery>
#include <QVariant>
#include <QDebug>
#include <QVector>


QString current_place_file_path;

DatabaseManager current_place_dbmanager;

DatabaseManager learned_place_dbmanager;

DatabaseManager detected_place_dbmanager;

Place current_place;

LearnedPlace recognized_place;

bool rec_place_read = false;

bool current_place_read = false;

QString learned_place_db_path, detected_place_db_path;

int recognized_place_id;

int current_place_id;


void current_place_file_callback(std_msgs::String current_place_fp)
{

    std::string tempstr = current_place_fp.data;

    current_place_file_path = QString::fromStdString(tempstr);

    current_place_file_path.append("/detected_places.db");

    qDebug()<<"Current Place File Path Callback received"<< current_place_file_path;


}

void recognized_place_callback(std_msgs::Int16 input_recognized_place_id)
{
    rec_place_read = true;

    recognized_place_id = (int)input_recognized_place_id.data;

    std::cout << "Recognized place id: " << (int)input_recognized_place_id.data << std::endl;

}

void current_place_callback(std_msgs::Int16 input_current_place_id)
{
    current_place_read = true;

    current_place_id = (int)input_current_place_id.data;

    std::cout << "Current place id: " << (int)input_current_place_id.data << std::endl;
}

int main( int argc, char* argv[] )
{
    struct timespec t_initial, t_final, t_loc_estimation1, t_loc_estimation2;
    clock_gettime(CLOCK_MONOTONIC, &t_initial);

    //Initialize the ROS node for bubble space
    ros::init( argc, argv, "bubble_space_node" );
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    cv::Mat learned_place_locations;
    cv::Mat current_place_locations;
    cv::Mat location_estimation;

    //Read the parameters
    std::string learned_place_dir_path, detected_place_dir_path;

    // this part will be automatic in the future, now I am writing them manually
    nh.getParam("learned_place_dir_path",learned_place_dir_path);

    detected_place_dir_path = learned_place_dir_path;
    learned_place_dir_path.append("/knowledge.db");
    detected_place_dir_path.append("/detected_places.db");

    learned_place_db_path = QString::fromUtf8(learned_place_dir_path.c_str());
    detected_place_db_path = QString::fromUtf8(detected_place_dir_path.c_str());


    qDebug()<<"Learned place db path"<< learned_place_db_path;
    qDebug()<< "Detected place db path"<< detected_place_db_path;



    //ros::Subscriber recognized_place_subscriber = n.subscribe<std_msgs::Int16>("createBDSTISL/recognizedPlaceID",5, boost::bind(&recognized_place_callback, _1, &learned_place_db_path));
    ros::Subscriber recognized_place_subscriber = n.subscribe<std_msgs::Int16>("createBDSTISL/recognizedPlaceID",5, &recognized_place_callback);
    ros::Subscriber current_place_subscriber = n.subscribe<std_msgs::Int16>("placeDetectionISL/placeID",5, &current_place_callback);
    ros::Subscriber current_place_file_subscriber = n.subscribe<std_msgs::String>("placeDetectionISL/mainFilePath",2, &current_place_file_callback);

    //Set the bubble update period
    ros::Rate r(10);

    // if the offline localization (by images from datasets) and omnidirectional images are used:
    while(ros::ok())
    {
        ros::spinOnce();

        if (current_place_read && rec_place_read)
        {

            // construct the learned and current place location matrices, learned will be used for estimation
            // and current will be used for calculating the localization error

            if(current_place_dbmanager.openDB(current_place_file_path))
            {
                qDebug()<<"Current place db opened";
            }

            std::cout << "current_place_id: " << current_place_id << std::endl;
            std::cout << "recognized_place_id" << recognized_place_id << std::endl;

            current_place = current_place_dbmanager.getPlace(current_place_id);
            current_place_locations = current_place_dbmanager.createLocationMatrix(current_place.memberIds);

            current_place_dbmanager.closeDB();
            //current_place_dbmanager.deleteDB();

            if(learned_place_dbmanager.openDB(learned_place_db_path))
            {
                qDebug()<<"Recognized place db opened";
            }

            recognized_place = learned_place_dbmanager.getLearnedPlace(recognized_place_id);

            std::cout << "recognized place members: " << recognized_place.memberIds << std::endl;
            std::cout << "current place members: " << current_place.memberIds << std::endl;

            learned_place_dbmanager.closeDB();
            //learned_place_dbmanager.deleteDB();

            if(detected_place_dbmanager.openDB(detected_place_db_path))
            {
                qDebug()<<"Detected place db opened";
                learned_place_locations = detected_place_dbmanager.createLocationMatrix(recognized_place.memberIds);
            }

            detected_place_dbmanager.closeDB();
            //detected_place_dbmanager.deleteDB();

            location_estimation = localization::locationEstimation(recognized_place.memberInvariants, current_place.memberInvariants, learned_place_locations);
            double localization_error = localization::calculateLocalizationError(location_estimation, current_place_locations);

            current_place_read =false;
            rec_place_read = false;

        }

        r.sleep();        

    }



    ROS_INFO("ROS EXIT");
    return 0;
}
