#ifndef LOCALIZATION_H
#define LOCALIZATION_H
//#include "globals.h"
#include "databasemanager.h"
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <QFile>
#include <QRgb>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>

#include <pcl-1.5/pcl/common/common_headers.h>
#include <pcl-1.5/pcl/filters/voxel_grid.h>
#include <pcl-1.5/pcl/common/transforms.h>
#include <pcl-1.5/pcl/impl/point_types.hpp>
//#include <pcl-1.5/pcl/point_types_conversion.h>
#include <pcl-1.5/pcl/io/pcd_io.h>

class localization
{
public:
    localization();
    static cv::Mat locationEstimation(cv::Mat recognized_place_invariant, cv::Mat current_place_invariant, cv::Mat recognized_place_locations);
    static double calculateLocalizationError(cv::Mat estimated_locations, cv::Mat current_place_locations);
    static cv::Mat reduceRecognizedPlace(cv::Mat recognized_place_invariant, cv::Mat recognized_place_locations);

};

#endif
