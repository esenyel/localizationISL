#include "localization.h"
#include "imageprocess.h"
#include "bubbleprocess.h"
#include <ros/ros.h>
#include <QStringList>
#include <QTextStream>
#include "qstring.h"
#include <math.h>
#include <QDebug>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <QFile>
#include <QRgb>
#include <opencv2/opencv.hpp>

cv::Mat localization::locationEstimation(cv::Mat recognized_place_invariant, cv::Mat current_place_invariant, cv::Mat recognized_place_locations){

    int recognized_place_size = recognized_place_invariant.cols;
    int current_place_size = current_place_invariant.cols;

    std::cout << "current place size: " << current_place_size << std::endl;

    int *same_loc_index;
    same_loc_index = new int[recognized_place_size];

    double *rho;
    rho = new double[recognized_place_size];

    cv::Mat gamma(current_place_size, recognized_place_size, CV_64F);
    cv::Mat weights(current_place_size, recognized_place_size, CV_64F);
    cv::Mat weights_transpose(recognized_place_size,current_place_size, CV_64F);
    cv::Mat recognized_place_locations_transpose;
    cv::Mat location_estimation(current_place_size, 2, CV_64F);
    cv::Mat learned_locations_distance(recognized_place_size, recognized_place_size, CV_64F);

    double *total_energy;
    total_energy = new double[current_place_size];

    int a = 10*recognized_place_size;

    std::cout << "a: " << a << std::endl;

    double y_min, y_max;
    double place_weight = 0;

    // calculate the distance between the base points in the recognized place
    for (int i=0; i < recognized_place_size; i++){
        int counter = 0;
        for (int j=0; j < recognized_place_size; j++){

            float distance=norm(recognized_place_locations.row(i),recognized_place_locations.row(j), NORM_L2);

            if (distance == 0){
                counter++;
                learned_locations_distance.at<double>(i,j) = 10;
            }
            else{
                learned_locations_distance.at<double>(i,j) = distance;
            }

        }
        same_loc_index[i] = counter;

    }

    cv::minMaxLoc(learned_locations_distance, &y_min, &y_max);

    double weight_max_summation = 0;

    for (int i=0; i<current_place_size; i++){

        cv::Mat est_loc(1,2,CV_64F);
        double weight_summation = 0;

        for (int j=0; j<recognized_place_size; j++){       
            int n = same_loc_index[j];
            // calculate weights by considering the rho values and the gamma values
            if (n==1){
                double x_min, x_max;
                gamma.at<double>(i,j) = norm(current_place_invariant.col(i),recognized_place_invariant.col(j), NORM_L2);
                cv::minMaxLoc(learned_locations_distance.row(j), &x_min, &x_max);
                rho[j] = x_min/y_min;
                double dummy_par = -a/rho[j];
                double gamma_square = pow(gamma.at<double>(i,j),2);

                weights.at<double>(i,j) = exp(dummy_par*gamma_square);

            }
            // for base points with same location, divide weight by the number of base points in the same location
            else{

                gamma.at<double>(i,j) = norm(current_place_invariant.col(i),recognized_place_invariant.col(j), NORM_L2);
                weights.at<double>(i,j)=exp(-a*pow(gamma.at<double>(i,j),2))/n;

            }

            weight_summation = weight_summation + weights.at<double>(i,j);



        }
        double weight_min, weight_max;
        cv::minMaxLoc(weights.row(i), &weight_min, &weight_max);
        weight_max_summation = weight_max_summation + weight_max;

        // place_weight = place_weight + weight_summation;
        cv::transpose(recognized_place_locations,recognized_place_locations_transpose);
        cv::transpose(weights,weights_transpose);

        // estimate the location of an individual base point in current place
        est_loc = (1/weight_summation)*recognized_place_locations_transpose*weights_transpose.col(i);
        location_estimation.at<double>(i,0) = est_loc.at<double>(0,0);
        location_estimation.at<double>(i,1) = est_loc.at<double>(0,1);

    }

    //place_weight = place_weight / (current_place_size*recognized_place_size);
    double place_weight_max, place_weight_min;

    cv::minMaxLoc(weights, &place_weight_min, &place_weight_max);

    place_weight = weight_max_summation/current_place_size;

    std::cout << "place weight summation: " << place_weight << std::endl;
    std::cout << "place weight maximum: " << place_weight_max << std::endl;

    double step_size=0;

    for (int i=0; i<recognized_place_size-1; i++){

        step_size = step_size + norm(recognized_place_locations.row(i),recognized_place_locations.row(i+1), NORM_L2);

    }

    step_size = step_size/recognized_place_size;

    std::cout << "step size: " << step_size << std::endl;

    delete [] rho;
    delete [] same_loc_index;
    delete [] total_energy;

    std::cout << "estimated_locations: " << location_estimation << std::endl;

    std::cout << "recognized_place_locations: " << recognized_place_locations << std::endl;

    return location_estimation;

}

double localization::calculateLocalizationError(cv::Mat estimated_locations, cv::Mat current_place_locations){

    double result=0;

    int size = estimated_locations.rows;
    std::cout << "current_place_locations: " << current_place_locations << std::endl;

    //cv::Mat error(size, 1, CV_64F);

    double error=0;

    for (int i=0; i<size; i++){
        //error.at<double>(i,0)=cv::norm(estimated_locations.row(i), current_place_locations.row(i),NORM_L2 );
        error = error + cv::norm(estimated_locations.row(i), current_place_locations.row(i),NORM_L2 );
    }

    // std::cout << "error: " << error << std::endl;

    // result = cv::norm(error, NORM_L2);

    result = error / size;

    std::cout << "result: " << result << std::endl;

    return result;

}


cv::Mat localization::reduceRecognizedPlace(cv::Mat recognized_place_invariant, cv::Mat recognized_place_locations){
    int recognized_place_size = recognized_place_invariant.cols;

    cv::Mat recognized_place_invariant_reduced;
    cv::Mat recognized_place_locations_reduced;

    int a = 50;

    cv::Mat learned_locations_distance(recognized_place_size, recognized_place_size, CV_64F);
    std::cout << "recognized_place_size: " << recognized_place_size << std::endl;
    std::cout << "recognized_place_locations.rows: " << recognized_place_locations.rows << std::endl;

    for (int i=0; i < recognized_place_size; i++){

        for (int j=0; j < recognized_place_size; j++){

            float distance=norm(recognized_place_locations.row(i),recognized_place_locations.row(j), NORM_L2);
            std::cout << "distance: " << distance << std::endl;

            learned_locations_distance.at<float>(i,j) = distance;

        }
    }

    int counter = 0;
    /*for (int i=1; i < recognized_place_size; i++){

        for (int j=i; j < recognized_place_size; j++){

            if (learned_locations_distance.at<float>(i,j)==0 ){

                recognized_place_locations_reduced.row(counter) = recognized_place_locations.row(j);
                recognized_place_invariant_reduced.col(counter) = (recognized_place_invariant.col(counter)+recognized_place_invariant.col(j))/2;

            }
            else{
                recognized_place_locations_reduced.row(counter) = recognized_place_locations.row(j);
                recognized_place_invariant_reduced.col(counter) = recognized_place_invariant.col(j);
                counter++;

            }

        }

    }*/

    std::cout << "recognized_place_locations_reduced: " << recognized_place_locations_reduced << std::endl;
    std::cout << "recognized_place_invariant_reduced: " << recognized_place_invariant_reduced << std::endl;

    return recognized_place_locations_reduced;


}
