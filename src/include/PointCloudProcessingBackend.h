#ifndef POINTCLOUDPROCESSINGBACKEND_H
#define POINTCLOUDPROCESSINGBACKEND_H

#include <memory>
#include <chrono>
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <math.h>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <flann/flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include "src/include/PointCloudProcessingObject.h"

#define _USE_MATH_DEFINES

using namespace std;
using namespace std::chrono;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::search;
using namespace pcl::visualization;

//!
//! \class PointCloudProcessingBackend
//! \brief The Point Cloud Processing Backend class.
//! This class can,
//! load headers,
//! load data,
//! output data as txt and bin,
//! calculate pcl,
//! output pcl as txt and bin
//!
class PointCloudProcessingBackend
{
public:

    //! This enum represents the strings in the header file
    enum header_enum
    {
        kpclp_header_version,
        data_type,
        data_size,
        data_dimensions,
        data_resolution,
        data_path,
        epoch_timestamp,
        kinect_timestamp,
        kpclp_header_status
    };

    //! This enum represents the types in the header file
    enum data_enum
    {
        u,
        i,
        f
    };

    //! Constructor
    explicit PointCloudProcessingBackend();

    //! Destructor
    ~PointCloudProcessingBackend();

    //! Copy and move constructos and assignment opperators
    PointCloudProcessingBackend(PointCloudProcessingBackend &);
    PointCloudProcessingBackend & operator = (PointCloudProcessingBackend &);
    PointCloudProcessingBackend(PointCloudProcessingBackend &&);
    PointCloudProcessingBackend & operator = (PointCloudProcessingBackend &&);

    //! Gets the header map object
    inline map<string, header_enum> & get_header_map()
    {
        return m_header_map;
    }

    //! Sets the header map object
    inline int set_header_map(map<string, header_enum> &header_map)
    {
        m_header_map = header_map;

        return 1;
    }

    //! Gets the data map object
    inline map<string, data_enum> & get_data_map()
    {
        return m_data_map;
    }

    //! Sets the data map object
    inline int set_data_map(map<string, data_enum> &data_map)
    {
        m_data_map = data_map;

        return 1;
    }

    //! Gets the object array
    inline vector<shared_ptr<PointCloudProcessingObject>> & get_objects()
    {
        return m_objects;
    }

    //! Sets the object array
    inline int set_objects(vector<shared_ptr<PointCloudProcessingObject>> &objects)
    {
        m_objects = objects;

        return 1;
    }

    //! Gets input path string
    inline string & get_input_path()
    {
        return m_input_path;
    }

    //! Sets output path string
    inline int set_input_path(string input_path)
    {
        m_input_path = input_path;

        return 1;
    }

    //! Gets output path string
    inline string & get_output_path()
    {
        return m_output_path;
    }

    //! Sets output path string
    inline int set_output_path(string output_path)
    {
        m_output_path = output_path;

        return 1;
    }

    //! Gets the log string
    inline string & get_log()
    {
        return m_log;
    }

    //! Sets the log string
    inline int set_log(string &log)
    {
        m_log = log;

        return 1;
    }

    inline double get_threshold()
    {
        return m_threshold;
    }

    inline int set_threshold(double threshold)
    {
        m_threshold = threshold;

        return 1;
    }

    inline double get_distance_movement()
    {
        return m_distance_movement;
    }

    inline int set_distance_movement(int distance_movement)
    {
        m_distance_movement = distance_movement;

        return 1;
    }

    inline double get_eigen_movement()
    {
        return m_eigen_movement;
    }

    inline int set_eigen_movement(double eigen_movement)
    {
        m_eigen_movement = eigen_movement;

        return 1;
    }

    inline double get_transformation_epsilon()
    {
        return m_transformation_epsilon;
    }

    inline int set_transformation_epsilon(double transformation_epsilon)
    {
        m_transformation_epsilon = transformation_epsilon;

        return 1;
    }

    inline float get_focal_length()
    {
        return m_focal_length;
    }

    inline int set_focal_length(float focal_length)
    {
        m_focal_length = focal_length;

        return 1;
    }

    inline float get_filter_x()
    {
        return m_filter_x;
    }

    inline int set_filter_x(float filter_x)
    {
        m_filter_x = filter_x;

        return 1;
    }

    inline float get_filter_y()
    {
        return m_filter_y;
    }

    inline int set_filter_y(float filter_y)
    {
        m_filter_y = filter_y;

        return 1;
    }

    inline float get_filter_z()
    {
        return m_filter_z;
    }

    inline int set_filter_z(float filter_z)
    {
        m_filter_z = filter_z;

        return 1;
    }

    inline float get_rotation_guess()
    {
        return m_rotation_guess;
    }

    inline int set_rotation_guess(float rotation_guess)
    {
        m_rotation_guess = rotation_guess;

        return 1;
    }

    inline float get_translation_guess_x()
    {
        return m_translation_guess_x;
    }

    inline int set_translation_guess_x(float translation_guess_x)
    {
        m_translation_guess_x = translation_guess_x;

        return 1;
    }

    inline float get_translation_guess_y()
    {
        return m_translation_guess_y;
    }

    inline int set_translation_guess_y(float translation_guess_y)
    {
        m_translation_guess_y = translation_guess_y;

        return 1;
    }

    inline float get_translation_guess_z()
    {
        return m_translation_guess_z;
    }

    inline int set_translation_guess_z(float translation_guess_z)
    {
        m_translation_guess_z = translation_guess_z;

        return 1;
    }

    inline int get_cloud_point_size()
    {
        return m_cloud_point_size;
    }

    inline int set_cloud_point_size(int cloud_point_size)
    {
        m_cloud_point_size = cloud_point_size;

        return 1;
    }

    inline int get_centroid_point_size()
    {
        return m_centroid_point_size;
    }

    inline int set_centroid_point_size(int centroid_point_size)
    {
        m_centroid_point_size = centroid_point_size;

        return 1;
    }

    inline int get_offset()
    {
        return m_offset;
    }

    inline int set_offset(int offset)
    {
        m_offset = offset;

        return 1;
    }

    inline int get_smoothing_size()
    {
        return m_smoothing_size;
    }

    inline int set_smoothing_size(int smoothing_size)
    {
        m_smoothing_size = smoothing_size;

        return 1;
    }

    inline int get_smoothing_deviation()
    {
        return m_smoothing_deviation;
    }

    inline int set_smoothing_deviation(int smoothing_deviation)
    {
        m_smoothing_deviation = smoothing_deviation;

        return 1;
    }

    inline int get_iterations()
    {
        return m_iterations;
    }

    inline int set_iterations(int iterations)
    {
        m_iterations = iterations;

        return 1;
    }

    inline unsigned char get_cloud_one_r()
    {
        return m_cloud_one_r;
    }

    inline int set_cloud_one_r(unsigned char cloud_one_r)
    {
        m_cloud_one_r = cloud_one_r;

        return 1;
    }

    inline unsigned char get_cloud_one_g()
    {
        return m_cloud_one_g;
    }

    inline int set_cloud_one_g(unsigned char cloud_one_g)
    {
        m_cloud_one_g = cloud_one_g;

        return 1;
    }

    inline unsigned char get_cloud_one_b()
    {
        return m_cloud_one_b;
    }

    inline int set_cloud_one_b(unsigned char cloud_one_b)
    {
        m_cloud_one_b = cloud_one_b;

        return 1;
    }

    inline unsigned char get_cloud_two_r()
    {
        return m_cloud_two_r;
    }

    inline int set_cloud_two_r(unsigned char cloud_two_r)
    {
        m_cloud_two_r = cloud_two_r;

        return 1;
    }

    inline unsigned char get_cloud_two_g()
    {
        return m_cloud_two_g;
    }

    inline int set_cloud_two_g(unsigned char cloud_two_g)
    {
        m_cloud_two_g = cloud_two_g;

        return 1;
    }

    inline unsigned char get_cloud_two_b()
    {
        return m_cloud_two_b;
    }

    inline int set_cloud_two_b(unsigned char cloud_two_b)
    {
        m_cloud_two_b = cloud_two_b;

        return 1;
    }

    inline unsigned char get_centroid_one_r()
    {
        return m_centroid_one_r;
    }

    inline int set_centroid_one_r(unsigned char centroid_one_r)
    {
        m_centroid_one_r = centroid_one_r;

        return 1;
    }

    inline unsigned char get_centroid_one_g()
    {
        return m_centroid_one_g;
    }

    inline int set_centroid_one_g(unsigned char centroid_one_g)
    {
        m_centroid_one_g = centroid_one_g;

        return 1;
    }

    inline unsigned char get_centroid_one_b()
    {
        return m_centroid_one_b;
    }

    inline int set_centroid_one_b(unsigned char centroid_one_b)
    {
        m_centroid_one_b = centroid_one_b;

        return 1;
    }

    inline unsigned char get_centroid_two_r()
    {
        return m_centroid_two_r;
    }

    inline int set_centroid_two_r(unsigned char centroid_two_r)
    {
        m_centroid_two_r = centroid_two_r;

        return 1;
    }

    inline unsigned char get_centroid_two_g()
    {
        return m_centroid_two_g;
    }

    inline int set_centroid_two_g(unsigned char centroid_two_g)
    {
        m_centroid_two_g = centroid_two_g;

        return 1;
    }

    inline unsigned char get_centroid_two_b()
    {
        return m_centroid_two_b;
    }

    inline int set_centroid_two_b(unsigned char centroid_two_b)
    {
        m_centroid_two_b = centroid_two_b;

        return 1;
    }

    inline bool get_point_cloud_text()
    {
        return m_point_cloud_text;
    }

    inline int set_point_cloud_text(bool point_cloud_text)
    {
        m_point_cloud_text = point_cloud_text;

        return 1;
    }

    inline bool get_point_cloud_binary()
    {
        return m_point_cloud_binary;
    }

    inline int set_point_cloud_binary(bool point_cloud_binary)
    {
        m_point_cloud_binary = point_cloud_binary;

        return 1;
    }

    inline bool get_visualisation()
    {
        return m_visualisation;
    }

    inline int set_visualisation(bool visualisation)
    {
        m_visualisation = visualisation;

        return 1;
    }

    inline bool get_translation_text()
    {
        return m_translation_text;
    }

    inline int set_translation_text(bool translation_text)
    {
        m_translation_text = translation_text;

        return 1;
    }

    inline bool get_tranlsation_binary()
    {
        return m_translation_binary;
    }

    inline int set_translation_binary(bool translation_binary)
    {
        m_translation_binary = translation_binary;

        return 1;
    }

    inline bool get_icp()
    {
        return m_icp;
    }

    inline int set_icp(bool icp)
    {
        m_icp = icp;

        return 1;
    }

    inline bool get_ndt()
    {
        return m_ndt;
    }

    inline int set_ndt(bool ndt)
    {
        m_ndt = ndt;

        return 1;
    }

    inline bool get_distance()
    {
        return m_distance;
    }

    inline int set_distance(bool distance)
    {
        m_distance = distance;

        return 1;
    }

    inline bool get_eigen()
    {
        return m_eigen;
    }

    inline int set_eigen(bool eigen)
    {
        m_eigen = eigen;

        return 1;
    }

    //! Main, currently unused
    int kinect_input_output_main();

    //! Disconnect or destruct remotely
    int kinect_input_output_kill(bool);

    //! Loads the headers at the paths provided,
    //! sorts the headers at the paths provided
    int load_headers(vector<string> &);

    //! Loads the data from the paths in the headers
    int load_data();

    //! Calculates point clouds from the data loaded from the paths in the headers
    int calculate_point_cloud();

    //! Writes the point clouds which were calculated from the data loaded from the paths in the headers
    int write_point_cloud_to_file();

    int load_pcd(vector<string> &);

    int registration();

private:

    //! Holds the map which reperesents the strings in the header file
    map<string, header_enum> m_header_map;

    //! Holds the map which reperesents the data types in the header file
    map<string, data_enum> m_data_map;

    //! Holds the objects loaded from the header file
    vector<shared_ptr<PointCloudProcessingObject>> m_objects;

    //! Holds path to where input should be located
    string m_input_path;

    //! Holds path to where output should be located
    string m_output_path;

    //! Holds the log string
    string m_log;

    double m_threshold;

    double m_distance_movement;

    double m_eigen_movement;

    double m_transformation_epsilon;

    float m_focal_length;

    float m_filter_x;

    float m_filter_y;

    float m_filter_z;

    float m_rotation_guess;

    float m_translation_guess_x;

    float m_translation_guess_y;

    float m_translation_guess_z;

    int m_cloud_point_size;

    int m_centroid_point_size;

    int m_offset;

    int m_smoothing_size;

    int m_smoothing_deviation;

    int m_iterations;

    unsigned char m_cloud_one_r;

    unsigned char m_cloud_one_g;

    unsigned char m_cloud_one_b;

    unsigned char m_cloud_two_r;

    unsigned char m_cloud_two_g;

    unsigned char m_cloud_two_b;

    unsigned char m_centroid_one_r;

    unsigned char m_centroid_one_g;

    unsigned char m_centroid_one_b;

    unsigned char m_centroid_two_r;

    unsigned char m_centroid_two_g;

    unsigned char m_centroid_two_b;

    bool m_point_cloud_text;

    bool m_point_cloud_binary;

    bool m_visualisation;

    bool m_translation_text;

    bool m_translation_binary;

    bool m_icp;

    bool m_ndt;

    bool m_distance;

    bool m_eigen;

    int remove_nan(PointCloud<PointXYZ>::Ptr &);

    int filter(PointCloud<PointXYZ>::Ptr &);

    PointCloud<PointXYZ>::Ptr to_point_cloud_pointxyz_ptr(Eigen::Vector4f &);

    int ricp(PointCloud<PointXYZ>::Ptr &, PointCloud<PointXYZ>::Ptr &, Eigen::Matrix<float, 4, 4> &, shared_ptr<bool> &, shared_ptr<double> &);

    int rndt(PointCloud<PointXYZ>::Ptr &, PointCloud<PointXYZ>::Ptr &, Eigen::Matrix<float, 4, 4> &, shared_ptr<bool> &, shared_ptr<double> &);

    int visualise(PointCloud<PointXYZ>::Ptr &, PointCloud<PointXYZ>::Ptr, PointCloud<PointXYZ>::Ptr &, PointCloud<PointXYZ>::Ptr, Eigen::Matrix<float, 4, 4> &);

    //! Called by destructor,
    //! other methods may call to destruct the class
    int destructor(bool);

};

#endif // POINTCLOUDPROCESSINGBACKEND_H
