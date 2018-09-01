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
//! calculate centroids
//! calculate difference between centroids
//! calculate signals
//! calculate difference between signals
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

    //! Gets the threshold value
    inline double get_threshold()
    {
        return m_threshold;
    }

    //! Sets the threshold value
    inline int set_threshold(double threshold)
    {
        m_threshold = threshold;

        return 1;
    }

    //! Gets the distance movement value
    inline double get_distance_movement()
    {
        return m_distance_movement;
    }

    //! Sets the distance movement value
    inline int set_distance_movement(double distance_movement)
    {
        m_distance_movement = distance_movement;

        return 1;
    }

    //! Gets the eigen movement value
    inline double get_eigen_movement()
    {
        return m_eigen_movement;
    }

    //! Sets the eigen movement value
    inline int set_eigen_movement(double eigen_movement)
    {
        m_eigen_movement = eigen_movement;

        return 1;
    }

    //! Gets the smoothing deviation value
    inline double get_smoothing_deviation()
    {
        return m_smoothing_deviation;
    }

    //! Sets the smoothing deviation value
    inline int set_smoothing_deviation(double smoothing_deviation)
    {
        m_smoothing_deviation = smoothing_deviation;

        return 1;
    }

    //! Gets the transformation epsilon value
    inline double get_transformation_epsilon()
    {
        return m_transformation_epsilon;
    }

    //! Sets the transformation epsilon value
    inline int set_transformation_epsilon(double transformation_epsilon)
    {
        m_transformation_epsilon = transformation_epsilon;

        return 1;
    }

    //! Gets the focal length value
    inline float get_focal_length()
    {
        return m_focal_length;
    }

    //! Sets the focal length value
    inline int set_focal_length(float focal_length)
    {
        m_focal_length = focal_length;

        return 1;
    }

    //! Gets the filter x value
    inline float get_filter_x()
    {
        return m_filter_x;
    }

    //! Sets the filter x value
    inline int set_filter_x(float filter_x)
    {
        m_filter_x = filter_x;

        return 1;
    }

    //! Gets the filter y value
    inline float get_filter_y()
    {
        return m_filter_y;
    }

    //! Sets the filter y value
    inline int set_filter_y(float filter_y)
    {
        m_filter_y = filter_y;

        return 1;
    }

    //! Gets the filter z value
    inline float get_filter_z()
    {
        return m_filter_z;
    }

    //! Sets the filter z value
    inline int set_filter_z(float filter_z)
    {
        m_filter_z = filter_z;

        return 1;
    }

    //! Gets the rotation guess value
    inline float get_rotation_guess()
    {
        return m_rotation_guess;
    }

    //! Sets the rotation guess value
    inline int set_rotation_guess(float rotation_guess)
    {
        m_rotation_guess = rotation_guess;

        return 1;
    }

    //! Gets the translation guess x value
    inline float get_translation_guess_x()
    {
        return m_translation_guess_x;
    }

    //! Sets the translation guess x value
    inline int set_translation_guess_x(float translation_guess_x)
    {
        m_translation_guess_x = translation_guess_x;

        return 1;
    }

    //! Gets the translation guess y value
    inline float get_translation_guess_y()
    {
        return m_translation_guess_y;
    }

    //! Sets the translation guess y value
    inline int set_translation_guess_y(float translation_guess_y)
    {
        m_translation_guess_y = translation_guess_y;

        return 1;
    }

    //! Gets the translation guess z value
    inline float get_translation_guess_z()
    {
        return m_translation_guess_z;
    }

    //! Sets the translation guess z value
    inline int set_translation_guess_z(float translation_guess_z)
    {
        m_translation_guess_z = translation_guess_z;

        return 1;
    }

    //! Gets the signal magnitude value
    inline float get_signal_magnitude()
    {
        return m_signal_magnitude;
    }

    //! Sets the signal magnitude value
    inline int set_signal_magnitude(float signal_magnitude)
    {
        m_signal_magnitude = signal_magnitude;

        return 1;
    }

    //! Gets the signal x value
    inline float get_signal_x()
    {
        return m_signal_x;
    }

    //! Sets the signal x value
    inline int set_signal_x(float signal_x)
    {
        m_signal_x = signal_x;

        return 1;
    }

    //! Gets the signal y value
    inline float get_signal_y()
    {
        return m_signal_y;
    }

    //! Sets the signal y value
    inline int set_signal_y(float signal_y)
    {
        m_signal_y = signal_y;

        return 1;
    }

    //! Gets the signal z value
    inline float get_signal_z()
    {
        return m_signal_z;
    }

    //! Sets the signal z value
    inline int set_signal_z(float signal_z)
    {
        m_signal_z = signal_z;

        return 1;
    }

    //! Gets the cloud point size value
    inline int get_cloud_point_size()
    {
        return m_cloud_point_size;
    }

    //! Sets the cloud point size value
    inline int set_cloud_point_size(int cloud_point_size)
    {
        m_cloud_point_size = cloud_point_size;

        return 1;
    }

    //! Gets the centroid point size value
    inline int get_centroid_point_size()
    {
        return m_centroid_point_size;
    }

    //! Sets the centroid point size value
    inline int set_centroid_point_size(int centroid_point_size)
    {
        m_centroid_point_size = centroid_point_size;

        return 1;
    }

    //! Gets the offset value
    inline int get_offset()
    {
        return m_offset;
    }

    //! Sets the offset value
    inline int set_offset(int offset)
    {
        m_offset = offset;

        return 1;
    }

    //! Gets the smoothing size value
    inline int get_smoothing_size()
    {
        return m_smoothing_size;
    }

    //! Sets the smoothing size value
    inline int set_smoothing_size(int smoothing_size)
    {
        m_smoothing_size = smoothing_size;

        return 1;
    }

    //! Gets the iterations value
    inline int get_iterations()
    {
        return m_iterations;
    }

    //! Sets the iterations value
    inline int set_iterations(int iterations)
    {
        m_iterations = iterations;

        return 1;
    }

    //! Gets the cloud one r value
    inline unsigned char get_cloud_one_r()
    {
        return m_cloud_one_r;
    }

    //! Sets the cloud one r value
    inline int set_cloud_one_r(unsigned char cloud_one_r)
    {
        m_cloud_one_r = cloud_one_r;

        return 1;
    }

    //! Gets the cloud one g value
    inline unsigned char get_cloud_one_g()
    {
        return m_cloud_one_g;
    }

    //! Sets the cloud one g value
    inline int set_cloud_one_g(unsigned char cloud_one_g)
    {
        m_cloud_one_g = cloud_one_g;

        return 1;
    }

    //! Gets the cloud one b value
    inline unsigned char get_cloud_one_b()
    {
        return m_cloud_one_b;
    }

    //! Sets the cloud one b value
    inline int set_cloud_one_b(unsigned char cloud_one_b)
    {
        m_cloud_one_b = cloud_one_b;

        return 1;
    }

    //! Gets the cloud two r value
    inline unsigned char get_cloud_two_r()
    {
        return m_cloud_two_r;
    }

    //! Sets the cloud two r value
    inline int set_cloud_two_r(unsigned char cloud_two_r)
    {
        m_cloud_two_r = cloud_two_r;

        return 1;
    }

    //! Gets the cloud two g value
    inline unsigned char get_cloud_two_g()
    {
        return m_cloud_two_g;
    }

    //! Sets the cloud two g value
    inline int set_cloud_two_g(unsigned char cloud_two_g)
    {
        m_cloud_two_g = cloud_two_g;

        return 1;
    }

    //! Gets the cloud two b value
    inline unsigned char get_cloud_two_b()
    {
        return m_cloud_two_b;
    }

    //! Sets the cloud two b value
    inline int set_cloud_two_b(unsigned char cloud_two_b)
    {
        m_cloud_two_b = cloud_two_b;

        return 1;
    }

    //! Gets the centroid one r value
    inline unsigned char get_centroid_one_r()
    {
        return m_centroid_one_r;
    }

    //! Sets the centroid one r value
    inline int set_centroid_one_r(unsigned char centroid_one_r)
    {
        m_centroid_one_r = centroid_one_r;

        return 1;
    }

    //! Gets the centroid one g value
    inline unsigned char get_centroid_one_g()
    {
        return m_centroid_one_g;
    }

    //! Sets the centroid one g value
    inline int set_centroid_one_g(unsigned char centroid_one_g)
    {
        m_centroid_one_g = centroid_one_g;

        return 1;
    }

    //! Gets the centroid one b value
    inline unsigned char get_centroid_one_b()
    {
        return m_centroid_one_b;
    }

    //! Sets the centroid one b value
    inline int set_centroid_one_b(unsigned char centroid_one_b)
    {
        m_centroid_one_b = centroid_one_b;

        return 1;
    }

    //! Gets the centroid two r value
    inline unsigned char get_centroid_two_r()
    {
        return m_centroid_two_r;
    }

    //! Sets the centroid two r value
    inline int set_centroid_two_r(unsigned char centroid_two_r)
    {
        m_centroid_two_r = centroid_two_r;

        return 1;
    }

    //! Gets the centroid two g value
    inline unsigned char get_centroid_two_g()
    {
        return m_centroid_two_g;
    }

    //! Sets the centroid two g value
    inline int set_centroid_two_g(unsigned char centroid_two_g)
    {
        m_centroid_two_g = centroid_two_g;

        return 1;
    }

    //! Gets the centroid two b value
    inline unsigned char get_centroid_two_b()
    {
        return m_centroid_two_b;
    }

    //! Sets the centroid two b value
    inline int set_centroid_two_b(unsigned char centroid_two_b)
    {
        m_centroid_two_b = centroid_two_b;

        return 1;
    }

    //! Gets the signal r value
    inline unsigned char get_signal_r()
    {
        return m_signal_r;
    }

    //! Sets the signal r value
    inline int set_signal_r(unsigned char signal_r)
    {
        m_signal_r = signal_r;

        return 1;
    }

    //! Gets the signal g value
    inline unsigned char get_signal_g()
    {
        return m_signal_g;
    }

    //! Sets the signal g value
    inline int set_signal_g(unsigned char signal_g)
    {
        m_signal_g = signal_g;

        return 1;
    }

    //! Gets the signal b value
    inline unsigned char get_signal_b()
    {
        return m_signal_b;
    }

    //! Sets the signal b value
    inline int set_signal_b(unsigned char signal_b)
    {
        m_signal_b = signal_b;

        return 1;
    }

    //! Gets the point cloud text bool
    inline bool get_point_cloud_text()
    {
        return m_point_cloud_text;
    }

    //! Sets the point cloud text bool
    inline int set_point_cloud_text(bool point_cloud_text)
    {
        m_point_cloud_text = point_cloud_text;

        return 1;
    }

    //! Gets the point cloud binary bool
    inline bool get_point_cloud_binary()
    {
        return m_point_cloud_binary;
    }

    //! Sets the point cloud binary bool
    inline int set_point_cloud_binary(bool point_cloud_binary)
    {
        m_point_cloud_binary = point_cloud_binary;

        return 1;
    }

    inline bool get_test()
    {
        return m_test;
    }

    inline int set_test(bool test)
    {
        m_test = test;

        return 1;
    }

    //! Gets the visialisation bool
    inline bool get_visualisation()
    {
        return m_visualisation;
    }

    //! Sets the visualisation bool
    inline int set_visualisation(bool visualisation)
    {
        m_visualisation = visualisation;

        return 1;
    }

    //! Gets the translation text bool
    inline bool get_translation_text()
    {
        return m_translation_text;
    }

    //! Sets the translation text bool
    inline int set_translation_text(bool translation_text)
    {
        m_translation_text = translation_text;

        return 1;
    }

    //! Gets the translation binary bool
    inline bool get_tranlsation_binary()
    {
        return m_translation_binary;
    }

    //! Sets the translation binary bool
    inline int set_translation_binary(bool translation_binary)
    {
        m_translation_binary = translation_binary;

        return 1;
    }

    //! Gets the icp bool
    inline bool get_icp()
    {
        return m_icp;
    }

    //! Sets the icp bool
    inline int set_icp(bool icp)
    {
        m_icp = icp;

        return 1;
    }

    //! Gets the ndt bool
    inline bool get_ndt()
    {
        return m_ndt;
    }

    //! Sets the ndt bool
    inline int set_ndt(bool ndt)
    {
        m_ndt = ndt;

        return 1;
    }

    //! Gets the iterative bool
    inline bool get_iterative()
    {
        return m_iterative;
    }

    //! Sets the iterative bool
    inline int set_iterative(bool iterative)
    {
        m_iterative = iterative;

        return 1;
    }

    //! Gets the continuous bool
    inline bool get_continuous()
    {
        return m_continuous;
    }

    //! Sets the continuous bool
    inline int set_continuous(bool continuous)
    {
        m_continuous = continuous;

        return 1;
    }

    //! Gets the distance bool
    inline bool get_distance()
    {
        return m_distance;
    }

    //! Sets the distance bool
    inline int set_distance(bool distance)
    {
        m_distance = distance;

        return 1;
    }

    //! Gets the eigen bool
    inline bool get_eigen()
    {
        return m_eigen;
    }

    //! Sets the eigen bool
    inline int set_eigen(bool eigen)
    {
        m_eigen = eigen;

        return 1;
    }

    //! Gets the manual bool
    inline bool get_manual()
    {
        return m_manual;
    }

    //! Sets the manual bool
    inline int set_manual(bool manual)
    {
        m_manual = manual;

        return 1;
    }

    //! Gets the auto bool
    inline bool get_auto()
    {
        return m_auto;
    }

    //! Sets the auto bool
    inline int set_auto(bool _auto)
    {
        m_auto = _auto;

        return 1;
    }

    //! Gets the naive bool
    inline bool get_naive()
    {
        return m_naive;
    }

    //! Sets the naive bool
    inline int set_naive(bool naive)
    {
        m_naive = naive;

        return 1;
    }

    //! Gets the complex bool
    inline bool get_complex()
    {
        return m_complex;
    }

    //! Sets the complex bool
    inline int set_complex(bool complex)
    {
        m_complex = complex;

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

    //! Loads PCD file types into a point cloud type
    int load_pcd(vector<string> &);

    //! Applies all registration and other motion calulation methods to the current point clouds loaded in memory,
    //! the settings used come from the ponnector settings
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

    //! Holds the global threshold position
    double m_threshold;

    //! Holds the threshold for linear movement
    double m_distance_movement;

    //! Holds the threshold for eigen movement
    double m_eigen_movement;

    //! Holds the threshold for deviation
    double m_smoothing_deviation;

    //! Holds the threshold for registration
    double m_transformation_epsilon;

    //! Holds the focal length of the camera
    float m_focal_length;

    //! Holds the x size of the filter voxel
    float m_filter_x;

    //! Holds the y size of the filter voxel
    float m_filter_y;

    //! Holds the z size of the filter voxel
    float m_filter_z;

    //! Holds the initial rotation guess
    float m_rotation_guess;

    //! Holds the x component of the translation guess
    float m_translation_guess_x;

    //! Holds the y component of the translation guess
    float m_translation_guess_y;

    //! Holds the z component of the translation guess
    float m_translation_guess_z;

    //! Holds the radius of the signal region
    float m_signal_magnitude;

    //! Holds the x coordinate of the centre of the signal region
    float m_signal_x;

    //! Holds the y coordinate of the centre of the signal region
    float m_signal_y;

    //! Holds the z coordinate of the centre of the signal region
    float m_signal_z;

    //! Holds the visualisation point cloud point size
    int m_cloud_point_size;

    //! Holds the visualisation centroid point size
    int m_centroid_point_size;

    //! Holds the offset from the camera sensor to the aperture
    int m_offset;

    //! Holds the size of the point cloud that the standard deviation should be calculated from
    int m_smoothing_size;

    //! Holds the maximum number of iterations
    int m_iterations;

    //! Holds the red component of the visualisation point cloud one colour
    unsigned char m_cloud_one_r;

    //! Holds the green component of the visualisation point cloud one colour
    unsigned char m_cloud_one_g;

    //! Holds the blue component of the visualisation point cloud one colour
    unsigned char m_cloud_one_b;

    //! Holds the red component of the visualisation point cloud two colour
    unsigned char m_cloud_two_r;

    //! Holds the green component of the visualisation point cloud two colour
    unsigned char m_cloud_two_g;

    //! Holds the blue component of the visualisation point cloud two colour
    unsigned char m_cloud_two_b;

    //! Holds the red component of the visualisation centroid one colour
    unsigned char m_centroid_one_r;

    //! Holds the green component of the visualisation centroid one colour
    unsigned char m_centroid_one_g;

    //! Holds the blue component of the visualisation centroid one colour
    unsigned char m_centroid_one_b;

    //! Holds the red component of the visualisation centroid two colour
    unsigned char m_centroid_two_r;

    //! Holds the green component of the visualisation centroid two colour
    unsigned char m_centroid_two_g;

    //! Holds the blue component of the visualisation centroid two colour
    unsigned char m_centroid_two_b;

    //! Holds the red component of the visualisation signal colour
    unsigned char m_signal_r;

    //! Holds the green component of the visualisation signal colour
    unsigned char m_signal_g;

    //! Holds the blue component of the visualisation signal colour
    unsigned char m_signal_b;

    //! True if point clouds should be output as text
    bool m_point_cloud_text;

    //! True if point clouds should be output as binary
    bool m_point_cloud_binary;

    //! True if test
    bool m_test;

    //! True if visualisation should occur
    bool m_visualisation;

    //! True if registration should be output as text
    bool m_translation_text;

    //! True if registration should be output as binary
    bool m_translation_binary;

    //! True if ICP registration should occur
    bool m_icp;

    //! True if NDT registration should occur
    bool m_ndt;

    //! True if continuous registration should occur
    bool m_iterative;

    //! True if relative registration should occur
    bool m_continuous;

    //! True if linear distance should be used
    bool m_distance;

    //! True if eigen distance should be used
    bool m_eigen;

    //! True if manual guess matrix should be used
    bool m_manual;

    //! True if automatic guess matrix should be used
    bool m_auto;

    //! True if naive tracking should be used
    bool m_naive;

    //! True if complex tracking should be used
    bool m_complex;

    //! Finds the linear distance between two vectors,
    //! these vectors are represented as two collections of three floating point numbers.
    //! The first collection of floating point numbers is the origin.
    float distance(float, float, float, float, float, float);

    //! Returns the initial guess matrix
    Eigen::Matrix<float, 4, 4> initial_transformation_init();

    //! Returns the output header
    string output_header_init();

    //! Removes not a number elements from the point cloud
    int remove_nan(PointCloud<PointXYZ>::Ptr &);

    //! Applies the noise removal and downsampling filters
    int filter(PointCloud<PointXYZ>::Ptr &);

    //! Converts a eigen vector into a point cloud with one element
    PointCloud<PointXYZ>::Ptr to_point_cloud_pointxyz_ptr(Eigen::Vector4f &);

    //! Registers two point clouds using ICP
    int ricp(PointCloud<PointXYZ>::Ptr &, PointCloud<PointXYZ>::Ptr &, Eigen::Matrix<float, 4, 4> &, Eigen::Matrix<float, 4, 4> &, shared_ptr<bool> &, shared_ptr<double> &);

    //! Registers two point clouds using NDT
    int rndt(PointCloud<PointXYZ>::Ptr &, PointCloud<PointXYZ>::Ptr &, Eigen::Matrix<float, 4, 4> &, Eigen::Matrix<float, 4, 4> &, shared_ptr<bool> &, shared_ptr<double> &);

    //! Calculates the signal region and centroid based on the settings
    int calculate_signal_from_centroid(PointCloud<PointXYZ>::Ptr &, Eigen::Vector4f &);

    //! Calculates the difference between two centroids
    string calculate_vector_difference(Eigen::Vector4f &, Eigen::Vector4f &);

    //! Visualises the output to the user
    int visualise(PointCloud<PointXYZ>::Ptr &,
                  PointCloud<PointXYZ>::Ptr,
                  PointCloud<PointXYZ>::Ptr &,
                  PointCloud<PointXYZ>::Ptr,
                  PointCloud<PointXYZ>::Ptr &,
                  PointCloud<PointXYZ>::Ptr,
                  Eigen::Matrix<float, 4, 4> &,
                  Eigen::Matrix<float, 4, 4> &);

    //! Writes the output to file
    int write_translations_to_file(string);

    //! Called by destructor,
    //! other methods may call to destruct the class
    int destructor(bool);

};

#endif // POINTCLOUDPROCESSINGBACKEND_H
