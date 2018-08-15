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

    float m_filter_x;

    float m_filter_y;

    float m_filter_z;

    bool m_point_cloud_text;

    bool m_point_cloud_binary;

    bool m_visualisation;

    bool m_translation_text;

    bool m_translation_binary;

    bool m_icp;

    bool m_ndt;

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
