#ifndef POINTCLOUDPROCESSINGOBJECT_H
#define POINTCLOUDPROCESSINGOBJECT_H

#include <chrono>
#include <vector>
#include <string>
#include </usr/include/pcl-1.8/pcl/io/pcd_io.h>
#include </usr/include/pcl-1.8/pcl/point_types.h>

using namespace std;
using namespace std::chrono;
using namespace pcl;

//!
//! \class PointCloudProcessingObject
//! \brief The Point Cloud Processing Object class.
//! This class represents the objects loaded from the header files
//!
class PointCloudProcessingObject
{
public:

    //! Constructor
    explicit PointCloudProcessingObject();

    //! Destructor
    ~PointCloudProcessingObject();

    //! Copy and move constructos and assignment opperators
    PointCloudProcessingObject(PointCloudProcessingObject &);
    PointCloudProcessingObject & operator = (PointCloudProcessingObject &);
    PointCloudProcessingObject(PointCloudProcessingObject &&);
    PointCloudProcessingObject & operator = (PointCloudProcessingObject &&);

    //! Compares two objects of this type and finds the one with the smaller real timestamp
    inline bool operator <(PointCloudProcessingObject &point_cloud_processing_object)
    {
        return this->get_real_timestamp() < point_cloud_processing_object.get_real_timestamp();
    }

    //! Gets the data array
    inline vector<float> & get_data()
    {
        return m_data;
    }

    //! Sets the data array
    inline int set_data(vector<float> &data)
    {
        m_data = data;

        return 1;
    }

    //! \warning Legacy
    //! Gets the point cloud array
//    inline vector<vector<double>> & get_point_cloud()
//    {
//        return m_point_cloud;
//    }

    //! \warning Legacy
    //! Sets the point cloud array
//    inline int set_point_cloud(vector<vector<double>> &point_cloud)
//    {
//        m_point_cloud = point_cloud;

//        return 1;
//    }

    //! Gets the point cloud array
    inline PointCloud<PointXYZ> & get_point_cloud()
    {
        return m_point_cloud;
    }

    //! Sets the point cloud array
    inline int set_point_cloud(PointCloud<PointXYZ> &point_cloud)
    {
        m_point_cloud = point_cloud;

        return 1;
    }

    //! Gets the resolution array
    inline vector<unsigned int> & get_resolution()
    {
        return m_resolution;
    }

    //! Sets the resolution array
    inline int set_resolution(vector<unsigned int> &resolution)
    {
        m_resolution = resolution;

        return 1;
    }

    //! Gets the data path string
    inline string & get_data_path()
    {
        return m_data_path;
    }

    //! Sets the data path string
    inline int set_data_path(string &data_path)
    {
        m_data_path = data_path;

        return 1;
    }

    //! Gets the data type string
    inline string & get_data_type()
    {
        return m_data_type;
    }

    //! Sets the data type string
    inline int set_data_type(string &data_type)
    {
        m_data_type = data_type;

        return 1;
    }

    //! Gets the real timestamp object
    inline milliseconds::rep & get_real_timestamp()
    {
        return m_real_timestamp;
    }

    //! Sets the real timestamp object
    inline int set_real_timestamp(milliseconds::rep &real_timestamp)
    {
        m_real_timestamp = real_timestamp;

        return 1;
    }

    //! Gets the relative timestamp value
    inline unsigned int get_relative_timestamp()
    {
        return m_relative_timestamp;
    }

    //! Sets the relative timestamp value
    inline int set_relative_timestamp(unsigned int relative_timestamp)
    {
        m_relative_timestamp = relative_timestamp;

        return 1;
    }

    //! Gets the data size value
    inline unsigned int get_data_size()
    {
        return m_data_size;
    }

    //! Sets the data size value
    inline int set_data_size(unsigned int data_size)
    {
        m_data_size = data_size;

        return 1;
    }

    //! Main
    int point_cloud_processing_main();

    //! Destruct remotely
    int point_cloud_processing_kill(bool);

private:

    //! Holds the data
    vector<float> m_data;

    //! \warning Legacy
    //! Holds the point cloud
//    vector<vector<double>> m_point_cloud;

    //! Holds the point cloud
    PointCloud<PointXYZ> m_point_cloud;

    //! Holds the resolution that the kinect is set to
    vector<unsigned int> m_resolution;

    //! Holds the path to the data
    string m_data_path;

    //! Holds the type of the data
    string m_data_type;

    //! Holds the time since epoch when the header was written
    milliseconds::rep m_real_timestamp;

    //! Holds the time on the camera when the header was written
    unsigned int m_relative_timestamp;

    //! Holds the size in bits of the data
    unsigned int m_data_size;

    //! Called by destructor,
    //! other methods may call to destruct the class
    int destructor(bool);

};

#endif // POINTCLOUDPROCESSINGOBJECT_H
