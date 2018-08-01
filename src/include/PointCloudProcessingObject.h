#ifndef POINTCLOUDPROCESSINGOBJECT_H
#define POINTCLOUDPROCESSINGOBJECT_H

#include<chrono>
#include<vector>
#include<string>

using namespace std;
using namespace std::chrono;

//!
//! \class PointCloudProcessingObject
//! \brief
//! \details
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

    inline vector<double> & get_data()
    {
        return m_data;
    }

    inline int set_data(vector<double> &data)
    {
        m_data = data;

        return 1;
    }

    inline vector<unsigned int> & get_resolution()
    {
        return m_resolution;
    }

    inline int set_resolution(vector<unsigned int> &resolution)
    {
        m_resolution = resolution;

        return 1;
    }

    inline string & get_data_path()
    {
        return m_data_path;
    }

    inline int set_data_path(string &data_path)
    {
        m_data_path = data_path;

        return 1;
    }

    inline milliseconds::rep & get_real_timestamp()
    {
        return m_real_timestamp;
    }

    inline int set_real_timestamp(milliseconds::rep &real_timestamp)
    {
        m_real_timestamp = real_timestamp;

        return 1;
    }

    inline unsigned int get_relative_timestamp()
    {
        return m_relative_timestamp;
    }

    inline int set_relative_timestamp(unsigned int relative_timestamp)
    {
        m_relative_timestamp = relative_timestamp;

        return 1;
    }

    inline unsigned int get_data_size()
    {
        return m_data_size;
    }

    inline int set_data_size(unsigned int data_size)
    {
        m_data_size = data_size;

        return 1;
    }

    int point_cloud_processing_main();

    int point_cloud_processing_kill(bool);

private:

    vector<double> m_data;

    //! Holds the resolution that the kinect is set to
    vector<unsigned int> m_resolution;

    string m_data_path;

    milliseconds::rep m_real_timestamp;

    unsigned int m_relative_timestamp;

    unsigned int m_data_size;

    int destructor(bool);

};

#endif // POINTCLOUDPROCESSINGOBJECT_H
