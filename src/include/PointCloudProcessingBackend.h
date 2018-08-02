#ifndef POINTCLOUDPROCESSINGBACKEND_H
#define POINTCLOUDPROCESSINGBACKEND_H

#include <memory>
#include <chrono>
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <algorithm>

#include "src/include/PointCloudProcessingObject.h"

using namespace std;
using namespace std::chrono;

//!
//! \class PointCloudProcessingBackend
//! \brief
//! \details
//!
class PointCloudProcessingBackend
{
public:

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

    inline map<string, header_enum> & get_header_map()
    {
        return m_header_map;
    }

    inline int set_header_map(map<string, header_enum> &header_map)
    {
        m_header_map = header_map;

        return 1;
    }

    inline map<string, data_enum> & get_data_map()
    {
        return m_data_map;
    }

    inline int set_data_map(map<string, data_enum> &data_map)
    {
        m_data_map = data_map;

        return 1;
    }

    inline vector<shared_ptr<PointCloudProcessingObject>> & get_objects()
    {
        return m_objects;
    }

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

    inline string & get_log()
    {
        return m_log;
    }

    inline int set_log(string &log)
    {
        m_log = log;

        return 1;
    }

    //! Main, currently unused
    int kinect_input_output_main();

    //! Disconnect or destruct remotely
    int kinect_input_output_kill(bool);

    int load_headers(vector<string> &);

    int load_data();

    int write_data_to_file();

    int calculate_point_cloud();

    int write_point_cloud_to_file();

    int average_point_cloud_buffer(vector<float> &, vector<vector<float>> &, vector<unsigned short> &);

private:

    map<string, header_enum> m_header_map;

    map<string, data_enum> m_data_map;

    vector<shared_ptr<PointCloudProcessingObject>> m_objects;

    //! Holds path to where input should be located
    string m_input_path;

    //! Holds path to where output should be located
    string m_output_path;

    string m_log;

    //! Called by destructor,
    //! other methods may call to destruct the class
    int destructor(bool);

};

#endif // POINTCLOUDPROCESSINGBACKEND_H
