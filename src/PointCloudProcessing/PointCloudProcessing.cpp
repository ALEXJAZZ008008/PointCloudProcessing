#include "src/include/PointCloudProcessing.h"

int KinectBackend::calculate_point_cloud()
{
    vector<float>::iterator vector_iterator = m_kinect_object_ptr->get_point_cloud().begin();

    for(unsigned short i = 0; i < m_kinect_object_ptr->get_resolution().at(1); ++i)
    {
        for(unsigned short j = 0; j < m_kinect_object_ptr->get_resolution().at(0); ++j)
        {
            *vector_iterator = (j - (m_kinect_object_ptr->get_resolution().at(1) / 2.0f)) * (m_kinect_object_ptr->get_depth().at(j).at(i) - 10) * 0.0021f;
            ++vector_iterator;

            *vector_iterator = (i - (m_kinect_object_ptr->get_resolution().at(0) / 2.0f)) * (m_kinect_object_ptr->get_depth().at(j).at(i) - 10) * 0.0021f;
            ++vector_iterator;

            *vector_iterator = m_kinect_object_ptr->get_depth().at(j).at(i);
            ++vector_iterator;
        }
    }

    return 1;
}

int write_point_cloud_to_file()
{
    ofstream point_cloud_stream;

    point_cloud_stream.open("point_cloud_" + to_string(m_kinect_object_ptr->get_timestamp()) + ".bin", ios::out | ios::binary);

    vector<float>::iterator point_cloud_iterator = m_kinect_object_ptr->get_point_cloud().begin();

    for(unsigned short j = 0; j < m_kinect_object_ptr->get_resolution().at(1); ++j)
    {
        for(unsigned short k = 0; k < m_kinect_object_ptr->get_resolution().at(0); ++k)
        {
            point_cloud_stream.write(reinterpret_cast<char *>(&point_cloud_iterator), sizeof(float));
            ++point_cloud_iterator;

            point_cloud_stream.write(reinterpret_cast<char *>(&point_cloud_iterator), sizeof(float));
            ++point_cloud_iterator;

            point_cloud_stream.write(reinterpret_cast<char *>(&point_cloud_iterator), sizeof(float));
            ++point_cloud_iterator;
        }
    }

    point_cloud_stream.close();

    point_cloud_stream.open("point_cloud_" + to_string(m_kinect_object_ptr->get_timestamp()) + ".txt", ios::out | ios::binary);

    point_cloud_stream << "# .PCD v.7 - Point Cloud Data file format" << endl;
    point_cloud_stream << "VERSION .7" << endl;
    point_cloud_stream << "FIELDS x y z" << endl;
    point_cloud_stream << "SIZE 4 4 4" << endl;
    point_cloud_stream << "TYPE F F F" << endl;
    point_cloud_stream << "COUNT 1 1 1" << endl;
    point_cloud_stream << "WIDTH " << to_string(m_kinect_object_ptr->get_resolution().at(0) * m_kinect_object_ptr->get_resolution().at(1)) << endl;
    point_cloud_stream << "HEIGHT 1" << endl;
    point_cloud_stream << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
    point_cloud_stream << "POINTS " << to_string(m_kinect_object_ptr->get_resolution().at(0) * m_kinect_object_ptr->get_resolution().at(1)) << endl;
    point_cloud_stream << "DATA ascii" << endl;

    point_cloud_iterator = m_kinect_object_ptr->get_point_cloud().begin();

    for(unsigned short j = 0; j < m_kinect_object_ptr->get_resolution().at(1); ++j)
    {
        for(unsigned short k = 0; k < m_kinect_object_ptr->get_resolution().at(0); ++k)
        {
            point_cloud_stream << to_string(*point_cloud_iterator) << " ";
            ++point_cloud_iterator;

            point_cloud_stream << to_string(*point_cloud_iterator) << " ";
            ++point_cloud_iterator;

            point_cloud_stream << to_string(*point_cloud_iterator) << endl;
            ++point_cloud_iterator;
        }
    }

    point_cloud_stream.close();

    m_kinect_object_ptr->get_log() += "Wrote point cloud to file at " + to_string(m_kinect_object_ptr->get_timestamp()) + "\n";

    return 1;
}

int average_point_cloud_buffer(vector<float> & point_cloud, vector<vector<float>> &point_cloud_buffer, array<unsigned short, 2> &resolution)
{
    vector<vector<float>>::iterator point_cloud_buffer_iterator = point_cloud_buffer.begin();

    for(int i = 0; i < point_cloud_buffer.size(); ++i)
    {
        vector<float>::iterator point_cloud_buffer_point_cloud_iterator = point_cloud_buffer_iterator->begin();

        vector<float>::iterator point_cloud_iterator = point_cloud.begin();

        for(unsigned short j = 0; j < resolution.at(1); ++j)
        {
            for(unsigned short k = 0; k < resolution.at(0); ++k)
            {
                *point_cloud_iterator += *point_cloud_buffer_point_cloud_iterator;
                ++point_cloud_iterator;
                ++point_cloud_buffer_point_cloud_iterator;

                *point_cloud_iterator += *point_cloud_buffer_point_cloud_iterator;
                ++point_cloud_iterator;
                ++point_cloud_buffer_point_cloud_iterator;

                *point_cloud_iterator += *point_cloud_buffer_point_cloud_iterator;
                ++point_cloud_iterator;
                ++point_cloud_buffer_point_cloud_iterator;
            }
        }

        ++point_cloud_buffer_iterator;
    }

    vector<float>::iterator point_cloud_iterator = point_cloud.begin();

    for(unsigned short j = 0; j < resolution.at(1); ++j)
    {
        for(unsigned short k = 0; k < resolution.at(0); ++k)
        {
            *point_cloud_iterator = (*point_cloud_iterator / point_cloud_buffer.size()) * -1.0f;
            ++point_cloud_iterator;

            *point_cloud_iterator = *point_cloud_iterator / point_cloud_buffer.size();
            ++point_cloud_iterator;

            *point_cloud_iterator = *point_cloud_iterator / point_cloud_buffer.size();
            ++point_cloud_iterator;
        }
    }

    return 1;
}
