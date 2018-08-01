#include "src/include/PointCloudProcessingBackend.h"

PointCloudProcessingBackend::PointCloudProcessingBackend():
    m_header_map(),
    m_objects(0, nullptr),
    m_input_path(""),
    m_output_path(""),
    m_log("")
{
    m_header_map["kpclp_header_version"] = kpclp_header_version;
    m_header_map["data_type"] = data_type;
    m_header_map["data_size"] = data_size;
    m_header_map["data_dimensions"] = data_dimensions;
    m_header_map["data_resolution"] = data_resolution;
    m_header_map["data_path"] = data_path;
    m_header_map["epoch_timestamp"] = epoch_timestamp;
    m_header_map["kinect_timestamp"] = kinect_timestamp;
    m_header_map["kpclp_header_status"] = kpclp_header_status;
}

PointCloudProcessingBackend::~PointCloudProcessingBackend()
{
    destructor(true);
}

PointCloudProcessingBackend::PointCloudProcessingBackend(PointCloudProcessingBackend &point_cloud_processing_backend_output_ref):
    m_header_map(point_cloud_processing_backend_output_ref.get_header_map()),
    m_objects(point_cloud_processing_backend_output_ref.get_objects()),
    m_input_path(point_cloud_processing_backend_output_ref.get_input_path()),
    m_output_path(point_cloud_processing_backend_output_ref.get_output_path()),
    m_log(point_cloud_processing_backend_output_ref.get_log())
{

}

PointCloudProcessingBackend & PointCloudProcessingBackend::operator = (PointCloudProcessingBackend &point_cloud_processing_backend_output_ref)
{
    m_header_map = point_cloud_processing_backend_output_ref.get_header_map();
    m_objects = point_cloud_processing_backend_output_ref.get_objects();
    m_input_path = point_cloud_processing_backend_output_ref.get_input_path();
    m_output_path = point_cloud_processing_backend_output_ref.get_output_path();
    m_log = point_cloud_processing_backend_output_ref.get_log();

    return *this;
}

PointCloudProcessingBackend::PointCloudProcessingBackend(PointCloudProcessingBackend &&point_cloud_processing_backend_output_ref_ref):
    m_header_map(point_cloud_processing_backend_output_ref_ref.get_header_map()),
    m_objects(point_cloud_processing_backend_output_ref_ref.get_objects()),
    m_input_path(point_cloud_processing_backend_output_ref_ref.get_input_path()),
    m_output_path(point_cloud_processing_backend_output_ref_ref.get_output_path()),
    m_log(point_cloud_processing_backend_output_ref_ref.get_log())
{

}

PointCloudProcessingBackend & PointCloudProcessingBackend::operator = (PointCloudProcessingBackend &&point_cloud_processing_backend_output_ref_ref)
{
    m_header_map = point_cloud_processing_backend_output_ref_ref.get_header_map();
    m_objects = point_cloud_processing_backend_output_ref_ref.get_objects();
    m_input_path = point_cloud_processing_backend_output_ref_ref.get_input_path();
    m_output_path = point_cloud_processing_backend_output_ref_ref.get_output_path();
    m_log = point_cloud_processing_backend_output_ref_ref.get_log();

    return *this;
}

int PointCloudProcessingBackend::kinect_input_output_main()
{
    return 1;
}

int PointCloudProcessingBackend::kinect_input_output_kill(bool hard)
{
    destructor(hard);

    return 1;
}

int PointCloudProcessingBackend::load_files(vector<string> &input)
{
    if(input.size() >= 2)
    {
        m_objects = vector<shared_ptr<PointCloudProcessingObject>>(0, nullptr);

        m_objects = vector<shared_ptr<PointCloudProcessingObject>>(input.size(), shared_ptr<PointCloudProcessingObject>(new PointCloudProcessingObject()));

        for(unsigned long i = 0; i < input.size(); ++i)
        {
            ifstream header_stream(input[i], ios::in);

            string line = "";

            while(header_stream >> line)
            {
                vector<string> subline;

                istringstream line_stream(line);

                while(getline(line_stream, line, '='))
                {
                    subline.push_back(line);
                }

                vector<string> subsubline;

                istringstream subline_stream(subline[1]);

                switch(m_header_map[subline[0]])
                {
                case kpclp_header_version:

                    break;

                case data_type:

                    break;

                case data_size:

                    m_objects[i].get()->set_data_size(static_cast<unsigned int>(stoi(subline[1])));

                    break;

                case data_dimensions:

                    m_objects[i].get()->get_resolution()[2] = static_cast<unsigned int>(stoi(subline[1]));

                    break;

                case data_resolution:

                    while(getline(subline_stream, subline[1], ','))
                    {
                        subsubline.push_back(subline[1]);
                    }

                    m_objects[i].get()->get_resolution()[0] = static_cast<unsigned int>(stoi(subsubline[0]));
                    m_objects[i].get()->get_resolution()[1] = static_cast<unsigned int>(stoi(subsubline[1]));

                    break;

                case data_path:

                    m_objects[i].get()->get_data_path() = subline[1];

                    break;

                case epoch_timestamp:

                    m_objects[i].get()->get_real_timestamp() = stoi(subline[1]);

                    break;

                case kinect_timestamp:

                    m_objects[i].get()->set_relative_timestamp(static_cast<unsigned int>(stoi(subline[1])));

                    break;

                case kpclp_header_status:

                    break;
                }
            }
        }

        return 1;
    }
    else
    {
        return 0;
    }

}

int PointCloudProcessingBackend::calculate_point_cloud()
{
    //    vector<float>::iterator vector_iterator = m_kinect_object_ptr->get_point_cloud().begin();

    //    for(unsigned short i = 0; i < m_kinect_object_ptr->get_resolution()[1]; ++i)
    //    {
    //        for(unsigned short j = 0; j < m_kinect_object_ptr->get_resolution()[0]; ++j)
    //        {
    //            *vector_iterator = (j - (m_kinect_object_ptr->get_resolution()[1] / 2.0f)) * (m_kinect_object_ptr->get_depth()[j][i] - 10) * 0.0021f;
    //            ++vector_iterator;

    //            *vector_iterator = (i - (m_kinect_object_ptr->get_resolution()[0] / 2.0f)) * (m_kinect_object_ptr->get_depth()[j][i] - 10) * 0.0021f;
    //            ++vector_iterator;

    //            *vector_iterator = m_kinect_object_ptr->get_depth()[j][i];
    //            ++vector_iterator;
    //        }
    //    }

    return 1;
}

int PointCloudProcessingBackend::write_point_cloud_to_file()
{
    //    ofstream point_cloud_stream;

    //    point_cloud_stream.open("point_cloud_" + to_string(m_kinect_object_ptr->get_timestamp()) + ".bin", ios::out | ios::binary);

    //    vector<float>::iterator point_cloud_iterator = m_kinect_object_ptr->get_point_cloud().begin();

    //    for(unsigned short j = 0; j < m_kinect_object_ptr->get_resolution()[1]; ++j)
    //    {
    //        for(unsigned short k = 0; k < m_kinect_object_ptr->get_resolution()[0]; ++k)
    //        {
    //            point_cloud_stream.write(reinterpret_cast<char *>(&point_cloud_iterator), sizeof(float));
    //            ++point_cloud_iterator;

    //            point_cloud_stream.write(reinterpret_cast<char *>(&point_cloud_iterator), sizeof(float));
    //            ++point_cloud_iterator;

    //            point_cloud_stream.write(reinterpret_cast<char *>(&point_cloud_iterator), sizeof(float));
    //            ++point_cloud_iterator;
    //        }
    //    }

    //    point_cloud_stream.close();

    //    point_cloud_stream.open("point_cloud_" + to_string(m_kinect_object_ptr->get_timestamp()) + ".txt", ios::out | ios::binary);

    //    point_cloud_stream << "# .PCD v.7 - Point Cloud Data file format" << endl;
    //    point_cloud_stream << "VERSION .7" << endl;
    //    point_cloud_stream << "FIELDS x y z" << endl;
    //    point_cloud_stream << "SIZE 4 4 4" << endl;
    //    point_cloud_stream << "TYPE F F F" << endl;
    //    point_cloud_stream << "COUNT 1 1 1" << endl;
    //    point_cloud_stream << "WIDTH " << to_string(m_kinect_object_ptr->get_resolution()[0] * m_kinect_object_ptr->get_resolution()[1]) << endl;
    //    point_cloud_stream << "HEIGHT 1" << endl;
    //    point_cloud_stream << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
    //    point_cloud_stream << "POINTS " << to_string(m_kinect_object_ptr->get_resolution()[0] * m_kinect_object_ptr->get_resolution()[1]) << endl;
    //    point_cloud_stream << "DATA ascii" << endl;

    //    point_cloud_iterator = m_kinect_object_ptr->get_point_cloud().begin();

    //    for(unsigned short j = 0; j < m_kinect_object_ptr->get_resolution()[1]; ++j)
    //    {
    //        for(unsigned short k = 0; k < m_kinect_object_ptr->get_resolution()[0]; ++k)
    //        {
    //            point_cloud_stream << to_string(*point_cloud_iterator) << " ";
    //            ++point_cloud_iterator;

    //            point_cloud_stream << to_string(*point_cloud_iterator) << " ";
    //            ++point_cloud_iterator;

    //            point_cloud_stream << to_string(*point_cloud_iterator) << endl;
    //            ++point_cloud_iterator;
    //        }
    //    }

    //    point_cloud_stream.close();

    //    m_kinect_object_ptr->get_log() += "Wrote point cloud to file at " + to_string(m_kinect_object_ptr->get_timestamp()) + "\n";

    return 1;
}

int PointCloudProcessingBackend::average_point_cloud_buffer(vector<float> &point_cloud, vector<vector<float>> &point_cloud_buffer, vector<unsigned short> &resolution)
{
    vector<vector<float>>::iterator point_cloud_buffer_iterator = point_cloud_buffer.begin();

    for(unsigned long i = 0; i < point_cloud_buffer.size(); ++i)
    {
        vector<float>::iterator point_cloud_buffer_point_cloud_iterator = point_cloud_buffer_iterator->begin();

        vector<float>::iterator point_cloud_iterator = point_cloud.begin();

        for(unsigned short j = 0; j < resolution[1]; ++j)
        {
            for(unsigned short k = 0; k < resolution[0]; ++k)
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

    for(unsigned short j = 0; j < resolution[1]; ++j)
    {
        for(unsigned short k = 0; k < resolution[0]; ++k)
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

int PointCloudProcessingBackend::destructor(bool hard)
{
    if(hard)
    {

    }

    return 1;
}
