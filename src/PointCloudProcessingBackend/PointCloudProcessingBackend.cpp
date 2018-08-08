#include "src/include/PointCloudProcessingBackend.h"

PointCloudProcessingBackend::PointCloudProcessingBackend():
    m_header_map(),
    m_data_map(),
    m_objects(0, nullptr),
    m_input_path(""),
    m_output_path(""),
    m_log("")
{
    m_header_map["kpclp_header_version"] = header_enum::kpclp_header_version;
    m_header_map["data_type"] = header_enum::data_type;
    m_header_map["data_size"] = header_enum::data_size;
    m_header_map["data_dimensions"] = header_enum::data_dimensions;
    m_header_map["data_resolution"] = header_enum::data_resolution;
    m_header_map["data_path"] = header_enum::data_path;
    m_header_map["epoch_timestamp"] = header_enum::epoch_timestamp;
    m_header_map["kinect_timestamp"] = header_enum::kinect_timestamp;
    m_header_map["kpclp_header_status"] = header_enum::kpclp_header_status;

    m_data_map["u"] = data_enum::u;
    m_data_map["i"] = data_enum::i;
    m_data_map["f"] = data_enum::f;
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

int PointCloudProcessingBackend::load_headers(vector<string> &input)
{
    m_objects = vector<shared_ptr<PointCloudProcessingObject>>(0, nullptr);

    m_objects = vector<shared_ptr<PointCloudProcessingObject>>(input.size(), nullptr);

    for(unsigned long i = 0; i < input.size(); ++i)
    {
        m_objects[i] = shared_ptr<PointCloudProcessingObject>(new PointCloudProcessingObject());

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
            case header_enum::kpclp_header_version:

                break;

            case header_enum::data_type:

                m_objects[i].get()->get_data_type() = subline[1];

                break;

            case header_enum::data_size:

                m_objects[i].get()->set_data_size(static_cast<unsigned int>(stoi(subline[1])));

                break;

            case header_enum::data_dimensions:

                m_objects[i].get()->get_resolution()[2] = static_cast<unsigned int>(stoi(subline[1]));

                break;

            case header_enum::data_resolution:

                while(getline(subline_stream, subline[1], ','))
                {
                    subsubline.push_back(subline[1]);
                }

                m_objects[i].get()->get_resolution()[0] = static_cast<unsigned int>(stoi(subsubline[0]));
                m_objects[i].get()->get_resolution()[1] = static_cast<unsigned int>(stoi(subsubline[1]));

                break;

            case header_enum::data_path:

                m_objects[i].get()->get_data_path() = subline[1];

                break;

            case header_enum::epoch_timestamp:

                m_objects[i].get()->get_real_timestamp() = stol(subline[1]);

                break;

            case header_enum::kinect_timestamp:

                m_objects[i].get()->set_relative_timestamp(static_cast<unsigned int>(stoi(subline[1])));

                break;

            case header_enum::kpclp_header_status:

                break;
            }
        }

        header_stream.close();

        m_log += "-> header " + to_string(i) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
    }

    sort(m_objects.begin(), m_objects.end());

    return 1;
}

int PointCloudProcessingBackend::load_data()
{
    for(unsigned long i = 0; i < m_objects.size(); ++i)
    {
        m_objects[i].get()->get_data() = vector<float>(0, 0.0f);

        m_objects[i].get()->get_data() =
                vector<float>(m_objects[i].get()->get_resolution()[0] * m_objects[i].get()->get_resolution()[1] * m_objects[i].get()->get_resolution()[2], 0.0f);

        ifstream data_stream(m_objects[i].get()->get_data_path(), ios::in | ios::binary);

        if(data_stream.is_open())
        {
            switch(m_data_map[m_objects[i].get()->get_data_type()])
            {
            case data_enum::u:

                switch(m_objects[i].get()->get_data_size() / m_objects[i].get()->get_resolution()[2])
                {
                case 8:

                    for(unsigned long j = 0; j < m_objects[i].get()->get_data().size(); ++j)
                    {
                        unsigned char temp = 0;

                        data_stream.read(reinterpret_cast<char *>(&temp), sizeof(unsigned char));

                        if(data_stream.fail())
                        {
                            m_log += "Cannot read from " + m_objects[i].get()->get_data_path() + "!!\n";

                            return 0;
                        }

                        m_objects[i].get()->get_data()[j] = static_cast<float>(temp);
                    }

                    break;

                case 16:

                    for(unsigned long j = 0; j < m_objects[i].get()->get_data().size(); ++j)
                    {
                        unsigned short temp = 0;

                        data_stream.read(reinterpret_cast<char *>(&temp), sizeof(unsigned short));

                        if(data_stream.fail())
                        {
                            m_log += "Cannot read from " + m_objects[i].get()->get_data_path() + "!!\n";

                            return 0;
                        }

                        m_objects[i].get()->get_data()[j] = static_cast<float>(temp);
                    }

                    break;

                case 32:

                    for(unsigned long j = 0; j < m_objects[i].get()->get_data().size(); ++j)
                    {
                        unsigned int temp = 0;

                        data_stream.read(reinterpret_cast<char *>(&temp), sizeof(unsigned int));

                        if(data_stream.fail())
                        {
                            m_log += "Cannot read from " + m_objects[i].get()->get_data_path() + "!!\n";

                            return 0;
                        }

                        m_objects[i].get()->get_data()[j] = static_cast<float>(temp);
                    }

                    break;
                }

                break;

            case data_enum::i:

                switch(m_objects[i].get()->get_data_size() / m_objects[i].get()->get_resolution()[2])
                {
                case 8:

                    for(unsigned long j = 0; j < m_objects[i].get()->get_data().size(); ++j)
                    {
                        char temp = 0;

                        data_stream.read(reinterpret_cast<char *>(&temp), sizeof(char));

                        if(data_stream.fail())
                        {
                            m_log += "Cannot read from " + m_objects[i].get()->get_data_path() + "!!\n";

                            return 0;
                        }

                        m_objects[i].get()->get_data()[j] = static_cast<float>(temp);
                    }

                    break;

                case 16:

                    for(unsigned long j = 0; j < m_objects[i].get()->get_data().size(); ++j)
                    {
                        short temp = 0;

                        data_stream.read(reinterpret_cast<char *>(&temp), sizeof(short));

                        if(data_stream.fail())
                        {
                            m_log += "Cannot read from " + m_objects[i].get()->get_data_path() + "!!\n";

                            return 0;
                        }

                        m_objects[i].get()->get_data()[j] = static_cast<float>(temp);
                    }

                    break;

                case 32:

                    for(unsigned long j = 0; j < m_objects[i].get()->get_data().size(); ++j)
                    {
                        int temp = 0;

                        data_stream.read(reinterpret_cast<char *>(&temp), sizeof(int));

                        if(data_stream.fail())
                        {
                            m_log += "Cannot read from " + m_objects[i].get()->get_data_path() + "!!\n";

                            return 0;
                        }

                        m_objects[i].get()->get_data()[j] = static_cast<float>(temp);
                    }

                    break;
                }

                break;

            case data_enum::f:

                switch(m_objects[i].get()->get_data_size() / m_objects[i].get()->get_resolution()[2])
                {
                case 32:

                    for(unsigned long j = 0; j < m_objects[i].get()->get_data().size(); ++j)
                    {
                        float temp = 0.0f;

                        data_stream.read(reinterpret_cast<char *>(&temp), sizeof(float));

                        if(data_stream.fail())
                        {
                            m_log += "Cannot read from " + m_objects[i].get()->get_data_path() + "!!\n";

                            return 0;
                        }

                        m_objects[i].get()->get_data()[j] = static_cast<float>(temp);
                    }

                    break;
                }

                break;
            }

            data_stream.close();

            m_log += "-> data " + to_string(i) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
        }
        else
        {
            m_log += m_objects[i].get()->get_data_path() + " not found!!\n";

            return 0;
        }
    }

    return 1;
}

int PointCloudProcessingBackend::write_data_to_file()
{
    for(unsigned long i = 0; i < m_objects.size(); ++i)
    {
        ofstream data_bin_stream(m_output_path + "/data_bin_" + to_string(system_clock::now().time_since_epoch().count()) + ".bin", ios::out | ios::binary);

        for(unsigned long j = 0; j < m_objects[i].get()->get_data().size(); ++j)
        {
            data_bin_stream.write(reinterpret_cast<char *>(&m_objects[i].get()->get_data()[j]), sizeof(double));
        }

        data_bin_stream.flush();
        data_bin_stream.close();

        ofstream data_txt_stream(m_output_path + "/data_txt_" + to_string(system_clock::now().time_since_epoch().count()) + ".txt", ios::out);

        for(unsigned long j = 0; j < m_objects[i].get()->get_data().size(); ++j)
        {
            data_txt_stream << to_string(m_objects[i].get()->get_data()[j]) << endl;
        }

        data_txt_stream.flush();
        data_txt_stream.close();

        m_log += "<- data " + to_string(i) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
    }

    return 1;
}

//! \warning Legacy
//int PointCloudProcessingBackend::calculate_point_cloud()
//{
//    for(unsigned long i = 0; i < m_objects.size(); ++i)
//    {
//        m_objects[i].get()->get_point_cloud() = vector<vector<double>>(0, vector<double>(0, 0.0l));

//        m_objects[i].get()->get_point_cloud() =
//                vector<vector<double>>((m_objects[i].get()->get_resolution()[0] * m_objects[i].get()->get_resolution()[1]) * 3, vector<double>(3, 0.0l));

//        for(unsigned int j = 0; j < m_objects[i]->get_resolution()[1]; ++j)
//        {
//            for(unsigned int k = 0; k < m_objects[i]->get_resolution()[0]; ++k)
//            {
//                m_objects[i].get()->get_point_cloud()[(m_objects[i]->get_resolution()[0] * j) + k][0] =
//                        (k - (m_objects[i]->get_resolution()[1] / 2.0)) * (m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] - 10) * 0.0021;

//                m_objects[i].get()->get_point_cloud()[(m_objects[i]->get_resolution()[0] * j) + k][1] =
//                        (j - (m_objects[i]->get_resolution()[0] / 2.0)) * (m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] - 10) * 0.0021;

//                m_objects[i].get()->get_point_cloud()[(m_objects[i]->get_resolution()[0] * j) + k][2] = m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k];
//            }
//        }

//        m_log += "-> pcl " + to_string(i) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
//    }

//    return 1;
//}

int PointCloudProcessingBackend::calculate_point_cloud()
{
    for(unsigned long i = 0; i < m_objects.size(); ++i)
    {
        m_objects[i].get()->get_point_cloud().width = m_objects[i]->get_resolution()[0];
        m_objects[i].get()->get_point_cloud().height   = m_objects[i]->get_resolution()[1];
        m_objects[i].get()->get_point_cloud().is_dense = false;
        m_objects[i].get()->get_point_cloud().points.resize (m_objects[i]->get_resolution()[0] * m_objects[i]->get_resolution()[1]);

        for(unsigned int j = 0; j < m_objects[i]->get_resolution()[1]; ++j)
        {
            for(unsigned int k = 0; k < m_objects[i]->get_resolution()[0]; ++k)
            {
                m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].x =
                        (k - (m_objects[i]->get_resolution()[1] / 2.0f)) * (m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] - 10) * 0.0021f;

                m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].y =
                        (j - (m_objects[i]->get_resolution()[0] / 2.0f)) * (m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] - 10) * 0.0021f;

                m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].z = m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k];
            }
        }

        m_log += "-> pcl " + to_string(i) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
    }

    return 1;
}

//! \warning Legacy
//int PointCloudProcessingBackend::write_point_cloud_to_file()
//{
//    for(unsigned long i = 0; i < m_objects.size(); ++i)
//    {
//        ofstream point_cloud_bin_stream(m_output_path + "/point_cloud_bin_" + to_string(system_clock::now().time_since_epoch().count()) + ".bin", ios::out | ios::binary);

//        for(unsigned long j = 0; j < m_objects[i].get()->get_point_cloud().size(); ++j)
//        {
//            point_cloud_bin_stream.write(reinterpret_cast<char *>(&m_objects[i].get()->get_point_cloud()[j][0]), sizeof(double));
//            point_cloud_bin_stream.write(reinterpret_cast<char *>(&m_objects[i].get()->get_point_cloud()[j][1]), sizeof(double));
//            point_cloud_bin_stream.write(reinterpret_cast<char *>(&m_objects[i].get()->get_point_cloud()[j][2]), sizeof(double));
//        }

//        point_cloud_bin_stream.flush();
//        point_cloud_bin_stream.close();

//        ofstream point_cloud_txt_stream(m_output_path + "/point_cloud_txt_" + to_string(system_clock::now().time_since_epoch().count()) + ".txt", ios::out);

//        point_cloud_txt_stream << "# .PCD v.7 - Point Cloud Data file format" << endl;
//        point_cloud_txt_stream << "VERSION .7" << endl;
//        point_cloud_txt_stream << "FIELDS x y z" << endl;
//        point_cloud_txt_stream << "SIZE 8 8 8" << endl;
//        point_cloud_txt_stream << "TYPE F F F" << endl;
//        point_cloud_txt_stream << "COUNT 1 1 1" << endl;
//        point_cloud_txt_stream << "WIDTH " << to_string(m_objects[i].get()->get_point_cloud().size()) << endl;
//        point_cloud_txt_stream << "HEIGHT 1" << endl;
//        point_cloud_txt_stream << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
//        point_cloud_txt_stream << "POINTS " << to_string(m_objects[i].get()->get_point_cloud().size()) << endl;
//        point_cloud_txt_stream << "DATA ascii" << endl;

//        for(unsigned long j = 0; j < m_objects[i].get()->get_point_cloud().size(); ++j)
//        {
//            point_cloud_txt_stream << to_string(m_objects[i].get()->get_point_cloud()[j][0]) << " ";
//            point_cloud_txt_stream << to_string(m_objects[i].get()->get_point_cloud()[j][1]) << " ";
//            point_cloud_txt_stream << to_string(m_objects[i].get()->get_point_cloud()[j][2]) << endl;
//        }

//        point_cloud_txt_stream.flush();
//        point_cloud_txt_stream.close();

//        m_log += "<- pcl " + to_string(i) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
//    }

//    return 1;
//}

int PointCloudProcessingBackend::write_point_cloud_to_file()
{
    for(unsigned long i = 0; i < m_objects.size(); ++i)
    {
        savePCDFileBinary(m_output_path + "/point_cloud_txt_" + to_string(system_clock::now().time_since_epoch().count()) + ".bin", m_objects[i].get()->get_point_cloud());

        savePCDFileASCII(m_output_path + "/point_cloud_txt_" + to_string(system_clock::now().time_since_epoch().count()) + ".txt", m_objects[i].get()->get_point_cloud());

        m_log += "<- pcl " + to_string(i) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
    }

    return 1;
}

//! \warning Legacy
//int PointCloudProcessingBackend::average_point_cloud_buffer(vector<float> &point_cloud, vector<vector<float>> &point_cloud_buffer, vector<unsigned short> &resolution)
//{
//    vector<vector<float>>::iterator point_cloud_buffer_iterator = point_cloud_buffer.begin();

//    for(unsigned long i = 0; i < point_cloud_buffer.size(); ++i)
//    {
//        vector<float>::iterator point_cloud_buffer_point_cloud_iterator = point_cloud_buffer_iterator->begin();

//        vector<float>::iterator point_cloud_iterator = point_cloud.begin();

//        for(unsigned short j = 0; j < resolution[1]; ++j)
//        {
//            for(unsigned short k = 0; k < resolution[0]; ++k)
//            {
//                *point_cloud_iterator += *point_cloud_buffer_point_cloud_iterator;
//                ++point_cloud_iterator;
//                ++point_cloud_buffer_point_cloud_iterator;

//                *point_cloud_iterator += *point_cloud_buffer_point_cloud_iterator;
//                ++point_cloud_iterator;
//                ++point_cloud_buffer_point_cloud_iterator;

//                *point_cloud_iterator += *point_cloud_buffer_point_cloud_iterator;
//                ++point_cloud_iterator;
//                ++point_cloud_buffer_point_cloud_iterator;
//            }
//        }

//        ++point_cloud_buffer_iterator;
//    }

//    vector<float>::iterator point_cloud_iterator = point_cloud.begin();

//    for(unsigned short j = 0; j < resolution[1]; ++j)
//    {
//        for(unsigned short k = 0; k < resolution[0]; ++k)
//        {
//            *point_cloud_iterator = (*point_cloud_iterator / point_cloud_buffer.size()) * -1.0f;
//            ++point_cloud_iterator;

//            *point_cloud_iterator = *point_cloud_iterator / point_cloud_buffer.size();
//            ++point_cloud_iterator;

//            *point_cloud_iterator = *point_cloud_iterator / point_cloud_buffer.size();
//            ++point_cloud_iterator;
//        }
//    }

//    return 1;
//}

int PointCloudProcessingBackend::ricp()
{
    string output = "";

    IterativeClosestPoint<PointXYZ, PointXYZ> icp;

    for(unsigned long i = 0; i < m_objects.size() - 1; ++i)
    {
        PointCloud<PointXYZ>::Ptr source(new PointCloud<PointXYZ>(m_objects[i].get()->get_point_cloud()));
        PointCloud<PointXYZ>::Ptr target(new PointCloud<PointXYZ>(m_objects[i + 1].get()->get_point_cloud()));

        icp.setInputSource(source);
        icp.setInputTarget(target);

        PointCloud<PointXYZ> Final;

        icp.align(Final);

        string transform = to_string(icp.getFinalTransformation().coeff(0, 0)) + "\t" +
                to_string(icp.getFinalTransformation().coeff(0, 1)) + "\t" +
                to_string(icp.getFinalTransformation().coeff(0, 2)) + "\t" +
                to_string(icp.getFinalTransformation().coeff(0, 3)) + "\n" +
                to_string(icp.getFinalTransformation().coeff(1, 0)) + "\t" +
                to_string(icp.getFinalTransformation().coeff(1, 1)) + "\t" +
                to_string(icp.getFinalTransformation().coeff(1, 2)) + "\t" +
                to_string(icp.getFinalTransformation().coeff(1, 3)) + "\n" +
                to_string(icp.getFinalTransformation().coeff(2, 0)) + "\t" +
                to_string(icp.getFinalTransformation().coeff(2, 1)) + "\t" +
                to_string(icp.getFinalTransformation().coeff(2, 2)) + "\t" +
                to_string(icp.getFinalTransformation().coeff(2, 3)) + "\n" +
                to_string(icp.getFinalTransformation().coeff(3, 0)) + "\t" +
                to_string(icp.getFinalTransformation().coeff(3, 1)) + "\t" +
                to_string(icp.getFinalTransformation().coeff(3, 2)) + "\t" +
                to_string(icp.getFinalTransformation().coeff(3, 3));

        output += transform + "\n\n";

        m_log += "-> ricp " + to_string(i) + " " + to_string(i + 1) + "\n" +
                "Has converged: " + to_string(icp.hasConverged()) + "\n" +
                "Score: " + to_string(icp.getFitnessScore()) + "\n" +
                "Transformation:\n" + transform + "\n";
    }

    ofstream ricp_bin_stream(m_output_path + "/ricp_bin_" + to_string(system_clock::now().time_since_epoch().count()) + ".bin", ios::out | ios::binary);

    ricp_bin_stream.write(reinterpret_cast<char *>(&output), sizeof(output));

    ricp_bin_stream.flush();
    ricp_bin_stream.close();

    ofstream ricp_txt_stream(m_output_path + "/ricp_txt_" + to_string(system_clock::now().time_since_epoch().count()) + ".txt", ios::out);

    ricp_txt_stream << output << endl;

    ricp_txt_stream.flush();
    ricp_txt_stream.close();

    m_log += "<- ricp: " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

    return 1;
}

int PointCloudProcessingBackend::destructor(bool hard)
{
    if(hard)
    {

    }

    return 1;
}
