#include "src/include/PointCloudProcessingBackend.h"

PointCloudProcessingBackend::PointCloudProcessingBackend():
    m_header_map(),
    m_data_map(),
    m_objects(0, nullptr),
    m_input_path(""),
    m_output_path(""),
    m_log(""),
    m_filter_x(0.0f),
    m_filter_y(0.0f),
    m_filter_z(0.0f),
    m_point_cloud_text(false),
    m_point_cloud_binary(false),
    m_visualisation(false),
    m_translation_text(false),
    m_translation_binary(false),
    m_icp(false),
    m_ndt(false)
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
    m_log(point_cloud_processing_backend_output_ref.get_log()),
    m_filter_x(point_cloud_processing_backend_output_ref.get_filter_x()),
    m_filter_y(point_cloud_processing_backend_output_ref.get_filter_y()),
    m_filter_z(point_cloud_processing_backend_output_ref.get_filter_z()),
    m_point_cloud_text(point_cloud_processing_backend_output_ref.get_point_cloud_text()),
    m_point_cloud_binary(point_cloud_processing_backend_output_ref.get_point_cloud_binary()),
    m_visualisation(point_cloud_processing_backend_output_ref.get_visualisation()),
    m_translation_text(point_cloud_processing_backend_output_ref.get_translation_text()),
    m_translation_binary(point_cloud_processing_backend_output_ref.get_tranlsation_binary()),
    m_icp(point_cloud_processing_backend_output_ref.get_icp()),
    m_ndt(point_cloud_processing_backend_output_ref.get_ndt())
{

}

PointCloudProcessingBackend & PointCloudProcessingBackend::operator = (PointCloudProcessingBackend &point_cloud_processing_backend_output_ref)
{
    m_header_map = point_cloud_processing_backend_output_ref.get_header_map();
    m_objects = point_cloud_processing_backend_output_ref.get_objects();
    m_input_path = point_cloud_processing_backend_output_ref.get_input_path();
    m_output_path = point_cloud_processing_backend_output_ref.get_output_path();
    m_log = point_cloud_processing_backend_output_ref.get_log();
    m_filter_x = point_cloud_processing_backend_output_ref.get_filter_x();
    m_filter_y = point_cloud_processing_backend_output_ref.get_filter_y();
    m_filter_z = point_cloud_processing_backend_output_ref.get_filter_z();
    m_point_cloud_text = point_cloud_processing_backend_output_ref.get_point_cloud_text();
    m_point_cloud_binary = point_cloud_processing_backend_output_ref.get_point_cloud_binary();
    m_visualisation = point_cloud_processing_backend_output_ref.get_visualisation();
    m_translation_text = point_cloud_processing_backend_output_ref.get_translation_text();
    m_translation_binary = point_cloud_processing_backend_output_ref.get_tranlsation_binary();
    m_icp = point_cloud_processing_backend_output_ref.get_icp();
    m_ndt = point_cloud_processing_backend_output_ref.get_ndt();

    return *this;
}

PointCloudProcessingBackend::PointCloudProcessingBackend(PointCloudProcessingBackend &&point_cloud_processing_backend_output_ref_ref):
    m_header_map(point_cloud_processing_backend_output_ref_ref.get_header_map()),
    m_objects(point_cloud_processing_backend_output_ref_ref.get_objects()),
    m_input_path(point_cloud_processing_backend_output_ref_ref.get_input_path()),
    m_output_path(point_cloud_processing_backend_output_ref_ref.get_output_path()),
    m_log(point_cloud_processing_backend_output_ref_ref.get_log()),
    m_filter_x(point_cloud_processing_backend_output_ref_ref.get_filter_x()),
    m_filter_y(point_cloud_processing_backend_output_ref_ref.get_filter_y()),
    m_filter_z(point_cloud_processing_backend_output_ref_ref.get_filter_z()),
    m_point_cloud_text(point_cloud_processing_backend_output_ref_ref.get_point_cloud_text()),
    m_point_cloud_binary(point_cloud_processing_backend_output_ref_ref.get_point_cloud_binary()),
    m_visualisation(point_cloud_processing_backend_output_ref_ref.get_visualisation()),
    m_translation_text(point_cloud_processing_backend_output_ref_ref.get_translation_text()),
    m_translation_binary(point_cloud_processing_backend_output_ref_ref.get_tranlsation_binary()),
    m_icp(point_cloud_processing_backend_output_ref_ref.get_icp()),
    m_ndt(point_cloud_processing_backend_output_ref_ref.get_ndt())
{

}

PointCloudProcessingBackend & PointCloudProcessingBackend::operator = (PointCloudProcessingBackend &&point_cloud_processing_backend_output_ref_ref)
{
    m_header_map = point_cloud_processing_backend_output_ref_ref.get_header_map();
    m_objects = point_cloud_processing_backend_output_ref_ref.get_objects();
    m_input_path = point_cloud_processing_backend_output_ref_ref.get_input_path();
    m_output_path = point_cloud_processing_backend_output_ref_ref.get_output_path();
    m_log = point_cloud_processing_backend_output_ref_ref.get_log();
    m_filter_x = point_cloud_processing_backend_output_ref_ref.get_filter_x();
    m_filter_y = point_cloud_processing_backend_output_ref_ref.get_filter_y();
    m_filter_z = point_cloud_processing_backend_output_ref_ref.get_filter_z();
    m_point_cloud_text = point_cloud_processing_backend_output_ref_ref.get_point_cloud_text();
    m_point_cloud_binary = point_cloud_processing_backend_output_ref_ref.get_point_cloud_binary();
    m_visualisation = point_cloud_processing_backend_output_ref_ref.get_visualisation();
    m_translation_text = point_cloud_processing_backend_output_ref_ref.get_translation_text();
    m_translation_binary = point_cloud_processing_backend_output_ref_ref.get_tranlsation_binary();
    m_icp = point_cloud_processing_backend_output_ref_ref.get_icp();
    m_ndt = point_cloud_processing_backend_output_ref_ref.get_ndt();

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
    for(unsigned long i = 0; i < m_objects.size(); ++i)
    {
        shared_ptr<PointCloudProcessingObject> temp = m_objects[i];

        m_objects[i] = nullptr;

        temp.reset();
    }

    m_objects = vector<shared_ptr<PointCloudProcessingObject>>(0, nullptr);

    m_objects = vector<shared_ptr<PointCloudProcessingObject>>(input.size(), nullptr);

    for(unsigned long i = 0; i < m_objects.size(); ++i)
    {
        m_objects[i] = shared_ptr<PointCloudProcessingObject>(new PointCloudProcessingObject());
    }

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

int PointCloudProcessingBackend::calculate_point_cloud()
{
    for(unsigned long i = 0; i < m_objects.size(); ++i)
    {
        m_objects[i].get()->get_point_cloud().width = m_objects[i]->get_resolution()[0];
        m_objects[i].get()->get_point_cloud().height = m_objects[i]->get_resolution()[1];
        m_objects[i].get()->get_point_cloud().is_dense = false;
        m_objects[i].get()->get_point_cloud().points.resize(m_objects[i]->get_resolution()[0] * m_objects[i]->get_resolution()[1]);

        for(unsigned int j = 0; j < m_objects[i]->get_resolution()[1]; ++j)
        {
            for(unsigned int k = 0; k < m_objects[i]->get_resolution()[0]; ++k)
            {
                if(m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] > 0.0f || m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] < 0.0f)
                {
                    float x = (k - (m_objects[i]->get_resolution()[1] / 2.0f)) * (m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] - 10) * 0.0021f;

                    float y = (j - (m_objects[i]->get_resolution()[0] / 2.0f)) * (m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] - 10) * 0.0021f;

                    float z = m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k];

                    if(abs(sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))) < 1000.0)
                    {
                        m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].x = x;

                        m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].y = y;

                        m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].z = z;
                    }
                    else
                    {
                        m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].x = numeric_limits<float>::quiet_NaN();

                        m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].y = numeric_limits<float>::quiet_NaN();

                        m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].z = numeric_limits<float>::quiet_NaN();
                    }
                }
                else
                {
                    m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].x = numeric_limits<float>::quiet_NaN();

                    m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].y = numeric_limits<float>::quiet_NaN();

                    m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].z = numeric_limits<float>::quiet_NaN();
                }
            }
        }

        m_log += "-> pcl " + to_string(i) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
    }

    return 1;
}

int PointCloudProcessingBackend::write_point_cloud_to_file()
{
    for(unsigned long i = 0; i < m_objects.size(); ++i)
    {
        if(m_point_cloud_binary)
        {
            savePCDFileBinary(m_output_path + "/point_cloud" + to_string(system_clock::now().time_since_epoch().count()) + ".bin", m_objects[i].get()->get_point_cloud());

            m_log += "<- pcl_bin " + to_string(i) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
        }

        if(m_point_cloud_text)
        {
            savePCDFileASCII(m_output_path + "/point_cloud" + to_string(system_clock::now().time_since_epoch().count()) + ".pcd", m_objects[i].get()->get_point_cloud());

            m_log += "<- pcl_txt " + to_string(i) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
        }
    }

    return 1;
}

int PointCloudProcessingBackend::load_pcd(vector<string> &input)
{
    for(unsigned long i = 0; i < m_objects.size(); ++i)
    {
        shared_ptr<PointCloudProcessingObject> temp = m_objects[i];

        m_objects[i] = nullptr;

        temp.reset();
    }

    m_objects = vector<shared_ptr<PointCloudProcessingObject>>(0, nullptr);

    m_objects = vector<shared_ptr<PointCloudProcessingObject>>(input.size(), nullptr);

    for(unsigned long i = 0; i < m_objects.size(); ++i)
    {
        m_objects[i] = shared_ptr<PointCloudProcessingObject>(new PointCloudProcessingObject());
    }

    for(unsigned long i = 0; i < input.size(); ++i)
    {
        if(loadPCDFile<PointXYZ>(input[i], m_objects[i].get()->get_point_cloud()))
        {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");

            return 0;
        }

        m_objects[i].get()->get_data_path() = input[i];

        m_log += "-> pcl " + to_string(i) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
    }

    return 1;
}

int PointCloudProcessingBackend::registration()
{
    unsigned long i = 0;
    unsigned long j = 0;

    string output_header = "";
    string output = "";

    while(j < m_objects.size() - 1)
    {
        ++j;

        PointCloud<PointXYZ>::Ptr source(new PointCloud<PointXYZ>(m_objects[i].get()->get_point_cloud()));

        m_log += "-> pcl " + to_string(i) + " " + to_string(source->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

        remove_nan(source);

        m_log += "-> pcl " + to_string(i) + " " + to_string(source->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

        filter(source);

        m_log += "-> pcl " + to_string(i) + " " + to_string(source->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

        Eigen::Vector4f source_centroid;

        compute3DCentroid(*source, source_centroid);

        PointCloud<PointXYZ>::Ptr target(new PointCloud<PointXYZ>(m_objects[j].get()->get_point_cloud()));

        m_log += "-> pcl " + to_string(j) + " " + to_string(target->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

        remove_nan(target);

        m_log += "-> pcl " + to_string(j) + " " + to_string(target->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

        filter(target);

        m_log += "-> pcl " + to_string(j) + " " + to_string(target->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

        Eigen::Vector4f target_centroid;

        compute3DCentroid(*target, target_centroid);

        double movement = abs(sqrt(pow((target_centroid.coeff(0) - source_centroid.coeff(0)), 2) +
                                   pow((target_centroid.coeff(1) - source_centroid.coeff(0)), 2) +
                                   pow((target_centroid.coeff(2) - source_centroid.coeff(0)), 2)));

        m_log += "-> movement " + to_string(i) + " " + to_string(j) + ": " + to_string(movement) + "\n";

        if(movement < 1000.0)
        {
            m_log += "-> registration " + to_string(i) + " " + to_string(j) + ": " + "Skipped" + "\n";

            continue;
        }

        string source_centroid_position = to_string(source_centroid.coeff(0)) + "," +
                to_string(source_centroid.coeff(1)) + "," +
                to_string(source_centroid.coeff(2)) + "," +
                to_string(source_centroid.coeff(3));

        string target_centroid_position = to_string(target_centroid.coeff(0)) + "," +
                to_string(target_centroid.coeff(1)) + "," +
                to_string(target_centroid.coeff(2)) + "," +
                to_string(target_centroid.coeff(3));

        Eigen::Matrix<float, 4, 4> final_transformation;

        shared_ptr<bool> has_converged(new bool);

        shared_ptr<double> fitness_score(new double);

        if(m_icp)
        {
            ricp(source, target, final_transformation, has_converged, fitness_score);
        }
        else
        {
            if(m_ndt)
            {
                rndt(source, target, final_transformation, has_converged, fitness_score);
            }
        }

        if(m_visualisation)
        {
            visualise(source, to_point_cloud_pointxyz_ptr(source_centroid), target, to_point_cloud_pointxyz_ptr(target_centroid), final_transformation);
        }

        string transform = to_string(final_transformation.coeff(0, 0)) + "," +
                to_string(final_transformation.coeff(0, 1)) + "," +
                to_string(final_transformation.coeff(0, 2)) + "," +
                to_string(final_transformation.coeff(0, 3)) + "," +
                to_string(final_transformation.coeff(1, 0)) + "," +
                to_string(final_transformation.coeff(1, 1)) + "," +
                to_string(final_transformation.coeff(1, 2)) + "," +
                to_string(final_transformation.coeff(1, 3)) + "," +
                to_string(final_transformation.coeff(2, 0)) + "," +
                to_string(final_transformation.coeff(2, 1)) + "," +
                to_string(final_transformation.coeff(2, 2)) + "," +
                to_string(final_transformation.coeff(2, 3)) + "," +
                to_string(final_transformation.coeff(3, 0)) + "," +
                to_string(final_transformation.coeff(3, 1)) + "," +
                to_string(final_transformation.coeff(3, 2)) + "," +
                to_string(final_transformation.coeff(3, 3));

        output_header += m_objects[i].get()->get_data_path() + "," + m_objects[j].get()->get_data_path() + "\n";

        output += source_centroid_position + "\n" + target_centroid_position + "\n" + transform + "\n";

        m_log += "-> registration " + to_string(i) + " " + to_string(j) + ":" + "\n" +
                "Has converged: " + to_string(*has_converged) + "\n" +
                "Score: " + to_string(*fitness_score) + "\n" +
                "Source centroid: " + source_centroid_position + "\n" +
                "Target centroid: " + target_centroid_position + "\n" +
                "Transformation: " + transform + "\n";

        i = j;
    }

    if(m_translation_binary)
    {
        ofstream ricp_bin_stream(m_output_path + "/ricp_bin_" + to_string(system_clock::now().time_since_epoch().count()) + ".bin", ios::out | ios::binary);

        ricp_bin_stream.write(reinterpret_cast<char *>(&output_header), sizeof(output_header));
        ricp_bin_stream.write(reinterpret_cast<char *>(&output), sizeof(output));

        ricp_bin_stream.flush();
        ricp_bin_stream.close();

        m_log += "<- ricp_bin: " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
    }

    if(m_translation_text)
    {
        ofstream ricp_txt_stream(m_output_path + "/ricp_txt_" + to_string(system_clock::now().time_since_epoch().count()) + ".txt", ios::out);

        ricp_txt_stream << output_header << output << endl;

        ricp_txt_stream.flush();
        ricp_txt_stream.close();

        m_log += "<- ricp_txt: " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
    }

    return 1;
}

int PointCloudProcessingBackend::remove_nan(PointCloud<PointXYZ>::Ptr &point_cloud)
{
    vector<int> indices;

    removeNaNFromPointCloud(*point_cloud, *point_cloud, indices);

    return 1;
}

int PointCloudProcessingBackend::filter(PointCloud<PointXYZ>::Ptr &point_cloud)
{
    StatisticalOutlierRemoval<PointXYZ> statistical_outlier_removal;

    statistical_outlier_removal.setInputCloud(point_cloud);
    statistical_outlier_removal.setMeanK(50);
    statistical_outlier_removal.setStddevMulThresh(1.0);
    statistical_outlier_removal.filter(*point_cloud);

    ApproximateVoxelGrid<PointXYZ> approximate_voxel_filter;

    approximate_voxel_filter.setLeafSize(m_filter_x, m_filter_y, m_filter_z);

    approximate_voxel_filter.setInputCloud(point_cloud);
    approximate_voxel_filter.filter(*point_cloud);

    return 1;
}

PointCloud<PointXYZ>::Ptr PointCloudProcessingBackend::to_point_cloud_pointxyz_ptr(Eigen::Vector4f &centroid)
{
    PointCloud<PointXYZ>::Ptr centroid_ptr(new PointCloud<PointXYZ>());

    centroid_ptr.get()->width = 1;
    centroid_ptr.get()->height = 1;
    centroid_ptr.get()->is_dense = false;
    centroid_ptr.get()->points.resize(1);

    centroid_ptr.get()->points[0].x = centroid.coeff(0);
    centroid_ptr.get()->points[0].y = centroid.coeff(1);
    centroid_ptr.get()->points[0].z = centroid.coeff(2);

    return centroid_ptr;
}

int PointCloudProcessingBackend::ricp(PointCloud<PointXYZ>::Ptr &source,
                                      PointCloud<PointXYZ>::Ptr &target,
                                      Eigen::Matrix<float, 4, 4> &final_transformation,
                                      shared_ptr<bool> &has_converged,
                                      shared_ptr<double> &fitness_score)
{
    IterativeClosestPoint<PointXYZ, PointXYZ> icp;

    icp.setInputSource(source);
    icp.setInputTarget(target);

    PointCloud<PointXYZ>::Ptr final(new PointCloud<PointXYZ>());

    icp.align(*final);

    final_transformation = icp.getFinalTransformation();

    *has_converged = icp.hasConverged();

    *fitness_score = icp.getFitnessScore();

    return 1;
}

int PointCloudProcessingBackend::rndt(PointCloud<PointXYZ>::Ptr &source,
                                      PointCloud<PointXYZ>::Ptr &target,
                                      Eigen::Matrix<float, 4, 4> &final_transformation,
                                      shared_ptr<bool> &has_converged,
                                      shared_ptr<double> &fitness_score)
{
    NormalDistributionsTransform<PointXYZ, PointXYZ> ndt;

    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0f);
    ndt.setMaximumIterations(35);

    ndt.setInputSource(source);
    ndt.setInputTarget(target);

    Eigen::AngleAxisf rotation(0.6931f, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f translation(1.79387f, 0.720047f, 0);
    Eigen::Matrix4f guess_matrix = (rotation * translation).matrix();

    PointCloud<PointXYZ>::Ptr final(new PointCloud<PointXYZ>());

    ndt.align(*final, guess_matrix);

    final_transformation = ndt.getFinalTransformation();

    *has_converged = ndt.hasConverged();

    *fitness_score = ndt.getFitnessScore();

    return 1;
}

int PointCloudProcessingBackend::visualise(PointCloud<PointXYZ>::Ptr &source,
                                           PointCloud<PointXYZ>::Ptr source_centroid,
                                           PointCloud<PointXYZ>::Ptr &target,
                                           PointCloud<PointXYZ>::Ptr target_centroid,
                                           Eigen::Matrix<float, 4, 4> &transformation)
{
    transformPointCloud(*source, *source, transformation);

    PCLVisualizer *visualiser = new PCLVisualizer("PCL Viewer");

    visualiser->setBackgroundColor(0, 0, 0);

    PointCloudColorHandlerCustom<PointXYZ> source_color(source, 255, 0, 0);

    visualiser->addPointCloud<PointXYZ>(source, source_color, "source cloud");
    visualiser->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");

    PointCloudColorHandlerCustom<PointXYZ> source_centroid_colour(source_centroid, 255, 255, 0);

    visualiser->addPointCloud<PointXYZ>(source_centroid, source_centroid_colour, "source centroid");
    visualiser->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 10, "source centroid");

    PointCloudColorHandlerCustom<PointXYZ> target_colour(target, 0, 0, 255);

    visualiser->addPointCloud<PointXYZ>(target, target_colour, "target cloud");
    visualiser->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

    PointCloudColorHandlerCustom<PointXYZ> target_centroid_colour(target_centroid, 0, 255, 255);

    visualiser->addPointCloud<PointXYZ>(target_centroid, target_centroid_colour, "target centroid");
    visualiser->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 10, "target centroid");

    visualiser->addCoordinateSystem(1.0, "global");
    visualiser->initCameraParameters();

    visualiser->spin();

    visualiser->close();

    return 1;
}

int PointCloudProcessingBackend::destructor(bool hard)
{
    if(hard)
    {

    }

    return 1;
}
