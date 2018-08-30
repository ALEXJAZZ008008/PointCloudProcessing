#include "src/include/PointCloudProcessingBackend.h"

PointCloudProcessingBackend::PointCloudProcessingBackend():
    m_header_map(),
    m_data_map(),
    m_objects(0, nullptr),
    m_input_path(""),
    m_output_path(""),
    m_log(""),
    m_threshold(0.0),
    m_distance_movement(0.0),
    m_eigen_movement(0.0),
    m_smoothing_deviation(0.0),
    m_transformation_epsilon(0.0),
    m_focal_length(0.0f),
    m_filter_x(0.0f),
    m_filter_y(0.0f),
    m_filter_z(0.0f),
    m_rotation_guess(0.0f),
    m_translation_guess_x(0.0f),
    m_translation_guess_y(0.0f),
    m_translation_guess_z(0.0f),
    m_signal_magnitude(0.0f),
    m_signal_x(0.0f),
    m_signal_y(0.0f),
    m_signal_z(0.0f),
    m_cloud_point_size(0),
    m_centroid_point_size(0),
    m_offset(0),
    m_smoothing_size(0),
    m_iterations(0),
    m_cloud_one_r(0),
    m_cloud_one_g(0),
    m_cloud_one_b(0),
    m_cloud_two_r(0),
    m_cloud_two_g(0),
    m_cloud_two_b(0),
    m_centroid_one_r(0),
    m_centroid_one_g(0),
    m_centroid_one_b(0),
    m_centroid_two_r(0),
    m_centroid_two_g(0),
    m_centroid_two_b(0),
    m_signal_r(0),
    m_signal_g(0),
    m_signal_b(0),
    m_point_cloud_text(false),
    m_point_cloud_binary(false),
    m_test(false),
    m_visualisation(false),
    m_translation_text(false),
    m_translation_binary(false),
    m_icp(false),
    m_ndt(false),
    m_iterative(false),
    m_continuous(false),
    m_distance(false),
    m_eigen(false),
    m_manual(false),
    m_auto(false),
    m_naive(false),
    m_complex(false)
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
    m_threshold(point_cloud_processing_backend_output_ref.get_threshold()),
    m_distance_movement(point_cloud_processing_backend_output_ref.get_distance_movement()),
    m_eigen_movement(point_cloud_processing_backend_output_ref.get_eigen_movement()),
    m_smoothing_deviation(point_cloud_processing_backend_output_ref.get_smoothing_deviation()),
    m_transformation_epsilon(point_cloud_processing_backend_output_ref.get_transformation_epsilon()),
    m_focal_length(point_cloud_processing_backend_output_ref.get_focal_length()),
    m_filter_x(point_cloud_processing_backend_output_ref.get_filter_x()),
    m_filter_y(point_cloud_processing_backend_output_ref.get_filter_y()),
    m_filter_z(point_cloud_processing_backend_output_ref.get_filter_z()),
    m_rotation_guess(point_cloud_processing_backend_output_ref.get_rotation_guess()),
    m_translation_guess_x(point_cloud_processing_backend_output_ref.get_translation_guess_x()),
    m_translation_guess_y(point_cloud_processing_backend_output_ref.get_translation_guess_y()),
    m_translation_guess_z(point_cloud_processing_backend_output_ref.get_translation_guess_z()),
    m_signal_magnitude(point_cloud_processing_backend_output_ref.get_signal_magnitude()),
    m_signal_x(point_cloud_processing_backend_output_ref.get_signal_x()),
    m_signal_y(point_cloud_processing_backend_output_ref.get_signal_y()),
    m_signal_z(point_cloud_processing_backend_output_ref.get_signal_z()),
    m_cloud_point_size(point_cloud_processing_backend_output_ref.get_cloud_point_size()),
    m_centroid_point_size(point_cloud_processing_backend_output_ref.get_centroid_point_size()),
    m_offset(point_cloud_processing_backend_output_ref.get_offset()),
    m_smoothing_size(point_cloud_processing_backend_output_ref.get_smoothing_size()),
    m_iterations(point_cloud_processing_backend_output_ref.get_iterations()),
    m_cloud_one_r(point_cloud_processing_backend_output_ref.get_cloud_one_r()),
    m_cloud_one_g(point_cloud_processing_backend_output_ref.get_cloud_one_g()),
    m_cloud_one_b(point_cloud_processing_backend_output_ref.get_cloud_one_b()),
    m_cloud_two_r(point_cloud_processing_backend_output_ref.get_cloud_two_r()),
    m_cloud_two_g(point_cloud_processing_backend_output_ref.get_cloud_two_g()),
    m_cloud_two_b(point_cloud_processing_backend_output_ref.get_cloud_two_b()),
    m_centroid_one_r(point_cloud_processing_backend_output_ref.get_centroid_one_r()),
    m_centroid_one_g(point_cloud_processing_backend_output_ref.get_centroid_one_g()),
    m_centroid_one_b(point_cloud_processing_backend_output_ref.get_centroid_one_b()),
    m_centroid_two_r(point_cloud_processing_backend_output_ref.get_centroid_two_r()),
    m_centroid_two_g(point_cloud_processing_backend_output_ref.get_centroid_two_g()),
    m_centroid_two_b(point_cloud_processing_backend_output_ref.get_centroid_one_b()),
    m_signal_r(point_cloud_processing_backend_output_ref.get_signal_r()),
    m_signal_g(point_cloud_processing_backend_output_ref.get_signal_g()),
    m_signal_b(point_cloud_processing_backend_output_ref.get_signal_b()),
    m_point_cloud_text(point_cloud_processing_backend_output_ref.get_point_cloud_text()),
    m_point_cloud_binary(point_cloud_processing_backend_output_ref.get_point_cloud_binary()),
    m_test(point_cloud_processing_backend_output_ref.get_test()),
    m_visualisation(point_cloud_processing_backend_output_ref.get_visualisation()),
    m_translation_text(point_cloud_processing_backend_output_ref.get_translation_text()),
    m_translation_binary(point_cloud_processing_backend_output_ref.get_tranlsation_binary()),
    m_icp(point_cloud_processing_backend_output_ref.get_icp()),
    m_ndt(point_cloud_processing_backend_output_ref.get_ndt()),
    m_iterative(point_cloud_processing_backend_output_ref.get_iterative()),
    m_continuous(point_cloud_processing_backend_output_ref.get_continuous()),
    m_distance(point_cloud_processing_backend_output_ref.get_distance()),
    m_eigen(point_cloud_processing_backend_output_ref.get_eigen()),
    m_manual(point_cloud_processing_backend_output_ref.get_manual()),
    m_auto(point_cloud_processing_backend_output_ref.get_auto()),
    m_naive(point_cloud_processing_backend_output_ref.get_naive()),
    m_complex(point_cloud_processing_backend_output_ref.get_complex())
{

}

PointCloudProcessingBackend & PointCloudProcessingBackend::operator = (PointCloudProcessingBackend &point_cloud_processing_backend_output_ref)
{
    m_header_map = point_cloud_processing_backend_output_ref.get_header_map();
    m_objects = point_cloud_processing_backend_output_ref.get_objects();
    m_input_path = point_cloud_processing_backend_output_ref.get_input_path();
    m_output_path = point_cloud_processing_backend_output_ref.get_output_path();
    m_log = point_cloud_processing_backend_output_ref.get_log();
    m_threshold = point_cloud_processing_backend_output_ref.get_threshold();
    m_distance_movement = point_cloud_processing_backend_output_ref.get_distance_movement();
    m_eigen_movement = point_cloud_processing_backend_output_ref.get_eigen_movement();
    m_smoothing_deviation = point_cloud_processing_backend_output_ref.get_smoothing_deviation();
    m_transformation_epsilon = point_cloud_processing_backend_output_ref.get_transformation_epsilon();
    m_focal_length = point_cloud_processing_backend_output_ref.get_focal_length();
    m_filter_x = point_cloud_processing_backend_output_ref.get_filter_x();
    m_filter_y = point_cloud_processing_backend_output_ref.get_filter_y();
    m_filter_z = point_cloud_processing_backend_output_ref.get_filter_z();
    m_rotation_guess = point_cloud_processing_backend_output_ref.get_rotation_guess();
    m_translation_guess_x = point_cloud_processing_backend_output_ref.get_translation_guess_x();
    m_translation_guess_y = point_cloud_processing_backend_output_ref.get_translation_guess_y();
    m_translation_guess_z = point_cloud_processing_backend_output_ref.get_translation_guess_z();
    m_signal_magnitude = point_cloud_processing_backend_output_ref.get_signal_magnitude();
    m_signal_x = point_cloud_processing_backend_output_ref.get_signal_x();
    m_signal_y = point_cloud_processing_backend_output_ref.get_signal_y();
    m_signal_z = point_cloud_processing_backend_output_ref.get_signal_z();
    m_cloud_point_size = point_cloud_processing_backend_output_ref.get_cloud_point_size();
    m_centroid_point_size = point_cloud_processing_backend_output_ref.get_centroid_point_size();
    m_offset = point_cloud_processing_backend_output_ref.get_offset();
    m_smoothing_size = point_cloud_processing_backend_output_ref.get_smoothing_size();
    m_iterations = point_cloud_processing_backend_output_ref.get_iterations();
    m_cloud_one_r = point_cloud_processing_backend_output_ref.get_cloud_one_r();
    m_cloud_one_g = point_cloud_processing_backend_output_ref.get_cloud_one_g();
    m_cloud_one_b = point_cloud_processing_backend_output_ref.get_cloud_one_b();
    m_cloud_two_r = point_cloud_processing_backend_output_ref.get_cloud_two_r();
    m_cloud_two_g = point_cloud_processing_backend_output_ref.get_cloud_two_g();
    m_cloud_two_b = point_cloud_processing_backend_output_ref.get_cloud_two_b();
    m_centroid_one_r = point_cloud_processing_backend_output_ref.get_centroid_one_r();
    m_centroid_one_g = point_cloud_processing_backend_output_ref.get_centroid_one_g();
    m_centroid_one_b = point_cloud_processing_backend_output_ref.get_centroid_one_b();
    m_centroid_two_r = point_cloud_processing_backend_output_ref.get_centroid_two_r();
    m_centroid_two_g = point_cloud_processing_backend_output_ref.get_centroid_two_g();
    m_centroid_two_b = point_cloud_processing_backend_output_ref.get_centroid_one_b();
    m_signal_r = point_cloud_processing_backend_output_ref.get_signal_r();
    m_signal_g = point_cloud_processing_backend_output_ref.get_signal_g();
    m_signal_b = point_cloud_processing_backend_output_ref.get_signal_b();
    m_point_cloud_text = point_cloud_processing_backend_output_ref.get_point_cloud_text();
    m_point_cloud_binary = point_cloud_processing_backend_output_ref.get_point_cloud_binary();
    m_test = point_cloud_processing_backend_output_ref.get_test();
    m_visualisation = point_cloud_processing_backend_output_ref.get_visualisation();
    m_translation_text = point_cloud_processing_backend_output_ref.get_translation_text();
    m_translation_binary = point_cloud_processing_backend_output_ref.get_tranlsation_binary();
    m_icp = point_cloud_processing_backend_output_ref.get_icp();
    m_ndt = point_cloud_processing_backend_output_ref.get_ndt();
    m_iterative = point_cloud_processing_backend_output_ref.get_iterative();
    m_continuous = point_cloud_processing_backend_output_ref.get_continuous();
    m_distance = point_cloud_processing_backend_output_ref.get_distance();
    m_eigen = point_cloud_processing_backend_output_ref.get_eigen();
    m_manual = point_cloud_processing_backend_output_ref.get_manual();
    m_auto = point_cloud_processing_backend_output_ref.get_auto();
    m_naive = point_cloud_processing_backend_output_ref.get_naive();
    m_complex = point_cloud_processing_backend_output_ref.get_complex();

    return *this;
}

PointCloudProcessingBackend::PointCloudProcessingBackend(PointCloudProcessingBackend &&point_cloud_processing_backend_output_ref_ref):
    m_header_map(point_cloud_processing_backend_output_ref_ref.get_header_map()),
    m_objects(point_cloud_processing_backend_output_ref_ref.get_objects()),
    m_input_path(point_cloud_processing_backend_output_ref_ref.get_input_path()),
    m_output_path(point_cloud_processing_backend_output_ref_ref.get_output_path()),
    m_log(point_cloud_processing_backend_output_ref_ref.get_log()),
    m_threshold(point_cloud_processing_backend_output_ref_ref.get_threshold()),
    m_distance_movement(point_cloud_processing_backend_output_ref_ref.get_distance_movement()),
    m_eigen_movement(point_cloud_processing_backend_output_ref_ref.get_eigen_movement()),
    m_smoothing_deviation(point_cloud_processing_backend_output_ref_ref.get_smoothing_deviation()),
    m_transformation_epsilon(point_cloud_processing_backend_output_ref_ref.get_transformation_epsilon()),
    m_focal_length(point_cloud_processing_backend_output_ref_ref.get_focal_length()),
    m_filter_x(point_cloud_processing_backend_output_ref_ref.get_filter_x()),
    m_filter_y(point_cloud_processing_backend_output_ref_ref.get_filter_y()),
    m_filter_z(point_cloud_processing_backend_output_ref_ref.get_filter_z()),
    m_rotation_guess(point_cloud_processing_backend_output_ref_ref.get_rotation_guess()),
    m_translation_guess_x(point_cloud_processing_backend_output_ref_ref.get_translation_guess_x()),
    m_translation_guess_y(point_cloud_processing_backend_output_ref_ref.get_translation_guess_y()),
    m_translation_guess_z(point_cloud_processing_backend_output_ref_ref.get_translation_guess_z()),
    m_signal_magnitude(point_cloud_processing_backend_output_ref_ref.get_signal_magnitude()),
    m_signal_x(point_cloud_processing_backend_output_ref_ref.get_signal_x()),
    m_signal_y(point_cloud_processing_backend_output_ref_ref.get_signal_y()),
    m_signal_z(point_cloud_processing_backend_output_ref_ref.get_signal_z()),
    m_cloud_point_size(point_cloud_processing_backend_output_ref_ref.get_cloud_point_size()),
    m_centroid_point_size(point_cloud_processing_backend_output_ref_ref.get_centroid_point_size()),
    m_offset(point_cloud_processing_backend_output_ref_ref.get_offset()),
    m_smoothing_size(point_cloud_processing_backend_output_ref_ref.get_smoothing_size()),
    m_iterations(point_cloud_processing_backend_output_ref_ref.get_iterations()),
    m_cloud_one_r(point_cloud_processing_backend_output_ref_ref.get_cloud_one_r()),
    m_cloud_one_g(point_cloud_processing_backend_output_ref_ref.get_cloud_one_g()),
    m_cloud_one_b(point_cloud_processing_backend_output_ref_ref.get_cloud_one_b()),
    m_cloud_two_r(point_cloud_processing_backend_output_ref_ref.get_cloud_two_r()),
    m_cloud_two_g(point_cloud_processing_backend_output_ref_ref.get_cloud_two_g()),
    m_cloud_two_b(point_cloud_processing_backend_output_ref_ref.get_cloud_two_b()),
    m_centroid_one_r(point_cloud_processing_backend_output_ref_ref.get_centroid_one_r()),
    m_centroid_one_g(point_cloud_processing_backend_output_ref_ref.get_centroid_one_g()),
    m_centroid_one_b(point_cloud_processing_backend_output_ref_ref.get_centroid_one_b()),
    m_centroid_two_r(point_cloud_processing_backend_output_ref_ref.get_centroid_two_r()),
    m_centroid_two_g(point_cloud_processing_backend_output_ref_ref.get_centroid_two_g()),
    m_centroid_two_b(point_cloud_processing_backend_output_ref_ref.get_centroid_one_b()),
    m_signal_r(point_cloud_processing_backend_output_ref_ref.get_signal_r()),
    m_signal_g(point_cloud_processing_backend_output_ref_ref.get_signal_g()),
    m_signal_b(point_cloud_processing_backend_output_ref_ref.get_signal_b()),
    m_point_cloud_text(point_cloud_processing_backend_output_ref_ref.get_point_cloud_text()),
    m_point_cloud_binary(point_cloud_processing_backend_output_ref_ref.get_point_cloud_binary()),
    m_test(point_cloud_processing_backend_output_ref_ref.get_test()),
    m_visualisation(point_cloud_processing_backend_output_ref_ref.get_visualisation()),
    m_translation_text(point_cloud_processing_backend_output_ref_ref.get_translation_text()),
    m_translation_binary(point_cloud_processing_backend_output_ref_ref.get_tranlsation_binary()),
    m_icp(point_cloud_processing_backend_output_ref_ref.get_icp()),
    m_ndt(point_cloud_processing_backend_output_ref_ref.get_ndt()),
    m_iterative(point_cloud_processing_backend_output_ref_ref.get_iterative()),
    m_continuous(point_cloud_processing_backend_output_ref_ref.get_continuous()),
    m_distance(point_cloud_processing_backend_output_ref_ref.get_distance()),
    m_eigen(point_cloud_processing_backend_output_ref_ref.get_eigen()),
    m_manual(point_cloud_processing_backend_output_ref_ref.get_manual()),
    m_auto(point_cloud_processing_backend_output_ref_ref.get_auto()),
    m_naive(point_cloud_processing_backend_output_ref_ref.get_naive()),
    m_complex(point_cloud_processing_backend_output_ref_ref.get_complex())
{

}

PointCloudProcessingBackend & PointCloudProcessingBackend::operator = (PointCloudProcessingBackend &&point_cloud_processing_backend_output_ref_ref)
{
    m_header_map = point_cloud_processing_backend_output_ref_ref.get_header_map();
    m_objects = point_cloud_processing_backend_output_ref_ref.get_objects();
    m_input_path = point_cloud_processing_backend_output_ref_ref.get_input_path();
    m_output_path = point_cloud_processing_backend_output_ref_ref.get_output_path();
    m_log = point_cloud_processing_backend_output_ref_ref.get_log();
    m_threshold = point_cloud_processing_backend_output_ref_ref.get_threshold();
    m_distance_movement = point_cloud_processing_backend_output_ref_ref.get_distance_movement();
    m_eigen_movement = point_cloud_processing_backend_output_ref_ref.get_eigen_movement();
    m_smoothing_deviation = point_cloud_processing_backend_output_ref_ref.get_smoothing_deviation();
    m_transformation_epsilon = point_cloud_processing_backend_output_ref_ref.get_transformation_epsilon();
    m_focal_length = point_cloud_processing_backend_output_ref_ref.get_focal_length();
    m_filter_x = point_cloud_processing_backend_output_ref_ref.get_filter_x();
    m_filter_y = point_cloud_processing_backend_output_ref_ref.get_filter_y();
    m_filter_z = point_cloud_processing_backend_output_ref_ref.get_filter_z();
    m_rotation_guess = point_cloud_processing_backend_output_ref_ref.get_rotation_guess();
    m_translation_guess_x = point_cloud_processing_backend_output_ref_ref.get_translation_guess_x();
    m_translation_guess_y = point_cloud_processing_backend_output_ref_ref.get_translation_guess_y();
    m_translation_guess_z = point_cloud_processing_backend_output_ref_ref.get_translation_guess_z();
    m_signal_magnitude = point_cloud_processing_backend_output_ref_ref.get_signal_magnitude();
    m_signal_x = point_cloud_processing_backend_output_ref_ref.get_signal_x();
    m_signal_y = point_cloud_processing_backend_output_ref_ref.get_signal_y();
    m_signal_z = point_cloud_processing_backend_output_ref_ref.get_signal_z();
    m_cloud_point_size = point_cloud_processing_backend_output_ref_ref.get_cloud_point_size();
    m_centroid_point_size = point_cloud_processing_backend_output_ref_ref.get_centroid_point_size();
    m_offset = point_cloud_processing_backend_output_ref_ref.get_offset();
    m_smoothing_size = point_cloud_processing_backend_output_ref_ref.get_smoothing_size();
    m_iterations = point_cloud_processing_backend_output_ref_ref.get_iterations();
    m_cloud_one_r = point_cloud_processing_backend_output_ref_ref.get_cloud_one_r();
    m_cloud_one_g = point_cloud_processing_backend_output_ref_ref.get_cloud_one_g();
    m_cloud_one_b = point_cloud_processing_backend_output_ref_ref.get_cloud_one_b();
    m_cloud_two_r = point_cloud_processing_backend_output_ref_ref.get_cloud_two_r();
    m_cloud_two_g = point_cloud_processing_backend_output_ref_ref.get_cloud_two_g();
    m_cloud_two_b = point_cloud_processing_backend_output_ref_ref.get_cloud_two_b();
    m_centroid_one_r = point_cloud_processing_backend_output_ref_ref.get_centroid_one_r();
    m_centroid_one_g = point_cloud_processing_backend_output_ref_ref.get_centroid_one_g();
    m_centroid_one_b = point_cloud_processing_backend_output_ref_ref.get_centroid_one_b();
    m_centroid_two_r = point_cloud_processing_backend_output_ref_ref.get_centroid_two_r();
    m_centroid_two_g = point_cloud_processing_backend_output_ref_ref.get_centroid_two_g();
    m_centroid_two_b = point_cloud_processing_backend_output_ref_ref.get_centroid_one_b();
    m_signal_r = point_cloud_processing_backend_output_ref_ref.get_signal_r();
    m_signal_g = point_cloud_processing_backend_output_ref_ref.get_signal_g();
    m_signal_b = point_cloud_processing_backend_output_ref_ref.get_signal_b();
    m_point_cloud_text = point_cloud_processing_backend_output_ref_ref.get_point_cloud_text();
    m_point_cloud_binary = point_cloud_processing_backend_output_ref_ref.get_point_cloud_binary();
    m_test = point_cloud_processing_backend_output_ref_ref.get_test();
    m_visualisation = point_cloud_processing_backend_output_ref_ref.get_visualisation();
    m_translation_text = point_cloud_processing_backend_output_ref_ref.get_translation_text();
    m_translation_binary = point_cloud_processing_backend_output_ref_ref.get_tranlsation_binary();
    m_icp = point_cloud_processing_backend_output_ref_ref.get_icp();
    m_ndt = point_cloud_processing_backend_output_ref_ref.get_ndt();
    m_iterative = point_cloud_processing_backend_output_ref_ref.get_iterative();
    m_continuous = point_cloud_processing_backend_output_ref_ref.get_continuous();
    m_distance = point_cloud_processing_backend_output_ref_ref.get_distance();
    m_eigen = point_cloud_processing_backend_output_ref_ref.get_eigen();
    m_manual = point_cloud_processing_backend_output_ref_ref.get_manual();
    m_auto = point_cloud_processing_backend_output_ref_ref.get_auto();
    m_naive = point_cloud_processing_backend_output_ref_ref.get_naive();
    m_complex = point_cloud_processing_backend_output_ref_ref.get_complex();

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

                m_objects[i].get()->set_data_size(static_cast<unsigned char>(stoi(subline[1])));

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

                m_objects[i].get()->set_relative_timestamp(static_cast<unsigned int>(stol(subline[1])));

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
    if(m_test)
    {
        for(unsigned long i = 0; i < m_objects.size(); ++i)
        {
            m_objects[i].get()->get_point_cloud().width = m_objects[i]->get_resolution()[0] * m_objects[i]->get_resolution()[1];
            m_objects[i].get()->get_point_cloud().height = 1;
            m_objects[i].get()->get_point_cloud().is_dense = false;
            m_objects[i].get()->get_point_cloud().points.resize(m_objects[i]->get_resolution()[0] * m_objects[i]->get_resolution()[1]);

            for(unsigned int j = 0; j < m_objects[i]->get_resolution()[1]; ++j)
            {
                for(unsigned int k = 0; k < m_objects[i]->get_resolution()[0]; ++k)
                {
                    float x = (k - (m_objects[i]->get_resolution()[1] / 2.0f)) * (m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] - m_offset) * m_focal_length;

                    float y = (j - (m_objects[i]->get_resolution()[0] / 2.0f)) * (m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] - m_offset) * m_focal_length;

                    float z = m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k];

                    m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].x = x;
                    m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].y = y;
                    m_objects[i].get()->get_point_cloud().points[(m_objects[i]->get_resolution()[0] * j) + k].z = z;
                }
            }

            m_log += "-> pcl " + to_string(i) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
        }
    }
    else
    {
        for(unsigned long i = 0; i < m_objects.size(); ++i)
        {
            m_objects[i].get()->get_point_cloud().width = m_objects[i]->get_resolution()[0] * m_objects[i]->get_resolution()[1];
            m_objects[i].get()->get_point_cloud().height = 1;
            m_objects[i].get()->get_point_cloud().is_dense = false;
            m_objects[i].get()->get_point_cloud().points.resize(m_objects[i]->get_resolution()[0] * m_objects[i]->get_resolution()[1]);

            for(unsigned int j = 0; j < m_objects[i]->get_resolution()[1]; ++j)
            {
                for(unsigned int k = 0; k < m_objects[i]->get_resolution()[0]; ++k)
                {
                    if(m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] > 0.0f || m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] < 0.0f)
                    {
                        float x = (k - (m_objects[i]->get_resolution()[1] / 2.0f)) * (m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] - m_offset) * m_focal_length;

                        float y = (j - (m_objects[i]->get_resolution()[0] / 2.0f)) * (m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k] - m_offset) * m_focal_length;

                        float z = m_objects[i]->get_data()[(m_objects[i]->get_resolution()[0] * j) + k];

                        if(distance(0.0f, 0.0f, 0.0f, x, y, z) < static_cast<float>(m_threshold))
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

    Eigen::Matrix<float, 4, 4> initial_transformation = initial_transformation_init();

    string output_header = output_header_init();

    string output = "";

    while(j < m_objects.size() - 1)
    {
        ++j;

        PointCloud<PointXYZ>::Ptr source(new PointCloud<PointXYZ>(m_objects[i].get()->get_point_cloud()));

        m_log += "-> pcl " + to_string(i) + " " + to_string(source->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

        remove_nan(source);

        m_log += "-> pcl " + to_string(i) + " " + to_string(source->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

        if(!m_test)
        {
            filter(source);

            m_log += "-> pcl " + to_string(i) + " " + to_string(source->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
        }

        if(source->size())
        {

        }

        PointCloud<PointXYZ>::Ptr target(new PointCloud<PointXYZ>(m_objects[j].get()->get_point_cloud()));

        m_log += "-> pcl " + to_string(j) + " " + to_string(target->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

        remove_nan(target);

        m_log += "-> pcl " + to_string(j) + " " + to_string(target->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

        if(!m_test)
        {
            filter(target);

            m_log += "-> pcl " + to_string(j) + " " + to_string(target->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
        }

        Eigen::Vector4f source_centroid;

        compute3DCentroid(*source, source_centroid);

        Eigen::Vector4f target_centroid;

        compute3DCentroid(*target, target_centroid);

        double movement = static_cast<double>(distance(source_centroid.coeff(0),
                                                       source_centroid.coeff(1),
                                                       source_centroid.coeff(2),
                                                       target_centroid.coeff(0),
                                                       target_centroid.coeff(1),
                                                       target_centroid.coeff(2)));

        m_log += "-> movement " + to_string(i) + " " + to_string(j) + ": " + to_string(movement) + "\n";

        if(!m_test)
        {
            if(movement < m_distance_movement)
            {
                m_log += "-> registration " + to_string(i) + " " + to_string(j) + ": " + "Skipped" + "\n";

                output += "Registration " + to_string(i) + " " + to_string(j) + ": Skipped\n";

                continue;
            }
        }

        Eigen::Matrix<float, 4, 4> final_transformation;

        shared_ptr<bool> has_converged(new bool);

        shared_ptr<double> fitness_score(new double);

        if(m_icp)
        {
            ricp(source, target, initial_transformation, final_transformation, has_converged, fitness_score);
        }
        else
        {
            if(m_ndt)
            {
                rndt(source, target, initial_transformation, final_transformation, has_converged, fitness_score);
            }
        }

        if(!m_test)
        {
            if(!has_converged)
            {
                m_log += "-> registration " + to_string(i) + " " + to_string(j) + ": " + "Failed" + "\n";

                output += "Registration " + to_string(i) + " " + to_string(j) + ": Failed\n";

                continue;
            }
        }

        string source_centroid_position = "Source Centroid " + to_string(i) + ": " + to_string(source_centroid.coeff(0)) + "," +
                to_string(source_centroid.coeff(1)) + "," +
                to_string(source_centroid.coeff(2)) + "," +
                to_string(source_centroid.coeff(3));

        string target_centroid_position = "Target Centroid " + to_string(j) + ": " + to_string(target_centroid.coeff(0)) + "," +
                to_string(target_centroid.coeff(1)) + "," +
                to_string(target_centroid.coeff(2)) + "," +
                to_string(target_centroid.coeff(3));

        string centroid_difference = "Centroid Differences " +
                to_string(i) + " " +
                to_string(j) + ": " +
                calculate_vector_difference(target_centroid, source_centroid);

        PointCloud<PointXYZ>::Ptr signal_target(new PointCloud<PointXYZ>(*target));

        Eigen::Vector4f signal_target_centroid;

        string signal_position = "";

        string signal_difference = "";

        PointCloud<PointXYZ>::Ptr signal_cloud(new PointCloud<PointXYZ>());

        Eigen::Matrix<float, 4, 4> signal_transformation;

        if(m_naive)
        {
            PointCloud<PointXYZ>::Ptr signal_source(new PointCloud<PointXYZ>(*source));

            m_log += "-> pcl signal " + to_string(signal_source->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

            calculate_signal_from_centroid(signal_source, source_centroid);

            m_log += "-> pcl signal " + to_string(signal_source->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

            Eigen::Vector4f signal_source_centroid;

            if(signal_source->size() != 0)
            {
                compute3DCentroid(*signal_source, signal_source_centroid);

                signal_position += "Signal " + to_string(i) + ": " + to_string(signal_source_centroid.coeff(0)) + "," +
                        to_string(signal_source_centroid.coeff(1)) + "," +
                        to_string(signal_source_centroid.coeff(2)) + "," +
                        to_string(signal_source_centroid.coeff(3)) + "\n";

                m_log += "-> pcl signal " + to_string(signal_target->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

                calculate_signal_from_centroid(signal_target, target_centroid);

                m_log += "-> pcl signal " + to_string(signal_target->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

                if(signal_target->size() != 0)
                {
                    compute3DCentroid(*signal_target, signal_target_centroid);

                    signal_position += "Signal " + to_string(j) + ": " + to_string(signal_target_centroid.coeff(0)) + "," +
                            to_string(signal_target_centroid.coeff(1)) + "," +
                            to_string(signal_target_centroid.coeff(2)) + "," +
                            to_string(signal_target_centroid.coeff(3));

                    signal_difference += "Signal Differences " +
                            to_string(i) + " " +
                            to_string(j) + ": " +
                            calculate_vector_difference(signal_target_centroid, signal_source_centroid);
                }
                else
                {
                    signal_position += "Signal " + to_string(i) + ": Skipped";

                    signal_difference += "Signal Differences " + to_string(i) + " " + to_string(j) + ": Skipped";
                }
            }
            else
            {
                signal_position += "Signal " + to_string(i) + ": Skipped" + "\n";

                signal_difference += "Signal Differences " + to_string(i) + " " + to_string(j) + ": Skipped";
            }
        }
        else
        {
            if(m_complex)
            {
                signal_cloud->width = source.get()->width;
                signal_cloud->height = source.get()->height;
                signal_cloud->is_dense = source.get()->is_dense;
                signal_cloud->points.resize(source.get()->width * source.get()->height);
                signal_cloud->points = source.get()->points;

                m_log += "-> pcl signal " + to_string(signal_cloud->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

                calculate_signal_from_centroid(signal_cloud, source_centroid);

                m_log += "-> pcl signal " + to_string(signal_cloud->size()) + ": " + to_string(system_clock::now().time_since_epoch().count()) + "\n";

                shared_ptr<bool> signal_has_converged(new bool);

                shared_ptr<double> signal_fitness_score(new double);

                if(signal_cloud->size() != 0)
                {
                    if(m_icp)
                    {
                        ricp(signal_cloud, target, initial_transformation, signal_transformation, signal_has_converged, signal_fitness_score);
                    }
                    else
                    {
                        if(m_ndt)
                        {
                            rndt(signal_cloud, target, initial_transformation, signal_transformation, signal_has_converged, signal_fitness_score);
                        }
                    }

                    string signal_transform = to_string(signal_transformation.coeff(0, 0)) + "," +
                            to_string(signal_transformation.coeff(0, 1)) + "," +
                            to_string(signal_transformation.coeff(0, 2)) + "," +
                            to_string(signal_transformation.coeff(0, 3)) + "," +
                            to_string(signal_transformation.coeff(1, 0)) + "," +
                            to_string(signal_transformation.coeff(1, 1)) + "," +
                            to_string(signal_transformation.coeff(1, 2)) + "," +
                            to_string(signal_transformation.coeff(1, 3)) + "," +
                            to_string(signal_transformation.coeff(2, 0)) + "," +
                            to_string(signal_transformation.coeff(2, 1)) + "," +
                            to_string(signal_transformation.coeff(2, 2)) + "," +
                            to_string(signal_transformation.coeff(2, 3)) + "," +
                            to_string(signal_transformation.coeff(3, 0)) + "," +
                            to_string(signal_transformation.coeff(3, 1)) + "," +
                            to_string(signal_transformation.coeff(3, 2)) + "," +
                            to_string(signal_transformation.coeff(3, 3));

                    signal_difference += "Signal " + to_string(i) + " " + to_string(j) + ":\n" +
                            "Signal Mean Square Error " + to_string(i) + " " + to_string(j) + ": " + to_string(*signal_fitness_score) + "\n" +
                            "Signal Transform " + to_string(i) + " " + to_string(j) + ": " + signal_transform;
                }
            }
            else
            {
                signal_difference += "Signal " + to_string(i) + " " + to_string(j) + ": Skipped";
            }
        }

        if(m_visualisation)
        {
            if(m_naive)
            {
                visualise(source,
                          to_point_cloud_pointxyz_ptr(source_centroid),
                          target,
                          to_point_cloud_pointxyz_ptr(target_centroid),
                          signal_target,
                          to_point_cloud_pointxyz_ptr(signal_target_centroid),
                          final_transformation,
                          signal_transformation);
            }
            else
            {
                if(m_complex)
                {
                    visualise(source,
                              to_point_cloud_pointxyz_ptr(source_centroid),
                              target,
                              to_point_cloud_pointxyz_ptr(target_centroid),
                              signal_cloud,
                              to_point_cloud_pointxyz_ptr(signal_target_centroid),
                              final_transformation,
                              signal_transformation);
                }
            }
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

        output_header += "File Paths " + to_string(i) + " " + to_string(j) + ": " + m_objects[i].get()->get_data_path() + "," + m_objects[j].get()->get_data_path() + "\n";

        if(m_naive)
        {
            output += "Registration " + to_string(i) + " " + to_string(j) + ":\n" +
                    source_centroid_position + "\n" +
                    target_centroid_position + "\n" +
                    centroid_difference + "\n" +
                    signal_position + "\n" +
                    signal_difference + "\n" +
                    "Mean Square Error " + to_string(i) + " " + to_string(j) + ": " + to_string(*fitness_score) + "\n" +
                    "Transform " + to_string(i) + " " + to_string(j) + ": " + transform + "\n";

            m_log += "-> registration " + to_string(i) + " " + to_string(j) + ":\n" +
                    source_centroid_position + "\n" +
                    target_centroid_position + "\n" +
                    centroid_difference + "\n" +
                    signal_position + "\n" +
                    signal_difference + "\n" +
                    "Mean Square Error " + to_string(i) + " " + to_string(j) + ": " + to_string(*fitness_score) + "\n" +
                    "Transform " + to_string(i) + " " + to_string(j) + ": " + transform + "\n";
        }
        else
        {
            if(m_complex)
            {
                output += "Registration " + to_string(i) + " " + to_string(j) + ":\n" +
                        source_centroid_position + "\n" +
                        target_centroid_position + "\n" +
                        centroid_difference + "\n" +
                        signal_difference + "\n" +
                        "Mean Square Error " + to_string(i) + " " + to_string(j) + ": " + to_string(*fitness_score) + "\n" +
                        "Transform " + to_string(i) + " " + to_string(j) + ": " + transform + "\n";

                m_log += "-> registration " + to_string(i) + " " + to_string(j) + ":\n" +
                        source_centroid_position + "\n" +
                        target_centroid_position + "\n" +
                        centroid_difference + "\n" +
                        signal_difference + "\n" +
                        "Mean Square Error " + to_string(i) + " " + to_string(j) + ": " + to_string(*fitness_score) + "\n" +
                        "Transform " + to_string(i) + " " + to_string(j) + ": " + transform + "\n";
            }
        }

        if(m_iterative)
        {
            i = j;
        }
        else
        {
            if(m_continuous)
            {
                i = 0;
            }
        }
    }

    write_translations_to_file(output_header + output);

    return 1;
}

float PointCloudProcessingBackend::distance(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float x = x2 - x1;
    float y = y2 - y1;
    float z = z2 - z1;

    x *= x;
    y *= y;
    z *= z;

    return sqrt(x + y + z);
}

Eigen::Matrix<float, 4, 4> PointCloudProcessingBackend::initial_transformation_init()
{
    Eigen::AngleAxisf rotation(m_rotation_guess, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f translation(m_translation_guess_x, m_translation_guess_y, m_translation_guess_z);
    Eigen::Matrix4f guess_matrix = (rotation * translation).matrix();

    return guess_matrix;
}

string PointCloudProcessingBackend::output_header_init()
{
    string output_header = "";

    if(m_test)
    {
        output_header += "TEST\n";
    }

    output_header += "Registration Type: ";

    if(m_icp)
    {
        output_header += "ICP\n";
    }
    else
    {
        if(m_ndt)
        {
            output_header += "NDT\n";
        }
    }

    output_header += "Registration Style: ";

    if(m_iterative)
    {
        output_header += "Iterative\n";
    }
    else
    {
        if(m_continuous)
        {
            output_header += "Continuous\n";
        }
    }

    output_header += "Movement Type: ";

    if(m_distance)
    {
        output_header += "Distance\nMovement Distance: " + to_string(m_distance_movement) + "\n";
    }
    else
    {
        if(m_eigen)
        {
            output_header += "Eigen\nEigen Distance: " + to_string(m_eigen_movement) + "\n";
        }
    }

    output_header += "Translation Estimation: ";

    if(m_manual)
    {
        output_header += "Manual\n";
    }
    else
    {
        if(m_auto)
        {
            output_header += "Automatic\n";
        }
    }

    output_header += "Tracking Type: ";

    if(m_naive)
    {
        output_header += "Naive\n";
    }
    else
    {
        if(m_complex)
        {
            output_header += "Complex\n";
        }
    }

    output_header += "Threshold: " + to_string(m_threshold) + "\n" +
            "Offset: " + to_string(m_offset) + "\n" +
            "Focal Length: " + to_string(m_focal_length) + "\n" +
            "Smoothing Size: " + to_string(m_smoothing_size) + "\n" +
            "Smoothing Deviation: " + to_string(m_smoothing_deviation) + "\n" +
            "Filter (xyz): " + to_string(m_filter_x) + ", " + to_string(m_filter_y) + ", " + to_string(m_filter_z) + "\n" +
            "Transformation Epsilon: " + to_string(m_transformation_epsilon) + "\n" +
            "Iterations: " + to_string(m_iterations) + "\n" +
            "Rotation Guess: " + to_string(m_rotation_guess) + "\n" +
            "Translation Guess (xyz): " + to_string(m_translation_guess_x) + ", " + to_string(m_translation_guess_y) + ", " + to_string(m_translation_guess_z) + "\n" +
            "Signal Magnitude: " + to_string(m_signal_magnitude) + "\n" +
            "Signal (xyz): " + to_string(m_signal_x) + ", " + to_string(m_signal_y) + ", " + to_string(m_signal_z) + "\n";

    return output_header;
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
    statistical_outlier_removal.setMeanK(m_smoothing_size);
    statistical_outlier_removal.setStddevMulThresh(m_smoothing_deviation);
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
                                      Eigen::Matrix<float, 4, 4> &initial_transformation,
                                      Eigen::Matrix<float, 4, 4> &final_transformation,
                                      shared_ptr<bool> &has_converged,
                                      shared_ptr<double> &fitness_score)
{
    IterativeClosestPoint<PointXYZ, PointXYZ> icp;

    icp.setTransformationEpsilon(m_transformation_epsilon);
    icp.setMaximumIterations(m_iterations);

    icp.setInputSource(source);
    icp.setInputTarget(target);

    PointCloud<PointXYZ>::Ptr final(new PointCloud<PointXYZ>());

    icp.align(*final, initial_transformation);

    if(m_manual)
    {
        initial_transformation = initial_transformation_init();
    }
    else
    {
        if(m_auto)
        {
            initial_transformation = icp.getFinalTransformation();
        }
    }

    final_transformation = icp.getFinalTransformation();

    *has_converged = icp.hasConverged();

    *fitness_score = icp.getFitnessScore();

    return 1;
}

int PointCloudProcessingBackend::rndt(PointCloud<PointXYZ>::Ptr &source,
                                      PointCloud<PointXYZ>::Ptr &target,
                                      Eigen::Matrix<float, 4, 4> &initial_transformation,
                                      Eigen::Matrix<float, 4, 4> &final_transformation,
                                      shared_ptr<bool> &has_converged,
                                      shared_ptr<double> &fitness_score)
{
    NormalDistributionsTransform<PointXYZ, PointXYZ> ndt;

    ndt.setTransformationEpsilon(m_transformation_epsilon);
    ndt.setMaximumIterations(m_iterations);

    ndt.setInputSource(source);
    ndt.setInputTarget(target);

    PointCloud<PointXYZ>::Ptr final(new PointCloud<PointXYZ>());

    ndt.align(*final, initial_transformation);

    if(m_manual)
    {
        initial_transformation = initial_transformation_init();
    }
    else
    {
        if(m_auto)
        {
            initial_transformation = ndt.getFinalTransformation();
        }
    }

    final_transformation = ndt.getFinalTransformation();

    *has_converged = ndt.hasConverged();

    *fitness_score = ndt.getFitnessScore();

    return 1;
}

int PointCloudProcessingBackend::calculate_signal_from_centroid(PointCloud<PointXYZ>::Ptr &point_cloud, Eigen::Vector4f &centroid)
{
    float x = centroid.coeff(0) + m_signal_x;
    float y = centroid.coeff(1) + m_signal_y;
    float z = centroid.coeff(2) + m_signal_z;

    for(unsigned long i = 0; i < point_cloud->size(); ++i)
    {
        if(distance(x, y, z, point_cloud.get()->points[i].x, point_cloud.get()->points[i].y, point_cloud.get()->points[i].z) > m_signal_magnitude)
        {
            point_cloud.get()->points[i].x = numeric_limits<float>::quiet_NaN();
            point_cloud.get()->points[i].y = numeric_limits<float>::quiet_NaN();
            point_cloud.get()->points[i].y = numeric_limits<float>::quiet_NaN();
        }
    }

    remove_nan(point_cloud);

    return 1;
}

string PointCloudProcessingBackend::calculate_vector_difference(Eigen::Vector4f &vector_one, Eigen::Vector4f &vector_two)
{
    Eigen::Vector4f temp;

    temp(0, 0) = vector_one.coeff(0) - vector_two.coeff(0);
    temp(1, 0) = vector_one.coeff(1) - vector_two.coeff(1);
    temp(2, 0) = vector_one.coeff(2) - vector_two.coeff(2);
    temp(3, 0) = vector_one.coeff(3) - vector_two.coeff(3);

    return to_string(temp.coeff(0)) + "," +
            to_string(temp.coeff(1)) + "," +
            to_string(temp.coeff(2)) + "," +
            to_string(temp.coeff(3));
}

int PointCloudProcessingBackend::visualise(PointCloud<PointXYZ>::Ptr &source,
                                           PointCloud<PointXYZ>::Ptr source_centroid,
                                           PointCloud<PointXYZ>::Ptr &target,
                                           PointCloud<PointXYZ>::Ptr target_centroid,
                                           PointCloud<PointXYZ>::Ptr &signal,
                                           PointCloud<PointXYZ>::Ptr signal_centroid,
                                           Eigen::Matrix<float, 4, 4> &final_transformation,
                                           Eigen::Matrix<float, 4, 4> &signal_transformation)
{
    transformPointCloud(*source, *source, final_transformation);

    if(m_complex)
    {
        transformPointCloud(*signal, *signal, signal_transformation);
    }

    PCLVisualizer *visualiser = new PCLVisualizer("PCL Viewer");

    visualiser->setBackgroundColor(0, 0, 0);

    PointCloudColorHandlerCustom<PointXYZ> source_color(source, m_cloud_one_r, m_cloud_one_g, m_cloud_one_b);

    visualiser->addPointCloud<PointXYZ>(source, source_color, "source cloud");
    visualiser->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, m_cloud_point_size, "source cloud");

    PointCloudColorHandlerCustom<PointXYZ> source_centroid_colour(source_centroid, m_centroid_one_r, m_centroid_one_g, m_centroid_one_b);

    visualiser->addPointCloud<PointXYZ>(source_centroid, source_centroid_colour, "source centroid");
    visualiser->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, m_centroid_point_size, "source centroid");

    PointCloudColorHandlerCustom<PointXYZ> target_colour(target, m_cloud_two_r, m_cloud_two_g, m_cloud_two_b);

    visualiser->addPointCloud<PointXYZ>(target, target_colour, "target cloud");
    visualiser->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, m_cloud_point_size, "target cloud");

    PointCloudColorHandlerCustom<PointXYZ> target_centroid_colour(target_centroid, m_centroid_two_r, m_centroid_two_g, m_centroid_two_b);

    visualiser->addPointCloud<PointXYZ>(target_centroid, target_centroid_colour, "target centroid");
    visualiser->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, m_centroid_point_size, "target centroid");

    if(signal->size() > 0)
    {
        PointCloudColorHandlerCustom<PointXYZ> signal_colour(signal, m_signal_r, m_signal_g, m_signal_b);

        visualiser->addPointCloud<PointXYZ>(signal, signal_colour, "signal cloud");
        visualiser->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, m_cloud_point_size, "signal cloud");

        if(m_naive)
        {
            PointCloudColorHandlerCustom<PointXYZ> signal_centroid_colour(signal_centroid, m_signal_r, m_signal_g, m_signal_b);

            visualiser->addPointCloud<PointXYZ>(signal_centroid, signal_centroid_colour, "signal centroid");
            visualiser->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, m_centroid_point_size, "signal centroid");
        }
    }

    visualiser->addCoordinateSystem(1.0, "global");
    visualiser->initCameraParameters();

    visualiser->spin();

    visualiser->close();

    return 1;
}

int PointCloudProcessingBackend::write_translations_to_file(string output)
{
    if(m_translation_binary)
    {
        ofstream registration_bin_stream(m_output_path + "/registration_bin_" + to_string(system_clock::now().time_since_epoch().count()) + ".bin", ios::out | ios::binary);

        registration_bin_stream.write(reinterpret_cast<char *>(&output), sizeof(output));

        registration_bin_stream.flush();
        registration_bin_stream.close();

        m_log += "<- registration_bin: " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
    }

    if(m_translation_text)
    {
        ofstream registration_txt_stream(m_output_path + "/registration_txt_" + to_string(system_clock::now().time_since_epoch().count()) + ".txt", ios::out);

        registration_txt_stream << output << endl;

        registration_txt_stream.flush();
        registration_txt_stream.close();

        m_log += "<- registration_txt: " + to_string(system_clock::now().time_since_epoch().count()) + "\n";
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
