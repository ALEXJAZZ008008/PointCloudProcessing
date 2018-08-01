#include "src/include/PointCloudProcessingObject.h"

PointCloudProcessingObject::PointCloudProcessingObject():
    m_data(0, 0.0),
    m_resolution(3, 0),
    m_data_path(""),
    m_real_timestamp(0),
    m_relative_timestamp(0),
    m_data_size(0)
{

}

PointCloudProcessingObject::~PointCloudProcessingObject()
{
    destructor(true);
}

PointCloudProcessingObject::PointCloudProcessingObject(PointCloudProcessingObject &point_cloud_processing_ref):
    m_data(point_cloud_processing_ref.get_data()),
    m_resolution(point_cloud_processing_ref.get_resolution()),
    m_data_path(point_cloud_processing_ref.get_data_path()),
    m_real_timestamp(point_cloud_processing_ref.get_real_timestamp()),
    m_relative_timestamp(point_cloud_processing_ref.get_relative_timestamp()),
    m_data_size(point_cloud_processing_ref.get_data_size())
{

}

PointCloudProcessingObject & PointCloudProcessingObject::operator = (PointCloudProcessingObject &point_cloud_processing_ref)
{
    m_data = point_cloud_processing_ref.get_data();
    m_resolution = point_cloud_processing_ref.get_resolution();
    m_data_path = point_cloud_processing_ref.get_data_path();
    m_real_timestamp = point_cloud_processing_ref.get_real_timestamp();
    m_relative_timestamp = point_cloud_processing_ref.get_relative_timestamp();
    m_data_size = point_cloud_processing_ref.get_data_size();

    return *this;
}

PointCloudProcessingObject::PointCloudProcessingObject(PointCloudProcessingObject &&point_cloud_processing_ref_ref):
    m_data(point_cloud_processing_ref_ref.get_data()),
    m_resolution(point_cloud_processing_ref_ref.get_resolution()),
    m_data_path(point_cloud_processing_ref_ref.get_data_path()),
    m_real_timestamp(point_cloud_processing_ref_ref.get_real_timestamp()),
    m_relative_timestamp(point_cloud_processing_ref_ref.get_relative_timestamp()),
    m_data_size(point_cloud_processing_ref_ref.get_data_size())
{

}

PointCloudProcessingObject & PointCloudProcessingObject::operator = (PointCloudProcessingObject &&point_cloud_processing_ref_ref)
{
    m_data = point_cloud_processing_ref_ref.get_data();
    m_resolution = point_cloud_processing_ref_ref.get_resolution();
    m_data_path = point_cloud_processing_ref_ref.get_data_path();
    m_real_timestamp = point_cloud_processing_ref_ref.get_real_timestamp();
    m_relative_timestamp = point_cloud_processing_ref_ref.get_relative_timestamp();
    m_data_size = point_cloud_processing_ref_ref.get_data_size();

    return *this;
}

int PointCloudProcessingObject::point_cloud_processing_main()
{
    return 1;
}

int PointCloudProcessingObject::point_cloud_processing_kill(bool hard)
{
    destructor(hard);

    return 1;
}

int PointCloudProcessingObject::destructor(bool hard)
{
    if(hard)
    {

    }

    return 1;
}
