#ifndef POINTCLOUDPROCESSINGBACKEND_H
#define POINTCLOUDPROCESSINGBACKEND_H

#include<array>
#include<vector>
#include<fstream>

using namespace std;

//!
//! \class PointCloudProcessingBackend
//! \brief
//! \details
//!
class PointCloudProcessingBackend
{
public:

    static int calculate_point_cloud();

    static int write_point_cloud_to_file();

    static int average_point_cloud_buffer(vector<float> &, vector<vector<float>> &, array<unsigned short, 2> &);

};

#endif // POINTCLOUDPROCESSINGBACKEND_H
