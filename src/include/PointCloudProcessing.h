#ifndef POINTCLOUDPROCESSING_H
#define POINTCLOUDPROCESSING_H

#include<array>
#include<vector>

using namespace std;

//!
//! \class
//! \brief
//! \details
//!
class PointCloudProcessing
{
public:

    static int average_point_cloud_buffer(vector<float> &, vector<vector<float>> &, array<unsigned short, 2> &);

};

#endif // POINTCLOUDPROCESSING_H
