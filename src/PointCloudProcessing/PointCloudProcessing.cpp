#include "src/include/PointCloudProcessing.h"

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
