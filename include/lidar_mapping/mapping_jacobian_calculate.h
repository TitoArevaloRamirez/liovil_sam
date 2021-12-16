#ifndef EDGE_PLANE_FNS_H
#define EDGE_PLANE_FNS_H
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

class edgePlaneJacobian
{
  public:
    inline pcl::PointNormal calc_edge_jacobians(float x00, float y00, float z00, float x1, float y1,
                                                float z1, float x2, float y2, float z2, float x0, float y0, float z0, float *transformTobeMapped)
    {

        float srx = sin(transformTobeMapped[0]);
        float crx = cos(transformTobeMapped[0]);
        float sry = sin(transformTobeMapped[1]);
        float cry = cos(transformTobeMapped[1]);
        float srz = sin(transformTobeMapped[2]);
        float crz = cos(transformTobeMapped[2]);

        // det of cross product
        float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

        float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

        float ld2 = a012 / l12;

        float s = 1 - 0.9 * fabs(ld2);

        //compute the derivative of distance error function to x0, y0, and z0
        float d_x0 = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

        float d_y0 = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

        float d_z0 = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

        d_x0 = d_x0 * s;
        d_y0 = d_y0 * s;
        d_z0 = d_z0 * s;

        // compute the derivative of x0, y0, z0 to rx, ry, rz, tx, ty, tz, respectively

        float arx = (crx * sry * srz * x00 + crx * crz * sry * y00 - srx * sry * z00) * d_x0 + (-srx * srz * x00 - crz * srx * y00 - crx * z00) * d_y0 + (crx * cry * srz * x00 + crx * cry * crz * y00 - cry * srx * z00) * d_z0;

        float ary = ((cry * srx * srz - crz * sry) * x00 + (sry * srz + cry * crz * srx) * y00 + crx * cry * z00) * d_x0 + ((-cry * crz - srx * sry * srz) * x00 + (cry * srz - crz * srx * sry) * y00 - crx * sry * z00) * d_z0;

        float arz = ((crz * srx * sry - cry * srz) * x00 + (-cry * crz - srx * sry * srz) * y00) * d_x0 + (crx * crz * x00 - crx * srz * y00) * d_y0 + ((sry * srz + cry * crz * srx) * x00 + (crz * sry - cry * srx * srz) * y00) * d_z0;

        pcl::PointNormal edge_jacobians;
        edge_jacobians.x = d_x0;
        edge_jacobians.y = d_y0;
        edge_jacobians.z = d_z0;
        edge_jacobians.normal_x = arx;
        edge_jacobians.normal_y = ary;
        edge_jacobians.normal_z = arz;

        return edge_jacobians;
    }
    inline pcl::PointNormal calc_plane_jacobians(float x00, float y00, float z00, float pa,
    float pb, float pc, float s, float* transformTobeMapped)
    {
        float srx = sin(transformTobeMapped[0]);
        float crx = cos(transformTobeMapped[0]);
        float sry = sin(transformTobeMapped[1]);
        float cry = cos(transformTobeMapped[1]);
        float srz = sin(transformTobeMapped[2]);
        float crz = cos(transformTobeMapped[2]);

        float d_x0 = s * pa;
        float d_y0 = s * pb;
        float d_z0 = s * pc;

        float arx = (crx * sry * srz * x00 + crx * crz * sry * y00 - srx * sry * z00) * d_x0 + (-srx * srz * x00 - crz * srx * y00 - crx * z00) * d_y0 + (crx * cry * srz * x00 + crx * cry * crz * y00 - cry * srx * z00) * d_z0;

        float ary = ((cry * srx * srz - crz * sry) * x00 + (sry * srz + cry * crz * srx) * y00 + crx * cry * z00) * d_x0 + ((-cry * crz - srx * sry * srz) * x00 + (cry * srz - crz * srx * sry) * y00 - crx * sry * z00) * d_z0;

        float arz = ((crz * srx * sry - cry * srz) * x00 + (-cry * crz - srx * sry * srz) * y00) * d_x0 + (crx * crz * x00 - crx * srz * y00) * d_y0 + ((sry * srz + cry * crz * srx) * x00 + (crz * sry - cry * srx * srz) * y00) * d_z0;

        pcl::PointNormal plane_jacobians;
        plane_jacobians.x = d_x0;
        plane_jacobians.y = d_y0;
        plane_jacobians.z = d_z0;
        plane_jacobians.normal_x = arx;
        plane_jacobians.normal_y = ary;
        plane_jacobians.normal_z = arz;

        return plane_jacobians;
    }
};
#endif
