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
                                                float z1, float x2, float y2, float z2, float x0, float y0, float z0, float *transform)
    {

        float srx = sin(transform[0]);
        float crx = cos(transform[0]);
        float sry = sin(transform[1]);
        float cry = cos(transform[1]);
        float srz = sin(transform[2]);
        float crz = cos(transform[2]);
        float tx = transform[3];
        float ty = transform[4];
        float tz = transform[5];

        int s = 1;

        // det of cross product
        float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

        float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

        //compute the derivative of distance error function to x0, y0, and z0
        float d_x0 = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

        float d_y0 = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

        float d_z0 = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

        // compute the derivative of x0, y0, z0 to rx, ry, rz, tx, ty, tz, respectively

        float d_x0_rx = (-crx * sry * srz * x00 + crx * crz * sry * y00 + srx * sry * z00 + tx * crx * sry * srz - ty * crx * crz * sry - tz * srx * sry);

        float d_y0_rx = (srx * srz * x00 - crz * srx * y00 + crx * z00 + ty * crz * srx - tz * crx - tx * srx * srz);

        float d_z0_rx = (crx * cry * srz * x00 - crx * cry * crz * y00 - cry * srx * z00 + tz * cry * srx + ty * crx * cry * crz - tx * crx * cry * srz);

        float d_x0_ry = ((-crz * sry - cry * srx * srz) * x00 + (cry * crz * srx - sry * srz) * y00 - crx * cry * z00 + tx * (crz * sry + cry * srx * srz) + ty * (sry * srz - cry * crz * srx) + tz * crx * cry);

        float d_y0_ry = 0;

        float d_z0_ry = ((cry * crz - srx * sry * srz) * x00 + (cry * srz + crz * srx * sry) * y00 - crx * sry * z00 + tz * crx * sry - ty * (cry * srz + crz * srx * sry) - tx * (cry * crz - srx * sry * srz));

        float d_x0_rz = ((-cry * srz - crz * srx * sry) * x00 + (cry * crz - srx * sry * srz) * y00 + tx * (cry * srz + crz * srx * sry) - ty * (cry * crz - srx * sry * srz));

        float d_y0_rz = (-crx * crz * x00 - crx * srz * y00 + ty * crx * srz + tx * crx * crz);

        float d_z0_rz = ((cry * crz * srx - sry * srz) * x00 + (crz * sry + cry * srx * srz) * y00 + tx * (sry * srz - cry * crz * srx) - ty * (crz * sry + cry * srx * srz));

        float d_x0_tx = -(cry * crz - srx * sry * srz);

        float d_y0_tx = crx * srz;

        float d_z0_tx = - (crz * sry + cry * srx * srz);

        float d_x0_ty = -(cry * srz + crz * srx * sry);

        float d_y0_ty = - crx * crz;

        float d_z0_ty = - (sry * srz - cry * crz * srx);

        float d_x0_tz = crx * sry;

        float d_y0_tz = - srx;

        float d_z0_tz = - crx * cry;

        // chain rules
        float df_drx = d_x0 * d_x0_rx + d_y0 * d_y0_rx + d_z0 * d_z0_rx;

        float df_dry = d_x0 * d_x0_ry + d_y0 * d_y0_ry + d_z0 * d_z0_ry;

        float df_drz = d_x0 * d_x0_rz + d_y0 * d_y0_rz + d_z0 * d_z0_rz;

        float df_dtx = d_x0 * d_x0_tx + d_y0 * d_y0_tx + d_z0 * d_z0_tx;

        float df_dty = d_x0 * d_y0_ty + d_y0 * d_y0_ty + d_z0 * d_z0_ty;

        float df_dtz = d_x0 * d_x0_tz + d_y0 * d_y0_tz + d_z0 * d_z0_tz;

        pcl::PointNormal edge_jacobians;
        edge_jacobians.x = df_drx;
        edge_jacobians.y = df_dry;
        edge_jacobians.z = df_drz;
        edge_jacobians.normal_x = df_dtx;
        edge_jacobians.normal_y = df_dty;
        edge_jacobians.normal_z = df_dtz;

        return edge_jacobians;
    }
    inline pcl::PointNormal calc_plane_jacobians(float x00, float y00, float z00, float x1, float y1,
                                                 float z1, float x2, float y2, float z2, float x3, float y3, float z3, float *transform)
    {
        float srx = sin(transform[0]);
        float crx = cos(transform[0]);
        float sry = sin(transform[1]);
        float cry = cos(transform[1]);
        float srz = sin(transform[2]);
        float crz = cos(transform[2]);
        float tx = transform[3];
        float ty = transform[4];
        float tz = transform[5];

        float d_x0 = (y2 - y1) * (z3 - z1) - (y3 - y1) * (z2 - z1);
        float d_y0 = (z2 - z1) * (x3 - x1) - (z3 - z1) * (x2 - x1);
        float d_z0 = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);

        float ps = sqrt(d_x0 * d_x0 + d_y0 * d_y0 + d_z0 * d_z0);
        d_x0 /= ps;
        d_y0 /= ps;
        d_z0 /= ps;

            // compute the derivative of x0, y0, z0 to rx, ry, rz, tx, ty, tz, respectively

        float d_x0_rx = (-crx * sry * srz * x00 + crx * crz * sry * y00 + srx * sry * z00 + tx * crx * sry * srz - ty * crx * crz * sry - tz * srx * sry);

        float d_y0_rx = (srx * srz * x00 - crz * srx * y00 + crx * z00 + ty * crz * srx - tz * crx - tx * srx * srz);

        float d_z0_rx = (crx * cry * srz * x00 - crx * cry * crz * y00 - cry * srx * z00 + tz * cry * srx + ty * crx * cry * crz - tx * crx * cry * srz);

        float d_x0_ry = ((-crz * sry - cry * srx * srz) * x00 + (cry * crz * srx - sry * srz) * y00 - crx * cry * z00 + tx * (crz * sry + cry * srx * srz) + ty * (sry * srz - cry * crz * srx) + tz * crx * cry);

        float d_y0_ry = 0;

        float d_z0_ry = ((cry * crz - srx * sry * srz) * x00 + (cry * srz + crz * srx * sry) * y00 - crx * sry * z00 + tz * crx * sry - ty * (cry * srz + crz * srx * sry) - tx * (cry * crz - srx * sry * srz));

        float d_x0_rz = ((-cry * srz - crz * srx * sry) * x00 + (cry * crz - srx * sry * srz) * y00 + tx * (cry * srz + crz * srx * sry) - ty * (cry * crz - srx * sry * srz));

        float d_y0_rz = (-crx * crz * x00 - crx * srz * y00 + ty * crx * srz + tx * crx * crz);

        float d_z0_rz = ((cry * crz * srx - sry * srz) * x00 + (crz * sry + cry * srx * srz) * y00 + tx * (sry * srz - cry * crz * srx) - ty * (crz * sry + cry * srx * srz));

        float d_x0_tx = -(cry * crz - srx * sry * srz);

        float d_y0_tx = crx * srz;

        float d_z0_tx = - (crz * sry + cry * srx * srz);

        float d_x0_ty = -(cry * srz + crz * srx * sry);

        float d_y0_ty = - crx * crz;

        float d_z0_ty = - (sry * srz - cry * crz * srx);

        float d_x0_tz = crx * sry;

        float d_y0_tz = - srx;

        float d_z0_tz = - crx * cry;

        // chain rules
        float df_drx = d_x0 * d_x0_rx + d_y0 * d_y0_rx + d_z0 * d_z0_rx;

        float df_dry = d_x0 * d_x0_ry + d_y0 * d_y0_ry + d_z0 * d_z0_ry;

        float df_drz = d_x0 * d_x0_rz + d_y0 * d_y0_rz + d_z0 * d_z0_rz;

        float df_dtx = d_x0 * d_x0_tx + d_y0 * d_y0_tx + d_z0 * d_z0_tx;

        float df_dty = d_x0 * d_y0_ty + d_y0 * d_y0_ty + d_z0 * d_z0_ty;

        float df_dtz = d_x0 * d_x0_tz + d_y0 * d_y0_tz + d_z0 * d_z0_tz;
        pcl::PointNormal plane_jacobians;
        plane_jacobians.x = df_drx;
        plane_jacobians.y = df_dry;
        plane_jacobians.z = df_drz;
        plane_jacobians.normal_x = df_dtx;
        plane_jacobians.normal_y = df_dty;
        plane_jacobians.normal_z = df_dtz;

        return plane_jacobians;
    }
};
#endif
