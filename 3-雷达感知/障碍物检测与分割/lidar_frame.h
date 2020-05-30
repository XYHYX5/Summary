#ifndef __LIDAR_FRAME_H__
#define __LIDAR_FRAME_H__

#include "common.h"
#include <vector>
#include <algorithm>
using namespace std;

#pragma pack(push)
#pragma pack(1)
enum Point_Flag
{
    NORMAL = 0,
    FILTERED,
    NO_RETURN,
    STRONGEST,
    LATEST
};

typedef struct tagOriginData
{
    PointLRDI  line_point[LASER_NUM][MAX_LINE_POINT];
    enum Point_Flag line_point_flag[LASER_NUM][MAX_LINE_POINT];
    int line_point_num[LASER_NUM];
}OriginData,*OriginData_ptr;

typedef struct tagPointCloud
{
    Point3II line_point_cloud[LASER_NUM][MAX_LINE_POINT];
    enum Point_Flag line_point_flag[LASER_NUM][MAX_LINE_POINT];
    int line_point_num[LASER_NUM];

}PointCloud,*PointCloud_ptr;

typedef struct tagGridValue
{
    double z_min;
    double z_max;
    double z_mean;
    int count;
    double z_idw;
    double sum;
    int filled;
}GridValue,*GridValue_ptr;

typedef struct tagGridPoint
{
    vector<Point3II> point;
    int min_z_label;
    int max_z_label;
    char is_full;
    char is_hanging_obs;
}GridPoint,*GridPoint_ptr;


typedef struct tagGridLabel
{
    char safe_area;
    char pos_obs;
    char neg_obs;
    char water_area;
    char shadow_area;
    char dynamic_area;
    char hang_area;
    char cliff_area;
    double safe_prob;
}GridLabel,*GridLabel_ptr;

typedef struct tagThresholdParam
{
    int car_front_grid;
    int car_right_grid;
    int car_left_grid;
    int car_back_grid;
    int obs_line;
    int hang_z_gap;
    int hd_threshold;
    int filter_windowsize;
    double rough_threshold;

    int ang_gap;
    int first_step;
    int after_step;
}ThresholdParam,*ThresholdParam_ptr;

typedef struct tagGridSettings
{
    int resolution_x;
    int resolution_y;
    int grid_size_x;
    int grid_size_y;
    int offset_x;
    int offset_y;
    int roi_start_angle;//-y:0-360
    int roi_end_angle;

}GridSettings,*GridSettings_ptr;

typedef struct tagTRValues
{
  double r[9];
  double t[3];
}TRValues,*TRValues_ptr;


#define MAX_ANGLE_NUM 360
#define MAX_STEP_NUM  4
#define MAX_CROSS_NUM 50
typedef struct tagCrossAng
{
    int angle_num;
    double angle[MAX_ANGLE_NUM];
    int point[MAX_ANGLE_NUM][2];
}CrossAng,*CrossAng_ptr;

typedef struct tagCrossPoint
{
    int cross_num;
    int o_x;
    int o_y;
    int step_num[MAX_CROSS_NUM];
    double cross_angle[MAX_CROSS_NUM][MAX_STEP_NUM];
    int cross_point_x[MAX_CROSS_NUM][MAX_STEP_NUM];
    int cross_point_y[MAX_CROSS_NUM][MAX_STEP_NUM];
}CrossPoint,*CrossPoint_ptr;

typedef struct tagBoundary
{
    int point_num;
    Point2I points[200];
    char point_type[200];
}Boundary,*Boundary_ptr;


#pragma pack(pop)


class LidarFrame
{
public:
    LidarFrame();
    ~LidarFrame();

    int setGrid(char * config_file);

    int input(vector<Point3FI> input_data);
    int process(Point2I guiding_point);
    int output(char * grid_obs, Boundary &output_boundary);
    int debugPointCloud(Point3II * out_pointcloud,int & point_num);
    int debugBoundary(Point2I * out_opoint,int & opoint_num);

private:
    //grid
    vector<GridPoint> grid_point;
    vector<GridValue> grid_value;
    vector<GridLabel> grid_label;
    vector< vector<PointIndex> > all_points;

    //settings and threshold
    GridSettings_ptr grid_setting;
    ThresholdParam_ptr grid_threshold;
    TRValues_ptr tr;

    //boundary
    double cos_t[360];
    double sin_t[360];
    vector<double> inv_obs_grid;
    CrossAng_ptr temp_angle;				//存放中间运算结果
    CrossAng_ptr res_angle;              //存放每一步最终角度结果
    CrossPoint_ptr res_cross;            //存放路口检测结果
    Boundary_ptr res_boundary;

    //common
    void variableInit();
    void variableFree();
    int getXindex(double x);
    int getYindex(double y);
    static bool comparePointIndex(PointIndex a,PointIndex b);

    //grid
    int fillGrid();
    int calGrid();
    //boundary
    int dist_tran();
    int searchAngle(int start_x, int start_y, int start_angle, int end_angle, int step_leng);
    int iterSearch(int step_leng);
    int getRoad(Point2I guiding_point);
    //cliff
    int labelWaterCliff();


};

#endif
