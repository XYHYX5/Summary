#include "lidar_frame.h"
#include "tinyxml2.h"

#include <string.h>
#include <math.h>

LidarFrame::LidarFrame()
{
    variableInit();
}

LidarFrame::~LidarFrame()
{
    variableFree();
}

int LidarFrame::setGrid(char *config_file)
{
    tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();
    doc->LoadFile( config_file );
    int erro_id = doc->ErrorID();
    if(erro_id==tinyxml2::XML_SUCCESS)
    {
        printf("LOG:CONFIG XML_SUCCESS\n");
    }
    if(erro_id==tinyxml2::XML_ERROR_FILE_NOT_FOUND)
    {
        printf("ERO:XML_ERROR_FILE_NOT_FOUND\n");
        return 0;
    }
    if(erro_id==tinyxml2::XML_ERROR_FILE_COULD_NOT_BE_OPENED)
    {
        printf("ERO:XML_ERROR_FILE_COULD_NOT_BE_OPENED\n");
        return 0;
    }
    if(erro_id==tinyxml2::XML_ERROR_FILE_READ_ERROR)
    {
        printf("ERO:XML_ERROR_FILE_READ_ERROR\n");
        return 0;
    }
    tinyxml2::XMLElement * db = doc->RootElement()->FirstChildElement("config");
    tinyxml2::XMLElement * settings = db->FirstChildElement("settings");
    tinyxml2::XMLElement * thresholds = db->FirstChildElement("thresholds");
    tinyxml2::XMLElement * ts = db->FirstChildElement("tr")->FirstChildElement("t")->FirstChildElement("item");
    tinyxml2::XMLElement * rs = db->FirstChildElement("tr")->FirstChildElement("r")->FirstChildElement("item");

    grid_setting->grid_size_x = settings->FirstChildElement("sizex")->IntText();
    grid_setting->grid_size_y = settings->FirstChildElement("sizey")->IntText();
    grid_setting->offset_x = settings->FirstChildElement("offsetx")->IntText();
    grid_setting->offset_y = settings->FirstChildElement("offsety")->IntText();
    grid_setting->resolution_x = settings->FirstChildElement("resx")->IntText();
    grid_setting->resolution_y = settings->FirstChildElement("resy")->IntText();
    grid_setting->roi_end_angle = settings->FirstChildElement("endangle")->IntText();
    grid_setting->roi_start_angle = settings->FirstChildElement("startangle")->IntText();

    grid_threshold->car_back_grid = thresholds->FirstChildElement("carback")->IntText();
    grid_threshold->car_front_grid = thresholds->FirstChildElement("carfront")->IntText();
    grid_threshold->car_left_grid = thresholds->FirstChildElement("carleft")->IntText();
    grid_threshold->car_right_grid = thresholds->FirstChildElement("carright")->IntText();
    grid_threshold->obs_line = thresholds->FirstChildElement("obsline")->IntText();
    grid_threshold->filter_windowsize = thresholds->FirstChildElement("windsize")->IntText();
    grid_threshold->hang_z_gap = thresholds->FirstChildElement("hangz")->IntText();
    grid_threshold->hd_threshold = thresholds->FirstChildElement("hd")->IntText();
    grid_threshold->rough_threshold = thresholds->FirstChildElement("rough")->IntText();
    grid_threshold->ang_gap = thresholds->FirstChildElement("anggap")->IntText();
    grid_threshold->first_step = thresholds->FirstChildElement("fstep")->IntText();
    grid_threshold->after_step = thresholds->FirstChildElement("afstep")->IntText();

    for(int i=0;i<3;i++)
    {
        tr->t[i] = ts->DoubleText();
        ts = ts->NextSiblingElement();
    }
    for(int i=0;i<9;i++)
    {
        tr->r[i] = rs->DoubleText();
        rs = rs->NextSiblingElement();
    }

    delete doc;

    grid_point.resize(grid_setting->grid_size_x*grid_setting->grid_size_y);
    grid_value.resize(grid_setting->grid_size_x*grid_setting->grid_size_y);
    grid_label.resize(grid_setting->grid_size_x*grid_setting->grid_size_y);
    all_points.resize(grid_setting->grid_size_x*grid_setting->grid_size_y);
    inv_obs_grid.resize(grid_setting->grid_size_x*grid_setting->grid_size_y);
    return 1;
}

int LidarFrame::process(Point2I guiding_point)
{
    fillGrid();
    calGrid();
    getRoad(guiding_point);
    //labelWaterCliff();
    return 1;
}


int LidarFrame::output(char *grid_obs, Boundary & output_boundary)
{
    for(int i=0;i<grid_label.size();i++)
    {
        if(grid_label[i].pos_obs)
            grid_obs[i] = 1;
        else
            grid_obs[i] = 0;
    }
    memcpy(&output_boundary,res_boundary,sizeof(Boundary));
    return 1;
}

int LidarFrame::debugPointCloud(Point3II *outmp_pcloud, int &point_num)
{
    point_num = 0;
    for(int i=0;i<grid_point.size();i++)
    {
        if(grid_point[i].point.empty())
            continue;
        else
        {
            for(int j=0;j<grid_point[i].point.size();j++)
                outmp_pcloud[point_num++] = grid_point[i].point[j];
        }
    }
    return 1;
}

int LidarFrame::debugBoundary(Point2I *out_opoint, int &opoint_num)
{
    opoint_num = 0;
    Point2I tmp;
    tmp.x = 0;
    tmp.y = 0;
    out_opoint[opoint_num++] = tmp;
    for(int i=0;i<res_cross->cross_num;i++)
    {
        for(int j=0;j<res_cross->step_num[i];j++)
        {
            tmp.x = res_cross->cross_point_x[i][j]*25-2000;
            tmp.y = res_cross->cross_point_y[i][j]*25-2000;
            out_opoint[opoint_num++] = tmp;
        }
    }
    return 1;
}

int LidarFrame::input(vector<Point3FI> input_data)
{
    for(int i=0;i<grid_point.size();i++)
    {
        grid_point[i].point.clear();
        grid_point[i].point.shrink_to_fit();
    }
    for(int i=0;i<all_points.size();i++)
    {
        all_points[i].clear();
        all_points[i].shrink_to_fit();
    }
    for(int i=0;i<input_data.size();i++)
    {
        if(input_data[i].y<1500&&input_data[i].z>0)
            continue;
        if(input_data[i].x<600&&input_data[i].x>-600&&input_data[i].y<600&&input_data[i].y>-600&input_data[i].z>0)
            continue;
            
        int x_id = getXindex(input_data[i].x*tr->r[0] + input_data[i].y*tr->r[1] + input_data[i].z*tr->r[2] + tr->t[0]);
        int y_id = getYindex(input_data[i].x*tr->r[3] + input_data[i].y*tr->r[4] + input_data[i].z*tr->r[5] + tr->t[1]);
        //in grid range
        if(-1==x_id||-1==y_id)
            continue;
        //not in car range
        if(y_id>=grid_threshold->car_back_grid+grid_setting->offset_y
                &&y_id<=grid_threshold->car_front_grid+grid_setting->offset_y
                &&x_id>=grid_threshold->car_left_grid+grid_setting->offset_x
                &&x_id<=grid_threshold->car_right_grid+grid_setting->offset_x)
            continue;
        PointIndex tmp_p;
        tmp_p.x = round(input_data[i].x*tr->r[0] + input_data[i].y*tr->r[1] + input_data[i].z*tr->r[2] + tr->t[0]);
        tmp_p.y = round(input_data[i].x*tr->r[3] + input_data[i].y*tr->r[4] + input_data[i].z*tr->r[5] + tr->t[1]);
        tmp_p.z = round(input_data[i].x*tr->r[6] + input_data[i].y*tr->r[7] + input_data[i].z*tr->r[8] + tr->t[2]);
        tmp_p.x_id = x_id;
        tmp_p.y_id = y_id;
        all_points[x_id*grid_setting->grid_size_y + y_id].push_back(tmp_p);
    }
    return 1;
}


// int LidarFrame::fillGrid()
// {
//     //remove hang obs
//     int window_size = grid_threshold->filter_windowsize;
//     for(int x_id=window_size;x_id<grid_setting->grid_size_x-window_size;x_id++)
//     {
//         for(int y_id=window_size;y_id<grid_setting->grid_size_y-window_size;y_id++)
//         {
//             if(all_points[x_id*grid_setting->grid_size_y + y_id].empty())
//                 continue;
//             vector<PointIndex> sort_points;
//             for(int i=-window_size;i<=window_size;i++)
//             {
//                 for(int j=-window_size;j<=window_size;j++)
//                 {
//                     if(all_points[(x_id+i)*grid_setting->grid_size_y + y_id+j].empty())
//                         continue;
//                     else
//                         sort_points.insert(sort_points.end(),all_points[(x_id+i)*grid_setting->grid_size_y + y_id+j].begin(),
//                                 all_points[(x_id+i)*grid_setting->grid_size_y + y_id+j].end());
//                 }
//             }
//             sort(sort_points.begin(),sort_points.end(),comparePointIndex);

//             int smaller_z = 0;
//             int larger_z = 0;
//             int con_z_gap = 0;
//             int max_id = sort_points.size()-1;
//             int min_id = 0;
//             for(int k=0;k<sort_points.size()-1;k++)
//             {
//                 larger_z = sort_points[k+1].z;
//                 smaller_z = sort_points[k].z;
//                 con_z_gap = larger_z - smaller_z;
//                 if(con_z_gap>grid_threshold->hang_z_gap)
//                 {
//                     if(fabs((double)larger_z)>fabs((double)smaller_z))
//                         max_id = k;
//                     else
//                         min_id = k+1;
//                     break;
//                 }
//             }
//             for(int k=min_id;k<=max_id;k++)
//             {
//                 Point3II tmp;
//                 tmp.x = sort_points[k].x;
//                 tmp.y = sort_points[k].y;
//                 tmp.z = sort_points[k].z;
//                 tmp.intensity = 255;
//                 //bug
//                 vector<Point3II>::iterator ifind = find(grid_point[(sort_points[k].x_id)*grid_setting->grid_size_y + (sort_points[k].y_id)].point.begin(),
//                  grid_point[(sort_points[k].x_id)*grid_setting->grid_size_y + (sort_points[k].y_id)].point.end(), tmp);
//                 if (ifind != grid_point[(sort_points[k].x_id)*grid_setting->grid_size_y + (sort_points[k].y_id)].point.end())
//                 {
//                     continue;
//                 }
//                 grid_point[(sort_points[k].x_id)*grid_setting->grid_size_y + (sort_points[k].y_id)].point.push_back(tmp);
//                 grid_point[(sort_points[k].x_id)*grid_setting->grid_size_y + (sort_points[k].y_id)].is_full = 1;

//             }
//             sort_points.clear();
//             sort_points.shrink_to_fit();
//         }
//     }

//     return 1;
// }

int LidarFrame::fillGrid()
{
    //remove hang obs
    int window_size = grid_threshold->filter_windowsize;
    for(int x_id=0;x_id<grid_setting->grid_size_x;x_id++)
    {
        for(int y_id=0;y_id<grid_setting->grid_size_y;y_id++)
        {
            vector<PointIndex> sort_points;
            for(int i=-window_size;i<=window_size;i++)
            {
                for(int j=-window_size;j<=window_size;j++)
                {
                    if(x_id+i<0||x_id+i>=grid_setting->grid_size_x||y_id+j<0||y_id+j>=grid_setting->grid_size_y)
                        continue;
                    if(all_points[(x_id+i)*grid_setting->grid_size_y + y_id+j].empty())
                        continue;
                    else
                        sort_points.insert(sort_points.end(),all_points[(x_id+i)*grid_setting->grid_size_y + y_id+j].begin(),
                                all_points[(x_id+i)*grid_setting->grid_size_y + y_id+j].end());
                }
            }
            if(sort_points.empty())
                continue;
            sort(sort_points.begin(),sort_points.end(),comparePointIndex);

            int smaller_z = 0;
            int larger_z = 0;
            int con_z_gap = 0;
            int max_id = sort_points.size()-1;
            int min_id = 0;
            for(int k=0;k<sort_points.size()-1;k++)
            {
                larger_z = sort_points[k+1].z;
                smaller_z = sort_points[k].z;
                con_z_gap = larger_z - smaller_z;
                if(con_z_gap>grid_threshold->hang_z_gap)
                {
                    if(fabs((double)larger_z)>fabs((double)smaller_z))
                        max_id = k;
                    else
                        min_id = k+1;
                    break;
                }
            }
            for(int k=min_id;k<=max_id;k++)
            {
                //bug
                if(sort_points[k].x_id!=x_id-window_size || sort_points[k].y_id!=y_id-window_size)
                    continue;
                Point3II tmp;
                tmp.x = sort_points[k].x;
                tmp.y = sort_points[k].y;
                tmp.z = sort_points[k].z;
                tmp.intensity = 255;
                grid_point[(sort_points[k].x_id)*grid_setting->grid_size_y + (sort_points[k].y_id)].point.push_back(tmp);
                grid_point[(sort_points[k].x_id)*grid_setting->grid_size_y + (sort_points[k].y_id)].is_full = 1;
            }
            sort_points.clear();
            sort_points.shrink_to_fit();
        }
    }

    return 1;
}


int LidarFrame::calGrid()
{
    for(int i=0;i<grid_point.size();i++)
    {
        if(grid_point[i].point.empty())
        {
            grid_label[i].pos_obs = 0;
            inv_obs_grid[i] = 255;
            continue;
        }
        else
        {
            grid_value[i].count = grid_point[i].point.size();
            grid_value[i].filled = 1;
            grid_value[i].sum = 0;
            grid_value[i].z_max = -10000;
            grid_value[i].z_min = 10000;
            double zz_sum = 0;
            int line_num = 0;
            for(int j=0;j<grid_point[i].point.size()-1;j++)
            {
                double z_p = grid_point[i].point[j].z;
                grid_value[i].sum += z_p;
                zz_sum += z_p*z_p;
                if(z_p>grid_value[i].z_max)
                    grid_value[i].z_max = z_p;
                if(z_p<grid_value[i].z_min)
                    grid_value[i].z_min = z_p;

                Point3II low = grid_point[i].point[j];
                Point3II high = grid_point[i].point[j+1];
                double angle_low = atan( fabs((double)low.z)/hypot(fabs((double)low.x),fabs((double)low.y)) );
                double angle_high = atan( fabs((double)high.z)/hypot(fabs((double)high.x),fabs((double)high.y)) );
                if(angle_high-angle_low>0.00523)
                    line_num++;
            }
            if(line_num>grid_threshold->obs_line-1)
            {
                grid_label[i].pos_obs = 1;
                inv_obs_grid[i] = 0;
                continue;
            }
            grid_value[i].z_mean =  grid_value[i].sum / grid_value[i].count;
            grid_value[i].z_idw = zz_sum / grid_value[i].count - grid_value[i].z_mean*grid_value[i].z_mean;
            if(grid_value[i].z_max-grid_value[i].z_min>grid_threshold->hd_threshold && grid_value[i].z_idw>grid_threshold->rough_threshold)
            {
                inv_obs_grid[i] = 0;
                grid_label[i].pos_obs = 1;
            }
            else
            {
                grid_label[i].pos_obs = 0;
                inv_obs_grid[i] = 255;
            }
        }
    }
    return 1;
}

// int LidarFrame::calGrid()
// {
//     for(int i=0;i<grid_point.size();i++)
//     {
//         if(grid_point[i].point.empty())
//         {
//             //grid_label[i].pos_obs = 0;
//             continue;
//         }
//         else
//         {
//             grid_value[i].count = grid_point[i].point.size();
//             grid_value[i].filled = 1;
//             grid_value[i].sum = 0;
//             grid_value[i].z_max = -10000;
//             grid_value[i].z_min = 10000;
//             double zz_sum = 0;
//             for(int j=0;j<grid_point[i].point.size()-1;j++)
//             {
//                double z_p = grid_point[i].point[j].z;
//                grid_value[i].sum += z_p;
//                zz_sum += z_p*z_p;
//                if(z_p>grid_value[i].z_max)
//                    grid_value[i].z_max = z_p;
//                if(z_p<grid_value[i].z_min)
//                    grid_value[i].z_min = z_p;
//             }
//            grid_value[i].z_mean =  grid_value[i].sum / grid_value[i].count;
//            grid_value[i].z_idw = zz_sum / grid_value[i].count - grid_value[i].z_mean*grid_value[i].z_mean;
//            if(grid_value[i].z_max-grid_value[i].z_min>grid_threshold->hd_threshold && grid_value[i].z_idw>grid_threshold->rough_threshold)
//                grid_label[i].pos_obs = 1;
//            else
//                grid_label[i].pos_obs = 0;
//         }

//     }
//     return 1;
// }

int LidarFrame::dist_tran()
{
    int i,j;
    double f_min = 0.0;
    double f_dist = 0.0;
    //------from bottom to top ,left to right-------------
    int size_x = grid_setting->grid_size_x;
    int size_y = grid_setting->grid_size_y;
    for(i = 1;i<size_x-1;i++)
    {
        for(j = 1;j<size_y;j++)
        {
            f_dist = hypot(1,1);
            f_min = min(inv_obs_grid[i*size_y+j],f_dist+inv_obs_grid[(i-1)*size_y + j-1]);

            f_dist = hypot(1,0);
            f_min = min(f_min, f_dist+inv_obs_grid[(i-1)*size_y + j]);

            f_dist = hypot(0,1);
            f_min = min(f_min, f_dist+inv_obs_grid[i*size_y + j-1]);

            f_dist = hypot(1,1);
            f_min = min(f_min, f_dist+inv_obs_grid[(i+1)*size_y + j-1]);
            if(f_min>255)
                inv_obs_grid[i*size_y+j] = 255;
            else
                inv_obs_grid[i*size_y+j] = f_min;
        }
    }
    //------from top to bottom ,right to left-------------
    for(i=size_x-2;i>0;i--)
    {
        for(j=size_y-2;j>=0;j--)
        {
            f_dist = hypot(1,0);
            f_min = min(inv_obs_grid[i*size_y+j], f_dist+inv_obs_grid[(i+1)*size_y+j]);

            f_dist = hypot(1,1);
            f_min = min(f_min, f_dist+inv_obs_grid[(i+1)*size_y+j+1]);

            f_dist = hypot(0,1);
            f_min = min(f_min, f_dist+inv_obs_grid[i*size_y+j+1]);

            f_dist = hypot(1,1);
            f_min = min( f_min, f_dist+inv_obs_grid[(i-1)*size_y+j+1]);

            inv_obs_grid[i*size_y+j] = f_min;
        }
    }
    return 1;
}

int LidarFrame::searchAngle(int start_x,int start_y,int start_angle,int end_angle,int step_leng)
{
    int i,j,k,x,y,x1,y1,dx,dy,s1,s2,nowdist;
    bool interchange;
    temp_angle->angle_num = 0;
    for(i=start_angle;i<=end_angle;i++)
    {
        x = start_x;
        y = start_y;
        x1 = round((step_leng+1)*cos_t[i])+x;
        y1 = round((step_leng+1)*sin_t[i])+y;
        dx = abs(x1-x);
        dy = abs(y1-y);
        s1 = x1>x?1:-1;
        s2 = y1>y?1:-1;
        nowdist = 0;
        interchange = false;
        if(dy>dx)
        {
            int temp = dx;
            dx = dy;
            dy = temp;
            interchange = true;
        }
        int p = 2*dy-dx;
        for(j=0;j<dx;j++)
        {
            if (p >= 0)
            {
                if (!interchange)  // 当斜率 < 1 时， 选取上下象素点
                    y += s2;
                else     // 当斜 率 > 1 时，选取左右象素点
                    x += s1;
                p -= 2 * dx;
            }
            if (!interchange)
                x += s1;  // 当斜 率 < 1 时，选取 x 为步长
            else
                y += s2;    // 当斜 率 > 1 时，选取 y 为步长
            p += 2 * dy;
            //---------判断是不是障碍物或者栅格图边界-----------
            if((inv_obs_grid[x*grid_setting->grid_size_y+y]<grid_threshold->car_right_grid+1)||(x<0)
                    ||(x>grid_setting->grid_size_x-1)||(y<0)||(y>grid_setting->grid_size_y-1))
                break;
            //---------------------------------------------------
        }
        //----------判断延伸方向可通行的条件---------------------
        x = (x+1==0)?(x+1):x;
        x = (x==grid_setting->grid_size_x)?(x-1):x;
        y = (y+1==0)?(y+1):y;
        y = (y==grid_setting->grid_size_y)?(y-1):y;
        nowdist = (int)hypot(fabs(x-start_x),fabs(y-start_y));
        if((nowdist>=step_leng)||(x==0)||(x==grid_setting->grid_size_x-1)||(y==0)||(y==grid_setting->grid_size_y-1))
        {
            temp_angle->angle[temp_angle->angle_num] = i;
            temp_angle->point[temp_angle->angle_num][0] = x;
            temp_angle->point[temp_angle->angle_num][1] = y;
            temp_angle->angle_num ++;
        }
        //-------------------------------------------------------
    }
    res_angle->angle_num = 0;
    int beg_ang = 0;
    int end_ang = 0;
    double var_ang;
    int gap_num;
    int tmp_angle_num = temp_angle->angle_num;
    for(i=0;i<tmp_angle_num;)
    {
        beg_ang = temp_angle->angle[i];
        gap_num = 1;
        var_ang = beg_ang;
        for(j=i+1;j<tmp_angle_num;j++)
        {
            if(temp_angle->angle[j]-temp_angle->angle[j-1]>=grid_threshold->ang_gap)
            {
                break;
            }
            else
            {
                end_ang = temp_angle->angle[j];
                gap_num++;
                var_ang += end_ang;
            }
        }
        //----------------相邻方向取平均值作为分支唯一方向---------------
        var_ang = var_ang/gap_num;
        for(k=0;k<tmp_angle_num-1;k++)
        {
            if(fabs(temp_angle->angle[k+1] - var_ang)>=fabs(temp_angle->angle[k] - var_ang))
                break;
        }
        res_angle->angle[res_angle->angle_num] = temp_angle->angle[k];
        res_angle->point[res_angle->angle_num][0] = temp_angle->point[k][0];
        res_angle->point[res_angle->angle_num][1] = temp_angle->point[k][1];
        res_angle->angle_num ++;
        //----------------------------------------------------------------
        i=j;
    }
    if(res_angle->angle_num>MAX_CROSS_NUM||res_angle->angle_num==0)
        return -1;
    else
        return 1;
}

int LidarFrame::iterSearch(int step_leng)
{
    CrossAng iter_ang[MAX_STEP_NUM];
    int i,j,k;
    int start_x,start_y;
    int start_ang,end_ang;
    int iter_num = res_angle->angle_num;
    res_cross->cross_num = 0;
    res_cross->o_x = grid_setting->offset_x;
    res_cross->o_y = grid_setting->offset_y;
    for(i=0;i<MAX_STEP_NUM;i++)
    {
        iter_ang[i].angle_num = 0;
    }
    //---------------此循环迭代更新深度搜索路径-------
    iter_ang[0] = *res_angle;
    for(i=0;i<iter_num;i++)
    {
        start_x = iter_ang[0].point[i][0];
        start_y = iter_ang[0].point[i][1];
        start_ang = iter_ang[0].angle[i] - grid_threshold->ang_gap;
        end_ang = iter_ang[0].angle[i] + grid_threshold->ang_gap;
        if(start_ang < grid_setting->roi_start_angle)
            start_ang = grid_setting->roi_start_angle;
        if(end_ang > grid_setting->roi_end_angle)
            end_ang = grid_setting->roi_end_angle-1;
        searchAngle(start_x,start_y,start_ang,end_ang,step_leng);
        int new_cross_flag = 1;
        for(j=1;j<MAX_STEP_NUM;j++)
        {
            iter_ang[j] = *res_angle;
            if(1!=iter_ang[j].angle_num)			//没有路径，或者有分叉，迭代停止
            {
                new_cross_flag = 0;
                break;
            }
            if((j==MAX_STEP_NUM-1)||(iter_ang[j].point[0][0]==0)||(iter_ang[j].point[0][0]==grid_setting->grid_size_x-1)
                    ||(iter_ang[j].point[0][1]==0)||(iter_ang[j].point[0][1]==grid_setting->grid_size_y-1))			 //迭代到最后一步，且没有分叉，保存该分支路径信息
            {
                break;
            }
            //没有到最深度，且没有分叉，继续迭代
            start_x = iter_ang[j].point[0][0];
            start_y = iter_ang[j].point[0][1];
            start_ang = iter_ang[j].angle[0] - grid_threshold->ang_gap;
            end_ang = iter_ang[j].angle[0] + grid_threshold->ang_gap;
            if(start_ang < grid_setting->roi_start_angle)
                start_ang = grid_setting->roi_start_angle;
            if(end_ang > grid_setting->roi_end_angle)
                end_ang = grid_setting->roi_end_angle-1;
            searchAngle(start_x,start_y,start_ang,end_ang,step_leng);
        }
        if(new_cross_flag)
            res_cross->step_num[i] = j+1;
        else
            res_cross->step_num[i] = j;
        res_cross->cross_angle[res_cross->cross_num][0] = iter_ang[0].angle[i];
        res_cross->cross_point_x[res_cross->cross_num][0] = iter_ang[0].point[i][0];
        res_cross->cross_point_y[res_cross->cross_num][0] = iter_ang[0].point[i][1];
        for(k=1;k<res_cross->step_num[i];k++)
        {
            res_cross->cross_angle[res_cross->cross_num][k] = iter_ang[k].angle[0];
            res_cross->cross_point_x[res_cross->cross_num][k] = iter_ang[k].point[0][0];
            res_cross->cross_point_y[res_cross->cross_num][k] = iter_ang[k].point[0][1];
        }
        res_cross->cross_num++;
    }
    //---------------------------------------------------------
    return 1;
}

int LidarFrame::getRoad(Point2I guiding_point)
{
    memset(res_boundary,0,sizeof(Boundary));
    res_boundary->point_num = 0;
    dist_tran();
    searchAngle(grid_setting->offset_x,grid_setting->offset_y+20,
                grid_setting->roi_start_angle,grid_setting->roi_end_angle,
                grid_threshold->first_step);
    iterSearch(grid_threshold->after_step);

    //chose path
    if(res_cross->cross_num!=1)
        return -1;
    double guiding_angle = 180*atan2(guiding_point.y,guiding_point.x)/M_PI ;
    if(guiding_angle<=-90.0)
        guiding_angle = guiding_angle + 450;
    else
        guiding_angle = guiding_angle + 90;
    int chose_flag = 0;
    double min_angle_gap = 360;
    for(int i=0;i<res_cross->cross_num;i++)
    {
        if(fabs(res_cross->cross_angle[i][0]-guiding_angle)<min_angle_gap)
        {
            chose_flag = i;
            min_angle_gap = fabs(res_cross->cross_angle[i][0]-guiding_angle);
        }
    }

    //filtered boundary
    double var_dist = inv_obs_grid[grid_setting->offset_x*grid_setting->grid_size_y+grid_setting->offset_y];
    double num = 0;
    int out_dist = 0;
    for(int i=0;i<res_cross->step_num[chose_flag];i++)
    {
        int id = res_cross->cross_point_x[chose_flag][i]*grid_setting->grid_size_y
                + res_cross->cross_point_y[chose_flag][i];
        if(inv_obs_grid[id]>20)
            return -1;
        var_dist += inv_obs_grid[id];
        num ++;
    }
    var_dist = var_dist/num;
    out_dist = round(var_dist);

    int start_x = grid_setting->offset_x;
    int start_y = grid_setting->offset_y+20;
    int end_x, end_y;
    int pre_x = start_x;
    int pre_y = start_y;
    for(int i=0;i<res_cross->step_num[chose_flag];i++)
    {
        end_x = res_cross->cross_point_x[chose_flag][i];
        end_y = res_cross->cross_point_y[chose_flag][i];
//brensham
        int x,y,x1,y1,s1,s2,dx,dy;
        bool interchange;
        x = start_x;
        y = start_y;
        x1 = end_x;
        y1 = end_y;
        dx = abs(x1-x);
        dy = abs(y1-y);
        s1 = x1>x?1:-1;
        s2 = y1>y?1:-1;
        interchange = false;
        if(dy>dx)
        {
            int temp = dx;
            dx = dy;
            dy = temp;
            interchange = true;
        }
        int p = 2*dy-dx;
        for(int j=0;j<dx;j++)
        {
            if (p >= 0)
            {
                if (!interchange)
                    y += s2;
                else
                    x += s1;
                p -= 2 * dx;
            }
            if (!interchange)
                x += s1;
            else
                y += s2;
            p += 2 * dy;
            //---------判断是不是障碍物或者栅格图边界-----------
            double now_dist = hypot(fabs(x-pre_x),fabs(y-pre_y));
            if(now_dist>4)
            {
                pre_x = x;
                pre_y = y;
                int k;
                for(k=0;k<out_dist;k++)
                {
                    if(inv_obs_grid[(x+k)*grid_setting->grid_size_y+y]<=1||x+k==grid_setting->grid_size_x-1)
                        break;
                }
                if(x+k<grid_setting->grid_size_x-1)
                {
                    res_boundary->points[res_boundary->point_num].x
                        = (x+k - grid_setting->offset_x)*grid_setting->resolution_x;
                    res_boundary->points[res_boundary->point_num].y
                        = (y - grid_setting->offset_y)*grid_setting->resolution_y;
                    res_boundary->point_type[res_boundary->point_num] = 2;
                    res_boundary->point_num++;
                }
                for(k=0;k<out_dist;k++)
                {
                    if(inv_obs_grid[(x-k)*grid_setting->grid_size_y+y]<=1||x-k==0)
                        break;
                }
                if(x-k>0)
                {
                    res_boundary->points[res_boundary->point_num].x
                        = (x-k - grid_setting->offset_x)*grid_setting->resolution_x;
                    res_boundary->points[res_boundary->point_num].y
                            = (y - grid_setting->offset_y)*grid_setting->resolution_y;
                    res_boundary->point_type[res_boundary->point_num] = 1;
                    res_boundary->point_num++;
                }
            }
            //---------------------------------------------------
        }
        start_x = end_x;
        start_y = end_y;
    }
    return 1;
}


int LidarFrame::labelWaterCliff()
{
    return 1;
}

bool LidarFrame::comparePointIndex(PointIndex a,PointIndex b)
{
    return a.z<b.z;
}

int LidarFrame::getXindex(double x)
{
    int x_index = round(x/grid_setting->resolution_x) + grid_setting->offset_x;
    if(x_index>=grid_setting->grid_size_x||x_index<0)
        return -1;
    else
        return x_index;
}

int LidarFrame::getYindex(double y)
{
    int y_index = round(y/grid_setting->resolution_y) + grid_setting->offset_y;
    if(y_index>=grid_setting->grid_size_y||y_index<0)
        return -1;
    else
        return y_index;
}

void LidarFrame::variableInit()
{
    grid_setting = new GridSettings;
    grid_threshold = new ThresholdParam;
    tr = new TRValues;
    temp_angle = new CrossAng;
    res_angle = new CrossAng;
    res_cross = new CrossPoint;
    res_boundary = new Boundary;
    for(int i=0;i<360;i++)
    {
        cos_t[i] = cos(M_PI*(i-90.0)/180.0);
        sin_t[i] = sin(M_PI*(i-90.0)/180.0);
    }
}

void LidarFrame::variableFree()
{
    grid_point.clear();
    grid_value.clear();
    grid_label.clear();
    inv_obs_grid.clear();

    if(grid_setting)
        delete grid_setting;
    if(grid_threshold)
        delete grid_threshold;
    if(tr)
        delete tr;
    if(temp_angle)
        delete temp_angle;
    if(res_angle)
        delete res_angle;
    if(res_cross)
        delete res_cross;
    if(res_boundary)
        delete res_boundary;
}
