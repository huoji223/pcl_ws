#include "pcl_test_core.h"

PclTestCore::PclTestCore(ros::NodeHandle &nh)
{
    sub_point_cloud_ = nh.subscribe("/velodyne_points", 10, &PclTestCore::point_cb, this);//订阅/velodyne_points话题的雷达数据并在PclTestCore类中的point_cb函数通过指针对数据进行操作

    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_ground", 10);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_no_ground", 10);
    //pub_test_ = nh.advertise<sensor_msgs::PointCloud2>("/test_points_", 10); //自建测试话题
    ros::spin(); //调用spin，开始subscribe消息
}

PclTestCore::~PclTestCore() {}

void PclTestCore::Spin()
{
}

//可以看出以上三步呼应pcl_test_node.cpp

void PclTestCore::clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,//切除过高点云
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        if (in->points[i].z > clip_height)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices 删除索引
    cliper.filter(*out);
}

void PclTestCore::remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,//切除过近点云(使用pcl::ExtractIndices进行剪裁)
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if (distance < min_distance)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}





/*!
 *
 * @param[in] in_cloud Input Point Cloud to be organized in radial segments 输入点云以径向线段组织
 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data 用 XYZRTZColor 数据填充的自定义点云
 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment 每个径向线段的原始云中点的索引
 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered 点云向量，每个元素将包含有序的点
此函数将XYZI点云格式转化为PointCloudXYZIRTColor，定义在头文件中
 */
void PclTestCore::XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                   PointCloudXYZIRTColor &out_organized_points,
                                   std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                   std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds)
{
    out_organized_points.resize(in_cloud->points.size());//resize为重设容器大小函数
    out_radial_divided_indices.clear();
/*vector.clear()函数并不会把所有元素清零。vector有两个参数，一个是size，表示当前vector容器内存储的元素个数，一个是capacity，表示当前vector在内存中申请的这片区域所能容纳的元素个数。通常capacity会比size大，如果往vector中push_back数据，这样就不用重新申请内存和拷贝元素到新内存区域了，便于节省时间。所以vector.clear()的真正作用是：把size设置成0，capacity不变。*/
    out_radial_divided_indices.resize(radial_dividers_num_);
    out_radial_ordered_clouds.resize(radial_dividers_num_);

    for (size_t i = 0; i < in_cloud->points.size(); i++)
    {
        PointXYZIRTColor new_point;
        auto radius = (float)sqrt(
            in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);//radius表示点到lidar的水平距离(半径)
        auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;//theta是点相对于车头正方向(即x方向)的夹角，公式如前
        if (theta < 0)
        {
            theta += 360;
        }
        //角度的微分
        auto radial_div = (size_t)floor(theta / RADIAL_DIVIDER_ANGLE);
        //半径的微分
        auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

        new_point.point = in_cloud->points[i];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radial_div = radial_div;
        new_point.concentric_div = concentric_div;
        new_point.original_index = i;

        out_organized_points[i] = new_point;

        //radial divisions更加角度的微分组织射线
        out_radial_divided_indices[radial_div].indices.push_back(i);

        out_radial_ordered_clouds[radial_div].push_back(new_point);

    } //end for

/*
我们用radial_div和concentric_div分别描述角度微分和距离微分。对点云进行水平角度微分之后，可得到： 360 0.18 = 2000 \frac{360}{0.18} = 2000 0.18360​=2000 条射线，将这些射线中的点按照距离的远近进行排序。下一个for是排序代码：
*/
#pragma omp for
    for (size_t i = 0; i < radial_dividers_num_; i++)
    {
        std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                  [](const PointXYZIRTColor &a, const PointXYZIRTColor &b) { return a.radius < b.radius; });
    }
}



/*!
 * Classifies Points in the PointCoud as Ground and Not Ground
 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin 按距原点的径向距离排序的有序 PointsCloud 的向量
 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud 返回原始 PointCloud 中分类为地面的点的索引
 * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud 返回原始 PointCloud 中分类为非地面的点的索引
 
*/
void PclTestCore::classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices,
                              pcl::PointIndices &out_no_ground_indices)
{
    out_ground_indices.indices.clear();
    out_no_ground_indices.indices.clear();
#pragma omp for
    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) //sweep through each radial division 遍历每一根射线
    {
        float prev_radius = 0.f;
        float prev_height = -SENSOR_HEIGHT;
        bool prev_ground = false;
        bool current_ground = false;
        for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) //loop through each point in the radial div
        {
            float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
            float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;//设定的同条射线上邻近两点的坡度阈值
            float current_height = in_radial_ordered_clouds[i][j].point.z;
            float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;


/*通过local_max_slope_和坡度阈值以及当前点的半径（到lidar的水平距离）求得高度阈值，通过判断当前点的高度（即点的z值）是否在地面加减高度阈值范围内来判断当前点是为地面
*/

            //for points which are very close causing the height threshold to be tiny, set a minimum value
            if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
            {
                height_threshold = min_height_threshold_;
            }

            //check current point height against the LOCAL threshold (previous point)
            if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
            {
                //Check again using general geometry (radius from origin) if previous points wasn't ground
                if (!prev_ground)
                {
                    if (current_height <= (-SENSOR_HEIGHT + general_height_threshold) && current_height >= (-SENSOR_HEIGHT - general_height_threshold))
                    {
                        current_ground = true;
                    }
                    else
                    {
                        current_ground = false;
                    }
                }
                else
                {
                    current_ground = true;
                }
            }
            else
            {
                //check if previous point is too far from previous one, if so classify again
                if (points_distance > reclass_distance_threshold_ &&
                    (current_height <= (-SENSOR_HEIGHT + height_threshold) && current_height >= (-SENSOR_HEIGHT - height_threshold)))
                {
                    current_ground = true;
                }
                else
                {
                    current_ground = false;
                }
            }

            if (current_ground)
            {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            }
            else
            {
                out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = false;
            }

            prev_radius = in_radial_ordered_clouds[i][j].radius;
            prev_height = in_radial_ordered_clouds[i][j].point.z;
        }
    }
}

void PclTestCore::publish_cloud(const ros::Publisher &in_publisher,//发布节点
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                                const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}

void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)//主函数(回调函数)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cliped_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr remove_close(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);//使用pcl::fromROSMsg和pcl::toROSMsg将sensor_msgs::PointCloud2转化为pcl::PointCloud<T>

    clip_above(CLIP_HEIGHT, current_pc_ptr, cliped_pc_ptr);//切除过高点云后输出cliped_pc_ptr

    remove_close_pt(MIN_DISTANCE, cliped_pc_ptr, remove_close);//切除过近点云后输出remove_close

    PointCloudXYZIRTColor organized_points;
    std::vector<pcl::PointIndices> radial_division_indices;//径向划分索引
    std::vector<pcl::PointIndices> closest_indices;
    std::vector<PointCloudXYZIRTColor> radial_ordered_clouds;//点云向量

/*
vector 是同一种类型的对象的集合,每个对象都有一个对应的整数索引值。 和 string 对象一样,标准库将负责管理与存储元素相关的内存。 我们把 vector称为容器,是因为它可以包含其他对象，能够存放任意类型的 动态数组 ，增加和压缩数据。
*/

    radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);//ceil为向上取整

    XYZI_to_RTZColor(remove_close, organized_points,
                     radial_division_indices, radial_ordered_clouds);

    pcl::PointIndices ground_indices, no_ground_indices;

    classify_pc(radial_ordered_clouds, ground_indices, no_ground_indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    
    /*
    测试模块
    pcl::PointCloud<pcl::PointXYZI>::Ptr pub_test_ptr(new pcl::PointCloud<pcl::PointXYZI>); //自建测试话题
    pcl::PointCloud<pcl::PointXYZI>::Ptr test_points;
    pcl::copyPointCloud(*remove_close, *test_points);
    publish_cloud(pub_test_, pub_test_ptr, in_cloud_ptr->header);
    */

    pcl::ExtractIndices<pcl::PointXYZI> extract_ground;//ExtractIndices滤波器，基于某一分割算法提取点云中的一个子集。
    extract_ground.setInputCloud(remove_close);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));

    extract_ground.setNegative(false); //true removes the indices, false leaves only the indices 创建滤波器对象
    extract_ground.filter(*ground_cloud_ptr);

    extract_ground.setNegative(true); //true removes the indices, false leaves only the indices
    extract_ground.filter(*no_ground_cloud_ptr);

    ////pub for debug
    // sensor_msgs::PointCloud2 pub_pc;
    // pcl::toROSMsg(*remove_close, pub_pc);

    // pub_pc.header = in_cloud_ptr->header;

    // pub_ground_.publish(pub_pc);

    publish_cloud(pub_ground_, ground_cloud_ptr, in_cloud_ptr->header);
    publish_cloud(pub_no_ground_, no_ground_cloud_ptr, in_cloud_ptr->header);
}
