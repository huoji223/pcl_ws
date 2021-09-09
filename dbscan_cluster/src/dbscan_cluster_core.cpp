#include "dbscan_cluster_core.h"
#include "DBSCAN_simple.h"
#include "DBSCAN_precomp.h"
#include "DBSCAN_kdtree.h"
#include <pcl/features/moment_of_inertia_estimation.h>
#include <math.h>

EuClusterCore::EuClusterCore(ros::NodeHandle &nh)
{
    sub_point_cloud_ = nh.subscribe("/filtered_points_no_ground", 5, &EuClusterCore::point_cb, this);//this 是 C++ 中的一个关键字，也是一个 const 指针，它指向当前对象，通过它可以访问当前对象的所有成员。

    pub_cluster_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/pub_cluster_cloud", 10);
    pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_boxs", 5);

    //chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

    cout << "test";
    ros::spin();
}

EuClusterCore::~EuClusterCore() {}

void EuClusterCore::publish_cloud(const ros::Publisher &in_publisher,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                                  const std_msgs::Header &in_header)//发布XYZI点云的函数
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}

void EuClusterCore::voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size)//
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;//实例化过滤器
    filter.setInputCloud(in);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*out);
}

void EuClusterCore::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list)
/*
聚类,而后将一个个点云，计算其形心作为该障碍物的中心，同时计算点云簇的长宽高，从而确定一个能够将点云簇包裹的三维Bounding Box
*/
{

    clock_t start_ms = clock();
    // KdTree, for more information, please ref [How to use a KdTree to search](https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html#kdtree-search)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    if (in_pc->points.size() > 0)
        tree->setInputCloud(in_pc);
    // Segmentation分割, [Euclidean Cluster Extraction欧几里德集群提取](https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html#cluster-extraction)
    std::vector<pcl::PointIndices> cluster_indices;
    
    // test 1.取消以下两行测试 simple dbscan
    //DBSCANSimpleCluster<pcl::PointXYZ> ec;
    //ec.setCorePointMinPts(20);

    // test 2.取消以下两行以测试 precomp dbscan
    //DBSCANPrecompCluster<pcl::PointXYZ>  ec;
    //ec.setCorePointMinPts(20);

    // test 3.取消以下两行测试 dbscan 与 Kdtree 加速
    DBSCANKdtreeCluster<pcl::PointXYZ> ec;
    ec.setCorePointMinPts(20);

    // test 4.取消以下行以测试欧几里德光谱排除(EuclideanClusterExtraction)
    // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance(0.6);//设置近邻搜索的搜索半径
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);//设置点云的搜索机制
    ec.setInputCloud(in_pc);
    ec.extract(cluster_indices);//从点云中提取聚类

    //bbox obb
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        // the structure to save one detected object
        Detected_Obj obj_info;//Detected_Obj ，用于存储检测到的障碍物的信息

        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();
        

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*in_pc)[*pit]); 

/*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
*/
	//TODO::定义一些需要用到的变量向量
	//TODO::惯性矩moment_of_inertia
	std::vector <float> moment_of_inertia;
	//TODO::偏心率eccentricity
	std::vector <float> eccentricity;
	//TODO::声明OBB的最小三坐标min_point_OBB和最大三坐标max_point_OBB
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	//TODO::声明OBB position_OBB
	pcl::PointXYZ position_OBB;
	//TODO::声明旋转矩阵rotational_matrix_OBB
	Eigen::Matrix3f rotational_matrix_OBB;
	//TODO::声明特征向量major_vector, middle_vector, minor_vector与特征值major_value,   middle_value, minor_value
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	//TODO::声明质心向量mass_center
	Eigen::Vector3f mass_center;

	//TODO::实例化惯性矩特征提取feature_extractor
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud_cluster);//加载点云，原本是从pcd文件
	feature_extractor.compute();

	//TODO::计算OBB包围盒 1.六个极值坐标 2.位置position_OBB 3.旋转矩阵rotational_matrix_OBB 4.特征值特征向量 5.质心mass_center
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter(mass_center);


        //min, max points
        obj_info.min_point_.x = min_point_OBB.x;
        obj_info.min_point_.y = min_point_OBB.y;
        obj_info.min_point_.z = min_point_OBB.z;

        obj_info.max_point_.x = max_point_OBB.x;
        obj_info.max_point_.y = max_point_OBB.y;
        obj_info.max_point_.z = max_point_OBB.z;

        //calculate centroid
        obj_info.centroid_.x = mass_center[0];
        obj_info.centroid_.y = mass_center[1];
        obj_info.centroid_.z = mass_center[2];
 

        //calculate bounding box
        double length_ = obj_info.max_point_.x - obj_info.min_point_.x;
        double width_ = obj_info.max_point_.y - obj_info.min_point_.y;
        double height_ = obj_info.max_point_.z - obj_info.min_point_.z;

        obj_info.bounding_box_.header = point_cloud_header_;

        obj_info.bounding_box_.pose.position.x = obj_info.centroid_.x;
        obj_info.bounding_box_.pose.position.y = obj_info.centroid_.y;
        obj_info.bounding_box_.pose.position.z = obj_info.centroid_.z;

        obj_info.bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);//c=a>b?a:b;//长宽高若为负，则发布其相反数
        obj_info.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
        obj_info.bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

        obj_list.push_back(obj_info);
    }
}


void EuClusterCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    point_cloud_header_ = in_cloud_ptr->header;

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

    voxel_grid_filer(current_pc_ptr, filtered_pc_ptr, LEAF_SIZE);

    //按角度选取点云
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (filtered_pc_ptr);//这个参数得是指针，类对象不行
    pass.setFilterFieldName ("y");//设置想在哪个坐标轴上操作
    pass.setFilterLimits (-10.0, 10.0);//将x轴的0到1范围内
    pass.setFilterLimitsNegative (false);//保留（true就是删除，false就是保留而删除此区间外的）
    pass.filter (*cloud_filtered);//输出到结果指针


    std::vector<Detected_Obj> global_obj_list;
    EuClusterCore::cluster_segment(cloud_filtered, global_obj_list);

    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    for (size_t i = 0; i < global_obj_list.size(); i++)
    {
        bbox_array.boxes.push_back(global_obj_list[i].bounding_box_);
    }
    bbox_array.header = point_cloud_header_;

    pub_bounding_boxs_.publish(bbox_array);

}
