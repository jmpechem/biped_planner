#include "nexus/traversability.h"

GridMap grid_map_data;

traversability::traversability()
{ 
  isfirstcb = true;
  search_radius = 0.1f;
  big_radius = 0.2f;
  reference_height = 0.0f;
  traversability_cmd_sub = t_nh.subscribe<planner_msgs::Mapbuilder>("/map_builder_cmd",100,&traversability::traversability_cmd,this);
  grid_map_sub = t_nh.subscribe("grid_map",100,&traversability::grid_map_cb,this);
  grid_map_data_pub = t_nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  grid_map_plane_seg_pub = t_nh.advertise<sensor_msgs::PointCloud2>("/plane_seg",100);
}

void traversability::grid_map_cb(const grid_map_msgs::GridMap& map_input){
  grid_map_data.clearAll();
  grid_map::GridMapRosConverter::fromMessage(map_input,grid_map_data);
  if(isfirstcb)
  {
      grid_map_data.add("normal_x");
      grid_map_data.add("normal_y");
      grid_map_data.add("normal_z");
      grid_map_data.add("slope");
      grid_map_data.add("rel_height");
      grid_map_data.add("obstacle");
      grid_map_data.add("obstacle_with_pf");

      grid_map_data.add("sub_elevation");
      grid_map_data.add("sub_rel_height");
      grid_map_data.add("sub_normal_x");
      grid_map_data.add("sub_normal_y");
      grid_map_data.add("sub_normal_z");
      grid_map_data.add("edge_line");
      isfirstcb = false;
 }
  normal_extraction(search_radius);
}
void traversability::normal_extraction(float radius)
{
  if(grid_map_data.exists("elevation"))
  {
  Eigen::Vector3d surfaceNormalPositiveAxis_;
  surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitZ();
  vector<string> surfaceNormalTypes;
  surfaceNormalTypes.push_back("normal_x");
  surfaceNormalTypes.push_back("normal_y");
  surfaceNormalTypes.push_back("normal_z");

  for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
  {
    if(!grid_map_data.isValid(*iter,"elevation")) continue;
    if(grid_map_data.isValid(*iter,surfaceNormalTypes)) continue; // normal extraction once...
    Length submapLength = Length::Ones() * (2.0 * radius);
    Position submapPosition;
    grid_map_data.getPosition(*iter,submapPosition);
    Index submapIdx;
    grid_map_data.getIndex(submapPosition,submapIdx);
    Position3 submapPosition3;
    grid_map_data.getPosition3("elevation",submapIdx,submapPosition3);
    const int maxNumberOfCells = pow(ceil(2*radius/grid_map_data.getResolution()),2);
    Eigen::MatrixXd points(3, maxNumberOfCells);
    size_t nPoints = 0;
    for (CircleIterator submapIterator(grid_map_data, submapPosition, radius);!submapIterator.isPastEnd(); ++submapIterator)
    {
        if (!grid_map_data.isValid(*submapIterator, "elevation")) continue;
        Position3 point;
        grid_map_data.getPosition3("elevation", *submapIterator, point);
        points.col(nPoints) = point;
        nPoints++;
    }
    points.conservativeResize(3, nPoints);

    const Position3 mean = points.leftCols(nPoints).rowwise().sum() / nPoints;
       const Eigen::MatrixXd NN = points.leftCols(nPoints).colwise() - mean;

       const Eigen::Matrix3d covarianceMatrix(NN * NN.transpose());
       Vector3 eigenvalues = Vector3::Ones();
       Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
       // Ensure that the matrix is suited for eigenvalues calculation
       if (covarianceMatrix.fullPivHouseholderQr().rank() >= 3) {
         const Eigen::EigenSolver<Eigen::MatrixXd> solver(covarianceMatrix);
         eigenvalues = solver.eigenvalues().real();
         eigenvectors = solver.eigenvectors().real();
       } else {
         ROS_DEBUG("Covariance matrix needed for eigen decomposition is degenerated. Expected cause: no noise in data (nPoints = %i)", (int) nPoints);

         eigenvalues.z() = 0.0;
       }
       // Keep the smallest eigenvector as surface normal
       int smallestId(0);
       double smallestValue(std::numeric_limits<double>::max());
       for (int j = 0; j < eigenvectors.cols(); j++) {
         if (eigenvalues(j) < smallestValue) {
           smallestId = j;
           smallestValue = eigenvalues(j);
         }
       }
       Vector3 eigenvector = eigenvectors.col(smallestId);
       if (eigenvector.dot(surfaceNormalPositiveAxis_) < 0.0) eigenvector = -eigenvector;
       grid_map_data.at("normal_x", *iter) = eigenvector.x();
       grid_map_data.at("normal_y", *iter) = eigenvector.y();
       grid_map_data.at("normal_z", *iter) = eigenvector.z();


   }    

  }
  else
  {
      ROS_INFO("Fail:No grid map elevation data!");
  }
}

void traversability::traversability_cmd(const planner_msgs::Mapbuilder::ConstPtr &cmd)
{
  if(cmd->state == "find_obstacle")
  {
    ROS_INFO("%s",cmd->state.c_str());
    obstacle_finder(cmd->val1,cmd->val2);
    reference_height = cmd->val2;
  }
  else if(cmd->state == "obstacle_potential_field")
  {
    ROS_INFO("%s",cmd->state.c_str());
    obstacle_potential_field(cmd->val1,cmd->val2);
  }
  else if(cmd->state == "step_detect")
  {
    ROS_INFO("%s",cmd->state.c_str());
    step_detection();
  }
}

void traversability::obstacle_finder(float slope, float rel_height)
{
    if(grid_map_data.exists("elevation") && grid_map_data.exists("normal_z"))
      {
    float angle = 0.0f;
    double cell_z = 0.0f;
    float ref_height = -1.09;
    float ref_angle = 45;
    for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
    {
        angle = rad2deg(acos(grid_map_data.at("normal_z",*iter)));
        double height;
        height = abs(ref_height-grid_map_data.at("elevation",*iter));
        double rel_slope = angle-ref_angle;
        grid_map_data.at("rel_height",*iter) = height;
        grid_map_data.at("slope",*iter) = rel_slope;

        if(rel_slope >= slope || height >= rel_height)
        {
            grid_map_data.at("obstacle",*iter) = 1;
        }
        else
        {
                grid_map_data.at("obstacle",*iter) = 0;
        }
/*
        Position3 center_point;
        grid_map_data.getPosition3("elevation",*iter,center_point);

        Position submapPosition;
        grid_map_data.getPosition(*iter,submapPosition);

        size_t nPoints = 0;

        for (CircleIterator submapIterator(grid_map_data, submapPosition, search_radius);!submapIterator.isPastEnd(); ++submapIterator)
        {
            if (!grid_map_data.isValid(*submapIterator, "elevation")) continue;
            Position3 point;
            grid_map_data.getPosition3("elevation", *submapIterator, point);
            cell_z += point(2);
            nPoints++;
        }
        double mean_z = cell_z / nPoints;
*/

        //grid_map_data.at("rel_height",*iter) = center_point(2) - mean_z;

    }
    ROS_INFO("obstacle founded!");
    ros::Time time = ros::Time::now();
    grid_map_data.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(grid_map_data, message);
    grid_map_data_pub.publish(message);
    }
    else
    {
        ROS_INFO("Fail:There is no grid data(obstacle cannot be found)!");
    }
}
float traversability::plane_dist_cal(float center_x, float center_y, float target_x, float target_y)
{
  return sqrt((center_x-target_x)*(center_x-target_x) + (center_y-target_y) * (center_y-target_y));
}

void traversability::obstacle_potential_field(float pf_radius,float grid_resolution)
{

  for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
  {
      grid_map_data.at("obstacle_with_pf",*iter) = 0;
  }


  for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
  {
      if(grid_map_data.at("obstacle",*iter) == 1)
      {
          grid_map_data.at("obstacle_with_pf",*iter) = grid_map_data.at("obstacle",*iter);
          Position submap_center_Position;
          grid_map_data.getPosition(*iter,submap_center_Position);

          for (CircleIterator submapIterator(grid_map_data, submap_center_Position, pf_radius);!submapIterator.isPastEnd(); ++submapIterator)
          {
              if (!grid_map_data.isValid(*submapIterator, "elevation")) continue;
              Position submap_sur_position;
              grid_map_data.getPosition(*submapIterator,submap_sur_position);
              float dist_from_center = plane_dist_cal(submap_center_Position(0),submap_center_Position(1),submap_sur_position(0),submap_sur_position(1));
              if(dist_from_center < grid_resolution)
              {
                  grid_map_data.at("obstacle_with_pf",*submapIterator) = 1;

              }
              else
              {
                  if(grid_map_data.at("obstacle",*submapIterator) != 1)
                  grid_map_data.at("obstacle_with_pf",*submapIterator) = 0.5;
              }

          }
      }

  }

  for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
  {
      if(grid_map_data.at("obstacle_with_pf",*iter) == 0)
      {
       grid_map_data.at("sub_elevation",*iter) = grid_map_data.at("elevation",*iter);
      }
  }


  ROS_INFO("obstacle potential field generated!");
  ros::Time time = ros::Time::now();
  grid_map_data.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(grid_map_data, message);
  grid_map_data_pub.publish(message);

  pcl::PointCloud<pcl::PointXYZ> seg_cloud;
  seg_cloud.clear();



  for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
  {
      if(grid_map_data.at("obstacle_with_pf",*iter) == 0)
      {          
          Position pos;
          grid_map_data.getPosition(*iter,pos);
          Index idx;
          grid_map_data.getIndex(pos,idx);
          Position3 pos3;
          grid_map_data.getPosition3("elevation",idx,pos3);
          pcl::PointXYZ   pt;
          pt.x = pos3(0);
          pt.y = pos3(1);
          pt.z = pos3(2);
          if(pt.x != 0) // I don't know why x=0 position grid generated....
          seg_cloud.push_back(pt);
      }
  }
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(seg_cloud, cloud_p);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "base_link";
  grid_map_plane_seg_pub.publish(output);

}
void traversability::step_detection()
{




/*
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
   pcl::PointCloud<pcl::PointXYZI> TotalCloud;
  for(int i = 0; i < 8; i++)
  {
       seg.setInputCloud(seg_cloud.makeShared ());
       pcl::ModelCoefficients coefficients;
       pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
       seg.segment (*inliers, coefficients);
       pcl::ExtractIndices<pcl::PointXYZ> extract;
       ROS_INFO("extract the inliers");
       pcl::PointCloud<pcl::PointXYZ> in_cloud;
       extract.setInputCloud (seg_cloud.makeShared ());
       extract.setIndices (inliers);
       extract.setNegative (false);
       extract.filter (in_cloud);
       extract.setNegative (true);
       extract.filter (seg_cloud);

       pcl::PointCloud<pcl::PointXYZ>::iterator index = in_cloud.begin();
           for(; index != in_cloud.end(); index++)
           {
              pcl::PointXYZ pt = *index;
              pcl::PointXYZI pt2;
              pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
              pt2.intensity = (float)(i + 1);

              TotalCloud.push_back(pt2);
           }
             ROS_INFO("%d. remained point cloud = %d", i, (int)seg_cloud.size());
  }
   sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(TotalCloud, cloud_p);
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "base_link";
    grid_map_plane_seg_pub.publish(output);
    */
/*
  for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
  {
       if(grid_map_data.at("obstacle_with_pf",*iter) == 0)
       grid_map_data.at("sub_elevation",*iter) = grid_map_data.at("elevation",*iter);
  }

  double srh_min = 0.0f;
  double srh_max = 0.0f;
  bool run_first = true;
  for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
  {
      double height;
      height = reference_height-grid_map_data.at("sub_elevation",*iter);
      if(std::isnan(height))
      {
      }
      else
      {
      grid_map_data.at("sub_rel_height",*iter) = height;
        if(run_first)
        {
         srh_max = height;
         srh_min = height;
         run_first = false;
        }
        else
        {
          if(height >= srh_max)
          {
              srh_max = height;
          }
          if(srh_min >= height )
          {
              srh_min = height;
          }
        }
      }
      grid_map_data.at("edge_line",*iter) = 0;
  }
  cout << srh_min << "  " << srh_max << endl;


  vector<Index> step_index;
  vector<Position3> step_pos3;


  Eigen::Vector3d surfaceNormalPositiveAxis_;
  surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitZ();
  vector<string> surfaceNormalTypes;
  surfaceNormalTypes.push_back("sub_normal_x");
  surfaceNormalTypes.push_back("sub_normal_y");
  surfaceNormalTypes.push_back("sub_normal_z");
  int grid_number = grid_map_data.getSize()(0) * grid_map_data.getSize()(1);
  double grid_roughness[grid_number];

  for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
  {
    if(!grid_map_data.isValid(*iter,"sub_elevation")) continue;
    if(grid_map_data.isValid(*iter,surfaceNormalTypes)) continue;
    Length submapLength = Length::Ones() * (2.0 * big_radius);
    Position submapPosition;
    grid_map_data.getPosition(*iter,submapPosition);
    Index submapIdx;
    grid_map_data.getIndex(submapPosition,submapIdx);

    const int maxNumberOfCells = pow(ceil(2*big_radius/grid_map_data.getResolution()),2);
    Eigen::MatrixXd points(3, maxNumberOfCells);
    size_t nPoints = 0;
    for (CircleIterator submapIterator(grid_map_data, submapPosition, big_radius);!submapIterator.isPastEnd(); ++submapIterator)
    {
        if (!grid_map_data.isValid(*submapIterator, "sub_elevation")) continue;
        Position3 point;
        grid_map_data.getPosition3("sub_elevation", *submapIterator, point);
        points.col(nPoints) = point;
        nPoints++;
    }
    points.conservativeResize(3, nPoints);

    const Position3 mean = points.leftCols(nPoints).rowwise().sum() / nPoints;
       const Eigen::MatrixXd NN = points.leftCols(nPoints).colwise() - mean;

       const Eigen::Matrix3d covarianceMatrix(NN * NN.transpose());
       Vector3 eigenvalues = Vector3::Ones();
       Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
       if (covarianceMatrix.fullPivHouseholderQr().rank() >= 3) {
         const Eigen::EigenSolver<Eigen::MatrixXd> solver(covarianceMatrix);
         eigenvalues = solver.eigenvalues().real();
         eigenvectors = solver.eigenvectors().real();
       } else {
         ROS_DEBUG("Covariance matrix needed for eigen decomposition is degenerated. Expected cause: no noise in data (nPoints = %i)", (int) nPoints);

         eigenvalues.z() = 0.0;
       }
       int smallestId(0);
       double smallestValue(std::numeric_limits<double>::max());
       for (int j = 0; j < eigenvectors.cols(); j++) {
         if (eigenvalues(j) < smallestValue) {
           smallestId = j;
           smallestValue = eigenvalues(j);
         }
       }
       Vector3 eigenvector = eigenvectors.col(smallestId);
       if (eigenvector.dot(surfaceNormalPositiveAxis_) < 0.0) eigenvector = -eigenvector;
       grid_map_data.at("sub_normal_x", *iter) = eigenvector.x();
       grid_map_data.at("sub_normal_y", *iter) = eigenvector.y();
       grid_map_data.at("sub_normal_z", *iter) = eigenvector.z();

       Position3 submapPosition3;
       grid_map_data.getPosition3("sub_elevation",submapIdx,submapPosition3);
       double small_radius = big_radius/2.0;
       Eigen::Vector3d plane_mean = points.leftCols(nPoints).rowwise().sum() / nPoints;
       double normalX = eigenvector.x();
       double normalY = eigenvector.y();
       double normalZ = eigenvector.z();
       double planeParameter = plane_mean.x()*normalX + plane_mean.y()*normalY + plane_mean.z()*normalZ;
       size_t sub_nPoints = 0;
       Eigen::MatrixXd sub_points(3, maxNumberOfCells);
       for (CircleIterator submapIterator(grid_map_data, submapPosition, small_radius);!submapIterator.isPastEnd(); ++submapIterator)
       {
           if (!grid_map_data.isValid(*submapIterator, "sub_elevation")) continue;
           Position3 point;
           grid_map_data.getPosition3("sub_elevation", *submapIterator, point);
           sub_points.col(sub_nPoints) = point;
           sub_nPoints++;
       }
       double sum = 0.0;
       for (int i = 0; i < sub_nPoints; i++) {
         double dist = normalX*sub_points(0,i) + normalY*sub_points(1,i) + normalZ*sub_points(2,i) - planeParameter;
         sum += pow(dist,2);
       }
       double var = sqrt(sum / (sub_nPoints -1));
       double edge = sqrt(var);
       if(edge > 0.10) // threshold of roughness should be researched!!!!!!!!!!!!!!!!!
       {
           Position pos;
           grid_map_data.getPosition(*iter,pos);
           Index idx;
           grid_map_data.getIndex(pos,idx);
           step_index.push_back(idx);
           Position3 pos3;
           grid_map_data.getPosition3("sub_rel_height",idx,pos3);
           step_pos3.push_back(pos3);
       }
   }

  vector<Index> step_edge;

  for(size_t i = 0 ; i<step_index.size();i++)
  {
      Position3 ref_pos3 = step_pos3[i];
      for(size_t j=0; j<step_pos3.size();j++)
      {
          Position3 compare_pos3 = step_pos3[j];
          if(  abs(ref_pos3(2) - compare_pos3(2)) > 0.02 && plane_dist_cal(ref_pos3(0),ref_pos3(1),compare_pos3(0),compare_pos3(1)) <= 0.05*sqrt(2) )
          {
            if(ref_pos3(2) > compare_pos3(2))
               step_edge.push_back(step_index[j]);
          }
      }
  }
  for(size_t i=0;i<step_edge.size();i++)
  {
      grid_map_data.at("edge_line",step_edge[i]) = 1;
  }

  cv::Mat subMat;
  GridMapCvConverter::toImage<unsigned char, 1>(grid_map_data, "sub_rel_height", CV_8UC1, srh_max, srh_min, subMat);
  cout << subMat << endl;
  namedWindow("height input", CV_WINDOW_AUTOSIZE);
  imshow("height input",subMat);
  cv::waitKey(10);


  step_index.clear();
  step_pos3.clear();
  ROS_INFO("edge generated!");
  */

  //ros::Time time = ros::Time::now();
  //grid_map_data.setTimestamp(time.toNSec());
  //grid_map_msgs::GridMap message;
  //GridMapRosConverter::toMessage(grid_map_data, message);
  //grid_map_data_pub.publish(message);
}
