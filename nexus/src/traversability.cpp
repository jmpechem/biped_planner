#include "nexus/traversability.h"
namespace jm_traversability{
GridMap grid_map_data;
pcl::PointCloud<pcl::PointXYZI>::Ptr seg_clouds (new pcl::PointCloud<pcl::PointXYZI>);

traversability::traversability()
{ 
  isfirstcb = true;
  search_radius = 0.1f;
  big_radius = 0.25f;
  reference_height = 0.0f;
  traversability_cmd_sub = t_nh.subscribe<planner_msgs::Mapbuilder>("/map_builder_cmd",100,&traversability::traversability_cmd,this);
  grid_map_sub = t_nh.subscribe("grid_map",100,&traversability::grid_map_cb,this);
  grid_map_data_pub = t_nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  segmented_grid_data_sub = t_nh.subscribe<sensor_msgs::PointCloud2>("/plane_seg_result",100,&traversability::segmented_grid_map_cb,this);
  segment_sub = t_nh.subscribe<planner_msgs::Mapbuilder>("/segment_info_recv",100,&traversability::plane_num_cb, this);
  seg_clouds->clear();
  total_segmented_number = 0;
  obstacle_planes.clear();
  not_obs_planes.clear();
  large_normal_vector.clear();
  small_normal_vector.clear();
  //grid_map_plane_seg_pub = t_nh.advertise<sensor_msgs::PointCloud2>("/plane_seg",100);
}
void traversability::segmented_grid_map_cb(const sensor_msgs::PointCloud2::ConstPtr &clouds)
{
  seg_clouds->clear();
  for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
  {
      if(grid_map_data.isValid(*iter,"elevation"))
      grid_map_data.at("segment",*iter) = 0; // -1
  }

  pcl::fromROSMsg(*clouds,*seg_clouds);
  for(size_t i=0;i<seg_clouds->size();++i)
  {
      auto& point = seg_clouds->points[i];
      Index index;
      Position position(point.x,point.y);
      if(!grid_map_data.getIndex(position, index)) continue;
      if(grid_map_data.isValid(index,"elevation")){grid_map_data.at("segment",index) = seg_clouds->points[i].intensity;}
  }

  ros::Time time = ros::Time::now();
  grid_map_data.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(grid_map_data, message);
  grid_map_data_pub.publish(message);

}

void traversability::grid_map_cb(const grid_map_msgs::GridMap& map_input){
  grid_map_data.clearAll();
  grid_map::GridMapRosConverter::fromMessage(map_input,grid_map_data);
  if(isfirstcb)
  {
      //grid_map_data.add("normal_x");
      //grid_map_data.add("normal_y");
      //grid_map_data.add("normal_z");
      grid_map_data.add("slope");
      grid_map_data.add("rel_height");
      grid_map_data.add("obstacle");
      grid_map_data.add("obstacle_with_pf");

      grid_map_data.add("sub_elevation");
      grid_map_data.add("sub_rel_height");
      grid_map_data.add("sub_normal_x");
      grid_map_data.add("sub_normal_y");
      grid_map_data.add("sub_normal_z");
      grid_map_data.add("diff_normal_x");
      grid_map_data.add("diff_normal_y");
      grid_map_data.add("diff_normal_z");
      grid_map_data.add("diff_norms");
      grid_map_data.add("segment");
      isfirstcb = false;
 }
  /*large_normal_vector.clear();
  small_normal_vector.clear();
  large_normal_vector.push_back("normal_x");
  large_normal_vector.push_back("normal_y");
  large_normal_vector.push_back("normal_z");
  normal_extraction(big_radius,large_normal_vector);
  small_normal_vector.push_back("sub_normal_x");
  small_normal_vector.push_back("sub_normal_y");
  small_normal_vector.push_back("sub_normal_z");
  normal_extraction(search_radius,small_normal_vector);

  diff_normal(large_normal_vector,small_normal_vector);*/
}
void traversability::diff_normal(vector<string> large_name,vector<string> small_name)
{
  double min_diff;
  double max_diff;
  int i=0;
  for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
  {
      if(!grid_map_data.isValid(*iter,"elevation")) continue;
      if(!grid_map_data.isValid(*iter,large_name.at(0))) continue;
      if(!grid_map_data.isValid(*iter,large_name.at(1))) continue;
      if(!grid_map_data.isValid(*iter,large_name.at(2))) continue;
      if(!grid_map_data.isValid(*iter,small_name.at(0))) continue;
      if(!grid_map_data.isValid(*iter,small_name.at(1))) continue;
      if(!grid_map_data.isValid(*iter,small_name.at(2))) continue;
      double dx = grid_map_data.at("diff_normal_x", *iter) = (grid_map_data.at("sub_normal_x", *iter) - grid_map_data.at("normal_x", *iter))/2.0;
      double dy = grid_map_data.at("diff_normal_y", *iter) = (grid_map_data.at("sub_normal_y", *iter) - grid_map_data.at("normal_y", *iter))/2.0;
      double dz = grid_map_data.at("diff_normal_z", *iter) = (grid_map_data.at("sub_normal_z", *iter) - grid_map_data.at("normal_z", *iter))/2.0;
      grid_map_data.at("diff_norms", *iter) = sqrt(dx*dx+dy*dy+dz*dz);
      if(i==0){ min_diff = grid_map_data.at("diff_norms", *iter); max_diff = grid_map_data.at("diff_norms", *iter);}
      if(min_diff >= grid_map_data.at("diff_norms", *iter)) {min_diff = grid_map_data.at("diff_norms", *iter);}
      if(max_diff <= grid_map_data.at("diff_norms", *iter)) {max_diff = grid_map_data.at("diff_norms", *iter);}
      i++;
  }
  cout << "min max diff : " << min_diff << ", " << max_diff << endl;
}

void traversability::normal_extraction(float radius,vector<string> normal_name_vector)
{
  if(grid_map_data.exists("elevation"))
  {
  Eigen::Vector3d surfaceNormalPositiveAxis_;
  surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitZ();
  vector<string> surfaceNormalTypes = normal_name_vector;
  //surfaceNormalTypes.push_back("normal_x");
  //surfaceNormalTypes.push_back("normal_y");
  //surfaceNormalTypes.push_back("normal_z");

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
       grid_map_data.at(surfaceNormalTypes.at(0), *iter) = eigenvector.x();
       grid_map_data.at(surfaceNormalTypes.at(1), *iter) = eigenvector.y();
       grid_map_data.at(surfaceNormalTypes.at(2), *iter) = eigenvector.z();


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
  }
  else if(cmd->state == "obstacle")
  {
      bool isthere = false;
      for(size_t i=0;i<obstacle_planes.size();i++)
        {
          if(obstacle_planes.at(i) == cmd->id)
            {
              isthere = true;
            }
        }
      if(!isthere){obstacle_planes.push_back(cmd->id);}

      for(size_t i=0; i<not_obs_planes.size();i++)
      {
          if(not_obs_planes.at(i) == cmd->id)
          {
              not_obs_planes.erase(not_obs_planes.begin() + i);
          }
      }
  }
  else if(cmd->state == "traversable")
  {
      bool isthere = false;
      for(size_t i=0; i<not_obs_planes.size();i++)
      {
              if(not_obs_planes.at(i) == cmd->id)
              {
                  isthere = true;
              }
      }
      if(!isthere){not_obs_planes.push_back(cmd->id);}

      for(size_t i=0; i<obstacle_planes.size();i++)
      {
          if(obstacle_planes.at(i) == cmd->id)
          {
              obstacle_planes.erase(obstacle_planes.begin() + i);
          }
      }

  }
}
void traversability::plane_num_cb(const planner_msgs::Mapbuilder::ConstPtr& num)
{
  if(num->state == "seg_plane")
  {
     total_segmented_number = num->id;
     for(size_t i=0; i<=total_segmented_number;i++)
     {         
         not_obs_planes.push_back(i);        
     }
  }
}

void traversability::obstacle_finder(float slope, float rel_height)
{  

    if(grid_map_data.exists("elevation") && grid_map_data.exists("normal_z"))
    {
        for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
        {

            grid_map_data.at("obstacle",*iter) = 0;
        }

        for(size_t i=0; i<obstacle_planes.size();i++)
        {
            for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
            {
                if(grid_map_data.at("segment",*iter) == obstacle_planes.at(i))
                {
                    grid_map_data.at("obstacle",*iter) = 1;
                }
            }
        }

    /*
    float angle = 0.0f;
    double cell_z = 0.0f;
    float ref_height = -1.09;
    //float ref_angle = 45;
    int seg_min_num = 0;
    int seg_max_num = 0;
    int seg_num = 0;
    for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
    {
        angle = (acos(grid_map_data.at("normal_z",*iter)));
        double height;
        height = abs(ref_height-grid_map_data.at("elevation",*iter));
        //double rel_slope = angle-ref_angle;
        grid_map_data.at("rel_height",*iter) = height;
        grid_map_data.at("slope",*iter) = angle;//grid_map_data.at("slope",*iter) = rel_slope;
        if(grid_map_data.isValid(*iter,"segment"))
        {

           seg_num = grid_map_data.at("segment",*iter);
           if(seg_num > seg_max_num && seg_num != -1){ seg_max_num = seg_num; }
           if(seg_num < seg_min_num && seg_num != -1 ){ seg_min_num = seg_num; }
        }
    }


    int seg_cnt = seg_max_num - seg_min_num + 1;
    int out_sloped = 0;
    float seg_height_sum[seg_cnt];
    float seg_height_mean[seg_cnt];
    for(size_t i =0; i<seg_cnt;i++)
    {
        seg_height_sum[i] = 0.0f;
        seg_height_mean[i] = 0.0f;
    }
    int seg_index=0;
    int seg_plane_cnt = 0;
    float each_seg[seg_cnt];
    vector<float> seg_height_mean_vec;
    seg_height_mean_vec.clear();
    for(size_t i=0; i< seg_cnt ; i++)
    {
        seg_plane_cnt = 0;
        out_sloped = 0;
        for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
        {
          if(grid_map_data.isValid(*iter,"segment"))
          {
              if(grid_map_data.at("segment",*iter) == seg_index)
              {
                  seg_height_sum[seg_index] += grid_map_data.at("rel_height",*iter);
                  if(grid_map_data.at("slope",*iter) > deg2rad(slope))
                  {
                      out_sloped++;
                  }
                  seg_plane_cnt++;
              }

          }

        }
        seg_height_mean[seg_index] = seg_height_sum[seg_index] / ((float)seg_plane_cnt);
        cout << "seg " << seg_index << "th mean h : "<< seg_height_mean[seg_index] << endl;
        if(seg_index != 0 ) {seg_height_mean_vec.push_back(seg_height_mean[seg_index]);}
        each_seg[seg_index] = ((float)out_sloped)/((float)seg_plane_cnt);
        seg_index++;
    }

    int max_height_index = 0;
    int min_height_index = 0;
    float max_h = 0.0f;
    float min_h = 0.0f;
    for(size_t i=0; i< seg_height_mean_vec.size(); i++)
    {
          if(i==0)
          {
            max_h = seg_height_mean_vec.at(i);
            min_h = seg_height_mean_vec.at(i);
          }
          if(seg_height_mean_vec.at(i) > max_h){ max_h = seg_height_mean_vec.at(i); max_height_index = i;}
          if(seg_height_mean_vec.at(i) < min_h){ min_h = seg_height_mean_vec.at(i); min_height_index = i;}
    }

    // not segmented area is obstacle area
    for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
    {
        if(grid_map_data.at("segment",*iter) == -1 || grid_map_data.at("segment",*iter) == 0)
        grid_map_data.at("obstacle",*iter) = 1;
    }

    vector<int> obstacle_plane_idx;
    obstacle_plane_idx.clear();
    // over slope segmented plane is obstacle area
    for(size_t i = 0; i< seg_cnt; i++)
    {
      if(each_seg[i] >= 0.95)
      {

          obstacle_plane_idx.push_back(i);
      }
    }

    cout << "max min plane idx : " << max_height_index << ", " << min_height_index << endl;
    cout << "h max min : " << seg_height_mean_vec.at(max_height_index) << ", " << seg_height_mean_vec.at(min_height_index) << endl;

    float h_max_p = seg_height_mean_vec.at(max_height_index)+rel_height;
    float h_max_m = seg_height_mean_vec.at(max_height_index)-rel_height;
    float h_min_p = seg_height_mean_vec.at(min_height_index)+rel_height;
    float h_min_m = seg_height_mean_vec.at(min_height_index)-rel_height;
    for(size_t i =0; i< seg_height_mean_vec.size(); i++)
    {
      cout << "h val : " << seg_height_mean_vec.at(i) << endl;
      float h_tmp = seg_height_mean_vec.at(i);
      if( h_max_p < h_tmp && h_max_m > h_tmp && h_min_p < h_tmp && h_min_m > h_tmp)
      {
          obstacle_plane_idx.push_back(i);
      }
    }


    for(size_t i=0;i<obstacle_plane_idx.size();i++)
    {
        for(GridMapIterator iter(grid_map_data);!iter.isPastEnd();++iter)
        {
               if(grid_map_data.at("segment",*iter) == obstacle_plane_idx.at(i))
               {
                  grid_map_data.at("obstacle",*iter) = 1;
               }
        }
        cout << "obstacle idx : " << obstacle_plane_idx.at(i) << endl;
    }
    */


    ROS_INFO("obstacle founded!");
    ros::Time time = ros::Time::now();
    grid_map_data.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(grid_map_data, message);
    grid_map_data_pub.publish(message);
    obstacle_planes.clear();
    not_obs_planes.clear();
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

}
}
