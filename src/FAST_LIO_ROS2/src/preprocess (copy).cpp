#include "preprocess.h"
#include <pcl/common/common.h>

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess() : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS = 16;
  SCAN_RATE = 6.37;
  group_size = 8;
  disA = 0.01;
  disA = 0.1;
  p2l_ratio = 225;
  limit_maxmid = 6.25;
  limit_midmin = 6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit / 180 * M_PI);
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);
  cos160 = cos(cos160 / 180 * M_PI);
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr& pcl_out)
{
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, PointCloudXYZI::Ptr& pcl_out)
{
  switch (time_unit)
  {
    case SEC:
      time_unit_scale = 1.e3f;
      break;
    case MS:
      time_unit_scale = 1.f;
      break;
    case US:
      time_unit_scale = 1.e-3f;
      break;
    case NS:
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  switch (lidar_type)
  {
    case OUST64:
      oust64_handler(msg);
      break;

    case VELO16:
      velodyne_handler(msg);
      break;

    case MID360:
      mid360_handler(msg);
      break;

    default:
      default_handler(msg);
      break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  int plsize = msg->point_num;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for (int i = 0; i < N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }

  for (uint i = 0; i < plsize; i++)
  {
    pl_full[i].x = msg->points[i].x;
    pl_full[i].y = msg->points[i].y;
    pl_full[i].z = msg->points[i].z;
    pl_full[i].intensity = msg->points[i].reflectivity;
    pl_full[i].curvature = msg->points[i].offset_time / 1000000.0; // ns -> ms

    if ((abs(pl_full[i].x - 0.0) > 1e-5) ||
        (abs(pl_full[i].y - 0.0) > 1e-5) ||
        (abs(pl_full[i].z - 0.0) > 1e-5))
    {
      if (msg->points[i].line < N_SCANS)
      {
        pl_buff[msg->points[i].line].push_back(pl_full[i]);
      }
    }
  }

  if (feature_enabled)
  {
    for (int j = 0; j < N_SCANS; j++)
    {
      if (pl_buff[j].size() <= 5) continue;

      pcl::PointCloud<PointType>& pl = pl_buff[j];
      int linesize = pl.size();
      vector<orgtype>& types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;

      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    for (int j = 0; j < N_SCANS; j++)
    {
      if (pl_buff[j].size() <= 5) continue;

      pcl::PointCloud<PointType>& pl = pl_buff[j];
      for (auto &pt : pl.points)
      {
        if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > blind * blind)
        {
          pl_surf.push_back(pt);
        }
      }
    }
  }
}

void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  default_handler(msg);
}

void Preprocess::velodyne_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  int plsize = msg->width * msg->height;
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  int x_offset = -1, y_offset = -1, z_offset = -1;
  int intensity_offset = -1, timestamp_offset = -1, time_offset = -1, ring_offset = -1;

  for (const auto& field : msg->fields)
  {
    if (field.name == "x") x_offset = field.offset;
    else if (field.name == "y") y_offset = field.offset;
    else if (field.name == "z") z_offset = field.offset;
    else if (field.name == "intensity") intensity_offset = field.offset;
    else if (field.name == "timestamp") timestamp_offset = field.offset;
    else if (field.name == "time" || field.name == "t") time_offset = field.offset;
    else if (field.name == "ring" || field.name == "layer") ring_offset = field.offset;
  }

  if (x_offset == -1 || y_offset == -1 || z_offset == -1) return;

  for (int i = 0; i < N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }

  for (int i = 0; i < plsize; i++)
  {
    if (i % point_filter_num != 0) continue;

    const uint8_t* point_ptr = &msg->data[i * msg->point_step];
    PointType added_pt;

    memcpy(&added_pt.x, point_ptr + x_offset, sizeof(float));
    memcpy(&added_pt.y, point_ptr + y_offset, sizeof(float));
    memcpy(&added_pt.z, point_ptr + z_offset, sizeof(float));

    double range = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
    if (range < (blind * blind)) continue;

    if (intensity_offset != -1)
    {
      float temp_intensity = 0.0f;
      memcpy(&temp_intensity, point_ptr + intensity_offset, sizeof(float));
      added_pt.intensity = temp_intensity;
    }
    else
    {
      added_pt.intensity = 0.0f;
    }

    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;

    int layer = 0;
    if (ring_offset != -1)
    {
      uint16_t ring_val = 0;
      memcpy(&ring_val, point_ptr + ring_offset, sizeof(uint16_t));
      layer = static_cast<int>(ring_val);

      if (layer >= N_SCANS)
      {
        uint8_t ring_val_8 = 0;
        memcpy(&ring_val_8, point_ptr + ring_offset, sizeof(uint8_t));
        layer = static_cast<int>(ring_val_8);
      }
    }

    if (layer < 0 || layer >= N_SCANS) layer = 0;

    // Hesai absolute timestamp와 header.stamp가 다른 clock domain일 수 있으므로
    // timestamp(double) 절대시간을 직접 빼지 않는다.
    // time 필드가 있으면 상대시간으로 사용하고,
    // 없으면 scan rate 기반 index fallback을 사용한다.
    if (time_offset != -1)
    {
      float rel_time = 0.0f;
      memcpy(&rel_time, point_ptr + time_offset, sizeof(float));
      added_pt.curvature = rel_time * time_unit_scale;
      given_offset_time = true;
    }
    else
    {
      added_pt.curvature = 0.0;
      given_offset_time = false;
    }

    if (!given_offset_time)
    {
      if (plsize > 1)
      {
        added_pt.curvature = (double(i) / double(plsize - 1)) * (1000.0 / double(SCAN_RATE));
      }
      else
      {
        added_pt.curvature = 0.0;
      }
    }

    pl_buff[layer].push_back(added_pt);
  }

  if (feature_enabled)
  {
    for (int j = 0; j < N_SCANS; j++)
    {
      if (pl_buff[j].size() <= 5) continue;

      PointCloudXYZI& pl = pl_buff[j];
      int linesize = pl.size();
      vector<orgtype>& types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;

      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    for (int j = 0; j < N_SCANS; j++)
    {
      if (pl_buff[j].size() <= 5) continue;

      pcl::PointCloud<PointType>& pl = pl_buff[j];
      for (auto &pt : pl.points)
      {
        if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > blind * blind)
        {
          pl_surf.push_back(pt);
        }
      }
    }
  }
}

void Preprocess::mid360_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  default_handler(msg);
}

void Preprocess::default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  int plsize = msg->width * msg->height;
  pl_surf.reserve(plsize);

  int x_off = 0, y_off = 4, z_off = 8;

  for (int i = 0; i < plsize; i++)
  {
    if (i % point_filter_num != 0) continue;

    const uint8_t* ptr = &msg->data[i * msg->point_step];
    PointType added_pt;
    memcpy(&added_pt.x, ptr + x_off, sizeof(float));
    memcpy(&added_pt.y, ptr + y_off, sizeof(float));
    memcpy(&added_pt.z, ptr + z_off, sizeof(float));

    added_pt.intensity = 0;
    added_pt.curvature = 0;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;

    if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind * blind)
      pl_surf.push_back(added_pt);
  }
}

void Preprocess::give_feature(pcl::PointCloud<PointType>& pl, vector<orgtype>& types)
{
  int plsize = pl.size();
  int plsize2;
  if (plsize == 0) return;
  uint head = 0;

  while (head < plsize && types[head].range < blind) head++;

  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0;
  uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  for (uint i = head; i < plsize2; i++)
  {
    if (types[i].range < blind) continue;
    i2 = i;
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if (plane_type == 1)
    {
      for (uint j = i; j <= i_nex; j++)
      {
        if (j != i && j != i_nex) types[j].ftype = Real_Plane;
        else types[j].ftype = Poss_Plane;
      }

      if (last_state == 1 && last_direct.norm() > 0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if (mod > -0.707 && mod < 0.707) types[i].ftype = Edge_Plane;
        else types[i].ftype = Real_Plane;
      }

      i = i_nex - 1;
      last_state = 1;
    }
    else
    {
      i = i_nex;
      last_state = 0;
    }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for (uint i = head + 3; i < plsize2; i++)
  {
    if (types[i].range < blind || types[i].ftype >= Real_Plane) continue;
    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16) continue;

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for (int j = 0; j < 2; j++)
    {
      int m = (j == 1) ? 1 : -1;
      if (types[i + m].range < blind)
      {
        if (types[i].range > inf_bound) types[i].edj[j] = Nr_inf;
        else types[i].edj[j] = Nr_blind;
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z) - vec_a;
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();

      if (types[i].angle[j] < jump_up_limit) types[i].edj[j] = Nr_180;
      else if (types[i].angle[j] > jump_down_limit) types[i].edj[j] = Nr_zero;
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();

    if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero &&
        types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista)
    {
      if (types[i].intersect > cos160 && edge_jump_judge(pl, types, i, Prev))
        types[i].ftype = Edge_Jump;
    }
    else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor &&
             types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista)
    {
      if (types[i].intersect > cos160 && edge_jump_judge(pl, types, i, Next))
        types[i].ftype = Edge_Jump;
    }
    else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
    {
      if (edge_jump_judge(pl, types, i, Prev))
        types[i].ftype = Edge_Jump;
    }
    else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor)
    {
      if (edge_jump_judge(pl, types, i, Next))
        types[i].ftype = Edge_Jump;
    }
    else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
    {
      if (types[i].ftype == Nor) types[i].ftype = Wire;
    }
  }

  plsize2 = plsize - 1;
  for (uint i = head + 1; i < plsize2; i++)
  {
    if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind) continue;
    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8) continue;

    if (types[i].ftype == Nor)
    {
      double ratio = (types[i - 1].dista > types[i].dista)
                       ? types[i - 1].dista / types[i].dista
                       : types[i].dista / types[i - 1].dista;

      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio)
      {
        if (types[i - 1].ftype == Nor) types[i - 1].ftype = Real_Plane;
        if (types[i + 1].ftype == Nor) types[i + 1].ftype = Real_Plane;
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for (uint j = head; j < plsize; j++)
  {
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)
    {
      if (last_surface == -1) last_surface = j;

      if (j == uint(last_surface + point_filter_num - 1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);
        last_surface = -1;
      }
    }
    else
    {
      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane)
        pl_corn.push_back(pl[j]);

      if (last_surface != -1)
      {
        PointType ap;
        ap.x = ap.y = ap.z = 0.0;
        ap.intensity = 0.0;
        ap.curvature = 0.0;

        for (uint k = last_surface; k < j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }

        ap.x /= (j - last_surface);
        ap.y /= (j - last_surface);
        ap.z /= (j - last_surface);
        ap.intensity /= (j - last_surface);
        ap.curvature /= (j - last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI& pl, const rclcpp::Time& ct)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

int Preprocess::plane_judge(const PointCloudXYZI& pl, vector<orgtype>& types,
                            uint i_cur, uint& i_nex, Eigen::Vector3d& curr_direct)
{
  double group_dis = disA * types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
  {
    if (types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }

  for (;;)
  {
    if ((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if (types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }

    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx * vx + vy * vy + vz * vz;

    if (two_dis >= group_dis) break;

    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;
  double v1[3], v2[3];

  for (uint j = i_cur + 1; j < i_nex; j++)
  {
    if ((j >= pl.size()) || (i_cur >= pl.size())) break;

    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];

    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
    if (lw > leng_wid) leng_wid = lw;
  }

  if ((two_dis * two_dis / leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for (uint j = 0; j < disarrsize - 1; j++)
  {
    for (uint k = j + 1; k < disarrsize; k++)
    {
      if (disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if (disarr[disarr.size() - 2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if (lidar_type == AVIA)
  {
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];
    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize - 2];
    if (dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI& pl, vector<orgtype>& types, uint i, Surround nor_dir)
{
  if (nor_dir == 0)
  {
    if (types[i - 1].range < blind || types[i - 2].range < blind) return false;
  }
  else if (nor_dir == 1)
  {
    if (types[i + 1].range < blind || types[i + 2].range < blind) return false;
  }

  double d1 = types[i + nor_dir - 1].dista;
  double d2 = types[i + 3 * nor_dir - 2].dista;
  double d;

  if (d1 < d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  if (d1 > edgea * d2 || (d1 - d2) > edgeb) return false;
  return true;
}