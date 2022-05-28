/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef ROSPOINTCLOUD2TOPCL_HPP
#define ROSPOINTCLOUD2TOPCL_HPP

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace pcl_df
{

template <typename PointT> void
fromROSMsg (const sensor_msgs::PointCloud2& msg, pcl::PointCloud<PointT>& cloud)
{
  // Copy info fields
  pcl_conversions::toPCL(msg.header, cloud.header);
  cloud.width    = msg.width;
  cloud.height   = msg.height;
  cloud.is_dense = msg.is_dense == 1;

  pcl::MsgFieldMap field_map;
  std::vector<pcl::PCLPointField> msg_fields;
  pcl_conversions::toPCL(msg.fields, msg_fields);
  pcl::createMapping<pcl::PointXYZ> (msg_fields, field_map);

  // Copy point data
  std::uint32_t num_points = msg.width * msg.height;
  cloud.points.resize (num_points);
  std::uint8_t* cloud_data = reinterpret_cast<std::uint8_t*>(&cloud.points[0]);

  // Check if we can copy adjacent points in a single memcpy.  We can do so if there
  // is exactly one field to copy and it is the same size as the source and destination
  // point types.
  if (field_map.size() == 1 &&
      field_map[0].serialized_offset == 0 &&
      field_map[0].struct_offset == 0 &&
      field_map[0].size == msg.point_step &&
      field_map[0].size == sizeof(PointT))
  {
    std::uint32_t cloud_row_step = static_cast<std::uint32_t> (sizeof (PointT) * cloud.width);
    const std::uint8_t* msg_data = &msg.data[0];
    // Should usually be able to copy all rows at once
    if (msg.row_step == cloud_row_step)
    {
      memcpy (cloud_data, msg_data, msg.data.size ());
    }
    else
    {
      for (std::uint32_t i = 0; i < msg.height; ++i, cloud_data += cloud_row_step, msg_data += msg.row_step)
        memcpy (cloud_data, msg_data, cloud_row_step);
    }

  }
  else
  {
    // If not, memcpy each group of contiguous fields separately
    for (std::uint32_t row = 0; row < msg.height; ++row)
    {
      const std::uint8_t* row_data = &msg.data[row * msg.row_step];
      for (std::uint32_t col = 0; col < msg.width; ++col)
      {
        const std::uint8_t* msg_data = row_data + col * msg.point_step;
        for (const pcl::detail::FieldMapping& mapping : field_map)
        {
          memcpy (cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
        }
        cloud_data += sizeof (PointT);
      }
    }
  }
}

} // end namespace

#endif // ROSPOINTCLOUD2TOPCL_HPP
