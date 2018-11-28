#pragma once

#include "struct_defines.h"

namespace Bgs
{

inline pcl::PCLHeader HeaderFromBgsToPcl(const Bgs::Header &bgs_header)
{
  pcl::PCLHeader header;
  header.seq = bgs_header.u_seq_num;
  header.frame_id = bgs_header.frame_id;
  header.stamp = bgs_header.stamp.toNSec() / 1000ull;

  return header;
}

inline pcl::PCLHeader HeaderFromBgsShmToPcl(const Bgs::HeaderInShm &bgs_header)
{
  pcl::PCLHeader header;
  header.seq = bgs_header.u_seq_num;
  header.frame_id = std::string(bgs_header.s_frame_id);
  header.stamp = bgs_header.stamp.toNSec() / 1000ull;

  return header;
}

inline Bgs::Header HeaderFromPclToBgs(const pcl::PCLHeader &pcl_header)
{
  Bgs::Header header;
  header.u_seq_num = pcl_header.seq;
  header.frame_id = pcl_header.frame_id;
  header.stamp = BgsTime::fromNSec(pcl_header.stamp * 1000ull);

  return header;
}

inline Bgs::PointField FieldFromPclToBgs(const pcl::PCLPointField &field)
{
  Bgs::PointField bgs_field;

  bgs_field.name = field.name;
  bgs_field.offset = field.offset;
  bgs_field.datatype = field.datatype;
  bgs_field.count = field.count;

  return bgs_field;
}

inline pcl::PCLPointField FieldFromBgsToPcl(const Bgs::PointField &field)
{
  pcl::PCLPointField bgs_field;

  bgs_field.name = field.name;
  bgs_field.offset = field.offset;
  bgs_field.datatype = field.datatype;
  bgs_field.count = field.count;

  return bgs_field;
}
}

// EOF conversions.h