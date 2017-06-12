void
  pcl::projectPointOnPlane (
  Eigen::Vector3f const &point,
  Eigen::Vector3f const &plane_point,
  Eigen::Vector3f const &plane_normal,
  Eigen::Vector3f &projected_point)
{
  float t;
  Eigen::Vector3f xo;

  xo = point - plane_point;
  t = plane_normal.dot (xo);

  projected_point = point - (t * plane_normal);
}
