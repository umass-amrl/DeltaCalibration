function aa = quat2aa_xyzw(q)
  qw = q(4);
  qx = q(2);
  qy = q(3);
  qz = q(1);
  angle = 2 * acos(qw);
  x = qx / sqrt(1-qw*qw);
  y = qy / sqrt(1-qw*qw);
  z = qz / sqrt(1-qw*qw);
  aa = [angle*x, angle*y, angle*z];
end