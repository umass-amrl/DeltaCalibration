function aa = quat2aa(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);
  angle = 2 * acos(qw);
  if qw ~= 1
    x = qx / sqrt(1-qw*qw);
    y = qy / sqrt(1-qw*qw);
    z = qz / sqrt(1-qw*qw);
  else
      x = qx;
      y = qy;
      z = qz;
  end
  aa = [angle*x, angle*y, angle*z];
end