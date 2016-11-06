function tf_inv = InverseTransform6D(tf)
  r_aa = tf(1:3);
  t = tf(4:6);
  r_m = quat2rotm(aa2quat(r_aa));

  r_inv_m = inv(r_m);
  r_inv_aa = quat2aa(rotm2quat(r_inv_m));
  t_inv = -r_inv_m * t;
  tf_inv = [r_inv_aa t_inv'];
end