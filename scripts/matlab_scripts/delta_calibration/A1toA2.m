function A2 = A1toA2(A,A1)
  % A1 is the 6DoF Affine transform for camera 1.
  % A2 is the 6DoF Affine transform for camera 2.
  % A is the 6DoF Affine transform for the extrinsic calibration from camera 2 to camera 1.
  r_aa = A(1:3);
  t = A(4:6);
  r_m = quat2rotm(aa2quat(r_aa));
  
  r1_aa = A1(1:3);
  t1 = A1(4:6);
  r1_m = quat2rotm(aa2quat(r1_aa));

  r2_m = inv(r_m) * r1_m * r_m;
  t2 = inv(r_m) * (r1_m * t + t1 - t);
  r2_aa = quat2aa(rotm2quat(r2_m));
  A2 = [r2_aa t2'];
end