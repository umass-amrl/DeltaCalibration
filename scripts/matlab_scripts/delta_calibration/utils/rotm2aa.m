function aa = rotm2aa(m)
  aa = quat2aa(rotm2quat(m));
end