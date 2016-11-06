function m = aa2rotm(aa)
  m = quat2rotm(aa2quat(aa));
end