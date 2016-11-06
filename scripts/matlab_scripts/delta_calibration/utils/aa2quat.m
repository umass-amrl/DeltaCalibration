function q = aa2quat(aa)
  angle = norm(aa);
  if angle == 0
    q = [1 0 0 0]';
  else
    axis = aa / angle;
    q = [cos(angle/2); sin(angle/2) * axis];
  end
end