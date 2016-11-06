function [r,t] = transformsFromPoses(poses)
  r = [];
  t = [];
  for i = poses'
    pose = i;
    rot = pose(1:3);
    ti = pose(4:6)';
    % Form the axis angle representation
    angle = norm(rot);
    axis = rot/angle;
    axAngle = [axis;angle];
    % Convert axis angle to quaternion
    ri = axAngle2Quat(axAngle');
    r = [r ; ri];
    t = [t ; ti'];
  end

function q = axAngle2Quat(axAngle)
  angle = axAngle(4);
  qx = axAngle(1) * sin(angle/2);
  qy = axAngle(2) * sin(angle/2);
  qz = axAngle(3) * sin(angle/2);
  qw = cos(angle/2);
  q = -[qw, qx, qy, qz];