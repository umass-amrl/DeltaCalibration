function [p] = quatRotate(quat,pose)
    t0 = [zeros(size(pose,1), 1) pose(:, 4:6)];
    r0 = [zeros(size(pose,1), 1) pose(:, 1:3)];
    quatIn = [quat(1)';-quat(2:4)'];
    t = quatmultiply(quatmultiply(quat, t0), quatIn');
    r = quatmultiply(quatmultiply(quat, r0), quatIn');
    p = horzcat(r(:, 2:4), t(:, 2:4));

function qout = quatmultiply( q, r )
% Calculate vector portion of quaternion product
% vec = s1*v2 + s2*v1 + cross(v1,v2)
vec = [q(:,1).*r(:,2) q(:,1).*r(:,3) q(:,1).*r(:,4)] + ...
         [r(:,1).*q(:,2) r(:,1).*q(:,3) r(:,1).*q(:,4)]+...
         [ q(:,3).*r(:,4)-q(:,4).*r(:,3) ...
           q(:,4).*r(:,2)-q(:,2).*r(:,4) ...
           q(:,2).*r(:,3)-q(:,3).*r(:,2)];

% Calculate scalar portion of quaternion product
% scalar = s1*s2 - dot(v1,v2)
scalar = q(:,1).*r(:,1) - q(:,2).*r(:,2) - ...
             q(:,3).*r(:,3) - q(:,4).*r(:,4);
    
qout = [scalar  vec];
       
function [r,t] = transformsFromPose(pose)
    rot = pose(1:3);
    t = pose(4:6);
    
    % Form the axis angle representation
    angle = norm(rot);
    axis = rot/angle;
    axAngle = [axis;angle];
    % Convert axis angle to quaternion
    r = axang2quat(axAngle');
    
