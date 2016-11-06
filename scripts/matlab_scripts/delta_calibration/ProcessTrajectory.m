function output = ProcessTrajectory(file)
clc;
dataset = file
fprintf('Using dataset %s\n\n', file);
C0 = dlmread(dataset, ' ');
Times = C0(:,1:1);
Trans = C0(:,2:4);
Quats = C0(:, 5:8);
N = size(C0, 1);

% Convert to our angle axis format
angleaxis = [];

base_R = quat2rotm(Quats(1,:)/norm(Quats(1,:)));
for i = 1:N
    q = Quats(i,:)/norm(Quats(i,:));
    q = [q(1), q(2), q(3), q(4)];
  R_prime = quat2aa_wxyz(q)
  fprintf('\n %f degrees about [%f %f %f]\n\n',...
        180 / pi * 2.0 * acos(q(4)),...
        q(1:3) / norm(q(1:3)));
  %%R_prime = rotm2aa(inv(base_R) * aa2rotm(R_prime'))
  norm(R_prime) * 180/pi
  R_prime / norm(R_prime)
  Trans(i,:);
  angleaxis = [angleaxis; R_prime];
end

% Convert to delta-transforms
delta_rots = [];
delta_trans = [];
R_last = [];
T_last = Trans(1,:);
R_last = quat2rotm_xyzw(Quats(1,:)); % Fix for order of quaternions
base_R = E(Quats(1,:));
for i = 2:N
  %R1_i * T1_i * T_2 * R2
  R_prime = quat2rotm_xyzw(Quats(i,:))
  R_last
  R = R_last' * R_prime %R1_i * R2
  T_prime = Trans(i,:);
  %T = R_last' * -T_last' + T_prime'; 
  T = -R_last * R_prime' * T_prime' + T_last';
  %delta_R = quat2aa(rotm2quat(R_prime * R));
  %delta_R = rotm2aa(inv(base_R) * aa2rotm(delta_R'));
  %delta_T = R_prime* (R * T_prime' - T_prime') + R_prime * T';
  delta_R = rotm2aa(R);
  delta_T = T;
  R_last = R_prime;
  T_Last = T_prime;
  delta_rots = [delta_rots; delta_R];
  delta_trans = [delta_trans; delta_T'];
end

delta_transforms = horzcat(delta_rots, delta_trans);
original_transforms = horzcat(angleaxis, Trans);
original_transforms = horzcat(Times, original_transforms);
dlmwrite('slam_delta_transforms.txt', delta_transforms, 'delimiter', ' ', 'precision', 20);
dlmwrite('slam_trajectory.txt', original_transforms, 'delimiter', ' ', 'precision', 20);