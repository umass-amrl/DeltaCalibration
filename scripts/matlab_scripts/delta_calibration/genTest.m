function [t_err, r_err] = genTest(N,angle,noise)
clc;
A = RandomTransform6D(3.14159, .5)';

%N = 10;
max_delta_angle = angle / 180 * pi;
max_delta_translation = 0.2;
noise_angular = (1.0 *noise) / 180 * pi;
noise_translation = 0.01 * noise;

A1 = [];
A1I = [];
A2 = [];
A2I = [];
for i=1:N
  A1 = [A1; RandomTransform6D(max_delta_angle, max_delta_translation)'];
  A1(end,:) = AddNoiseToTransform6D(A1(end,:)', noise_angular, noise_translation)';
  A1I = [A1I; InverseTransform6D(A1(i,:)')];
  A2 = [A2; A1toA2(A', A1(i,:)')];
  A2(end,:) = AddNoiseToTransform6D(A2(end,:)', noise_angular, noise_translation)';
  A2I = [A2I; InverseTransform6D(A2(i,:)')];
end

%% ===========================================
% Test Calibration Math with generated data.

% Build M1, M2 matrices
AA0 = A1(:,1:3);
AA1 = A2(:,1:3);
M1 = [];
M2 = [];
for i = 1:N
  q0 = aa2quat(AA0(i, :)');
  q1 = aa2quat(AA1(i, :)');
  % ==============================================
  % ERROR FOUND: WHY WAS THIS THE OTHER WAY ROUND?
  % ==============================================
  M1 = [M1; M1FromQuat(q1)];
  M2 = [M2; M2FromQuat(q0)];
  M = M1 - M2;
end

% Compute Rotation matrix
M = M1 - M2;
[E V] = eig(M'*M);
q = E(:,1);
q = q * sign(q(1));
A_cal_aa = quat2aa(q);
R = quat2rotm(aa2quat(A_cal_aa'));

% Compute Translation
T0 = A1(:,4:6);
T1 = A2(:,4:6);
ATA = zeros(3,3);
ATB = zeros(3,1);
I = eye(3);
for i = 1:N
  q0 = aa2quat(AA0(i, :)');
  a = I - quat2rotm(q0);
  b = T0(i,:)' - R * T1(i,:)';
  ATA = ATA + (a'*a);
  ATB = ATB + a'*b;
end
% Compute T
if rcond(ATA) < 1e-6
  fprintf('WARNING: ATA is ill-defined. eig(ATA):\n');
  [E V] = eig(ATA);
end
inv(ATA);
T = inv(ATA) * ATB;

A_cal = [A_cal_aa T']
A

% Compute angular error
error_aa = rotm2aa(inv(aa2rotm(A(1:3)')) * aa2rotm(A_cal(1:3)'));
r_err = norm(error_aa) / pi * 180

% Compute translation error
t_err = norm(A(4:6) - A_cal(4:6));
