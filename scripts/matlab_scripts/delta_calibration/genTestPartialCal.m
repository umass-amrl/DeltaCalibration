
function [t_err, r_err, t_m_err, r_m_err] = genTestPartialCal(N,angle,noise)
clc;
A = RandomTransform6D(3.14159, .5)';
max_delta_angle = angle / 180 * pi;
max_delta_translation = 0.2;
noise_angular = (1.0 *noise) / 180 * pi;
noise_translation = 0.01 * noise;

A1 = [0 0 0 0 0 0 0 0];
A1I = [];
A2 = [0 0 0 0 0 0 0 0];
A2I = [];
A1T = [];
A2T = [];
U1t = [0 0 0];
U2t = [0 0 0];
U1r = [0 0 0];
U2r = [0 0 0];
uz = [0 0 1];
ux = [1 0 0];
uy = [0 1 0];
u0 = [0 0 0];
V1 = [1000 1000 1000 1000 1000 1000];
V2 = [1000 1000 1000 1000 1000 1000];
T1 = [0 0 0 0 0 0];
T2 = [0 0 0 0 0 0];
empty = [1 1 1 1 1 1];
last_t1 = [0 0 0 0 0 0];
last_t2 = [0 0 0 0 0 0];
time = [0];
count = 0;
var = [noise_angular noise_angular noise_angular noise_translation noise_translation noise_translation];
for i=1:N
  % Canned test. O)ne sensor uncertainty
  % Set 1: x-axis rotation (uncertain x), x-axis rotation (uncertain y),
  % y-axis rotation(uncertain x), y-axis rotation (uncertain y)
  % Set 2: y-axis rotation, y-axis rotation, x-axis rotation, x-axis
  % rotation
  a1x = RandomTransform6D(max_delta_angle, max_delta_translation)';
  a1y = RandomTransform6D(max_delta_angle, max_delta_translation)';
  a1z = RandomTransform6D(max_delta_angle, max_delta_translation)';

  a1x2 = RandomXRotationAA(max_delta_angle)';
  a1y2 = RandomYRotationAA(max_delta_angle)';
  a1z2 = RandomZRotationAA(max_delta_angle)';

  a2x = A1toA2(A', a1x');
  a2y = A1toA2(A', a1y');
  a2z = A1toA2(A', a1z');
  a1x = AddNoiseToTransform6D(a1x', noise_angular, noise_translation)'
  a1y = AddNoiseToTransform6D(a1y', noise_angular, noise_translation)'
  a2x = AddNoiseToTransform6D(a2x', noise_angular, noise_translation)'
  a2y = AddNoiseToTransform6D(a2y', noise_angular, noise_translation)'
  a1x_ux = StripRotation(a1x,uy);
  a1x_uy = StripRotation(a1x,ux);
  a1y_ux = StripRotation(a1y, uy);
  a1y_uy = StripRotation(a1y, ux);
% 
  a1x_ux = StripTranslation(a1x_ux,ux);
  a1x_uy = StripTranslation(a1x_uy,uy);
  a1y_ux = StripTranslation(a1y_ux, ux);
  a1y_uy = StripTranslation(a1y_uy, uy);

%   A1 = [A1; a1x 0 0];
%   A1 = [A1; a1y 0 0];
%   A1 = [A1; a1x 0 0];
%   A1 = [A1; a1y 0 0];
%
  A1 = [A1; a1x_ux];
  A1 = [A1; a1y_ux];
  A1 = [A1; a1x_uy];
  A1 = [A1; a1y_uy];

  A2 = [A2; a2x 0 0];
  A2 = [A2; a2y 0 0];
  A2 = [A2; a2x 0 0];
  A2 = [A2; a2y 0 0];
  tr1 = rotm2aa(aa2rotm(a2x(1:3)') * aa2rotm(last_t1(1:3)'));
  tt1 = a2x(4:6) + last_t1(4:6)
  t1 = [tr1 tt1]
    tr2 = rotm2aa(aa2rotm(a2x(1:3)') * aa2rotm(last_t2(1:3)'));
  tt2 = a2x(4:6) + last_t2(4:6);
  t2 = [tr2 tt2];
  T1 = [T1; t1];
  T2 = [T2; t2];
  U1t = [U1t ; ux];
  U1t = [U1t ; ux];
  U1t = [U1t ; uy];
  U1t = [U1t ; uy];

  U1r = [U1r ; uy];
  U1r = [U1r ; uy];
  U1r = [U1r ; ux];
  U1r = [U1r ; ux];
%   U1t = [U1t ; u0];
%   U1t = [U1t ; u0];
%   U1t = [U1t ; u0];
%   U1t = [U1t ; u0];
%   U1r = [U1r ; u0];
%   U1r = [U1r ; u0];
%   U1r = [U1r ; u0];
%   U1r = [U1r ; u0];
%
  U2t = [U2t ; u0];
  U2t = [U2t ; u0];
  U2t = [U2t ; u0];
  U2t = [U2t ; u0];

  U2r = [U2r ; u0];
  U2r = [U2r ; u0];
  U2r = [U2r ; u0];
  U2r = [U2r ; u0];

  last_t1 = t1;
  last_t2 = t2;
  T1 = [T1; t1];
  T2 = [T2; t2];
  T1 = [T1; t1];
  T2 = [T2; t2];
  T1 = [T1; t1];
  T2 = [T2; t2];
  T1 = [T1; t1];
  T2 = [T2; t2];
  V1 = [V1; var];
  V2 = [V2; var];
  V1 = [V1; var];
  V2 = [V2; var];
  V1 = [V1; var];
  V2 = [V2; var];
  V1 = [V1; var];
  V2 = [V2; var];
  time = [time; count + 1];
  time = [time; count + 1];
  time = [time; count + 1];
  time = [time; count + 1];

  %A2(end,:) = AddNoiseToTransform6D(A2(end,:)', noise_angular, noise_translation)';
end

% ===========================================
% % Test Calibration Math with generated data.
C0 = [A1 A2];
Ut = [U1t U2t];
Ur = [U1r U2r];
deltas1 = A1;
deltas2 = A2;
% kinectData = struct('folder', '', 'files', [], 'time', time, 'type', 'kinect', 'T_Skm1_Sk', A1(:,1:6), 'T_S1_Sk', T1, 'T_Var_Skm1_Sk', V1, 'T_Var_S1_Sk', V2);
% save('GenKinectData.mat', 'kinectData');
% odomData = struct('folder', '', 'files', [], 'time', time, 'type', 'nav', 'T_Skm1_Sk', A2(:,1:6), 'T_S1_Sk', T2, 'T_Var_Skm1_Sk', V2, 'T_Var_S1_Sk', V2);
% save('GenNavData.mat', 'odomData');
dlmwrite('generated_deltas.txt', C0, ' ');
dlmwrite('generated_uncertaintiest.txt', Ut, ' ');
dlmwrite('generated_uncertaintiesr.txt', Ur, ' ');
dlmwrite('T_generated_uncertaintiest.txt', Ut, ' ');
dlmwrite('T_generated_uncertaintiesr.txt', Ur, ' ');
size(C0);
!../../../bin/partial_calibrate
% B_cal = Test3KinectNav();
A_cal = dlmread(strcat('calibration', '.pose'), '\t');
A
% A_multiCal = [B_cal.rot B_cal.tran];
% A_multiCal = A_multiCal(2,:)
q = aa2quat(A');
fprintf('\n %f degrees about [%f %f %f]\n\n',...
        180 / pi * 2.0 * acos(q(1)),...
        q(2:4) / norm(q(2:4)));
error_aa = rotm2aa(inv(aa2rotm(A(1:3)')) * aa2rotm(A_cal(1:3)'));
r_err = norm(error_aa) / pi * 180
% error_aa_multical = rotm2aa(inv(aa2rotm(A(1:3)')) * aa2rotm(A_multiCal(1:3)'));
% r_m_err = norm(error_aa_multical) / pi * 180
% Compute translation error
t_err = norm(A(4:6) - A_cal(4:6))
% t_m_err = norm(A(4:6) - A_multiCal(4:6))
A
A_cal
% A_multiCal
