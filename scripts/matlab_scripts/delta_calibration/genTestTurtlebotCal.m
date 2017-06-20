function [t_err, r_err, t_m_err, r_m_err] = genTestTurtlebotCal(N,angle,noise)
clc;
A = RandomTransform6D(3.14159, .5)';
% A = [0 0 1.5 1 1 0 0 0];
%N = 10;
max_delta_angle = angle / 180 * pi;
max_delta_translation = 0.2;
noise_angular = (1.0 *noise) / 180 * pi;
noise_translation = 0.01 * noise;

A1 = [0 0 0 0 0 0 0 0];
A1I = [0 0 0 0 0 0 0 0];
A2 = [0 0 0 0 0 0 0 0];
A2I = [0 0 0 0 0 0 0 0];
A1T = [0 0 0 0 0 0 0 0];
A2T = [0 0 0 0 0 0 0 0];
var = [noise_angular noise_angular noise_angular noise_translation noise_translation noise_translation];
V1 = [1000 1000 1000 1000 1000 1000; var];
V2 = [1000 1000 1000 1000 1000 1000; var];
T1 = [0 0 0 0 0 0];
T2 = [0 0 0 0 0 0];
U1t = [0 0 0];
U2t = [0 0 0];
U1r = [0 0 0];
U2r = [0 0 0];
small_empty = [0 0 0];
empty = [1 1 1 1 1 1];
variance = [0 0 0 0 0 0];

last_t1 = [0 0 0 0 0 0];
last_t2 = [0 0 0 0 0 0];
time = [0];
count = 0;
for i=1:N
  Ar = RandomZAxisRotate(max_delta_angle, 0)';

  At = RandomGroundTranslation6D(0, max_delta_translation)';
  a2r = A1toA2(A', Ar');
  a2t =  A1toA2(A', At');
  a2r = AddNoiseToTransform6D(a2r', noise_angular, noise_translation)'
  a2t = AddNoiseToTransform6D(a2t', noise_angular, noise_translation)'
  A2 = [A2; a2r 0 0];
  A2T = [A2T; a2t 0 0];
  Ar = AddNoiseToTransform6D(Ar', noise_angular, noise_translation)'
  At = AddNoiseToTransform6D(At', noise_angular, noise_translation)'
  A1 = [A1; Ar 0 0];
  A1T = [A1T; At 0 0];

  V1 = [V1; var];
  V2 = [V2; var];
  V1 = [V1; var];
  V2 = [V2; var];
  time = [time; count + 1];
  U1t = [U1t; small_empty];
  U2t = [U2t; small_empty];
  U1r = [U1r; small_empty];
  U2r = [U2r; small_empty];
  time = [time; count + 1];
  time = [time; count + 1];
end

%% ===========================================
% Test Calibration Math with generated data.
C0 = [A1 A2]
C1 = [A1T A2T];
Temp = [A1; A1T];
Temp2 = [A2; A2T];
Ut = [U1t U2t];
Ur = [U1r U2r];
size(Temp)
size(Temp2)
size(V1)
size(V2)
kinectData = struct('folder', '', 'files', [], 'time', time, 'type', 'kinect', 'T_Skm1_Sk', Temp(:,1:6), 'T_S1_Sk', T1, 'T_Var_Skm1_Sk', V1, 'T_Var_S1_Sk', V2);
save('multi-array-calib/storedTforms/GenKinectData.mat', 'kinectData');
odomData = struct('folder', '', 'files', [], 'time', time, 'type', 'nav', 'T_Skm1_Sk', Temp2(:,1:6), 'T_S1_Sk', T2, 'T_Var_Skm1_Sk', V2, 'T_Var_S1_Sk', V2);
save('multi-array-calib/storedTforms/GenNavData.mat', 'odomData');
dlmwrite('generated_deltas_rot.txt', C0, ' ');
dlmwrite('generated_deltas_trans.txt', C1, ' ');
dlmwrite('generated_uncertaintiest.txt', Ut, ' ');
dlmwrite('generated_uncertaintiesr.txt', Ur, ' ');
dlmwrite('T_generated_uncertaintiest.txt', Ut, ' ');
dlmwrite('T_generated_uncertaintiesr.txt', Ur, ' ');
B_cal = Test3KinectNav();

A_multiCal = [B_cal.rot B_cal.tran];
A_multiCal = A_multiCal(2,:)
!../../../bin/test_turtlebot_calibrate
A_cal = dlmread(strcat('calibration', '.pose'), '\t');


q = aa2quat(A');
error_aa = rotm2aa(inv(aa2rotm(A(1:3)')) * aa2rotm(A_cal(1:3)'));
r_err = norm(error_aa) / pi * 180
t_err = norm(A(4:6) - A_cal(4:6))
t_m_err = norm(A(4:6) - A_multiCal(4:6))
error_aa_multical = rotm2aa(inv(aa2rotm(A(1:3)')) * aa2rotm(A_multiCal(1:3)'));
r_m_err = norm(error_aa_multical) / pi * 180
A_cal
A
A_multiCal
% Compute translation error

