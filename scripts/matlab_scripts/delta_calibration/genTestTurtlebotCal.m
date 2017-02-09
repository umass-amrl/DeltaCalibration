function [t_err, r_err] = genTestTurtlebotCal(N,angle,noise)
clc;
A = RandomTransform6D(3.14159, .5)';
A = [.02 .01 .05 1 1 0 0 0];
%N = 10;
max_delta_angle = angle / 180 * pi;
max_delta_translation = 0.2;
noise_angular = (1.0 *noise) / 180 * pi;
noise_translation = 0.01 * noise;

A1 = [];
A1I = [];
A2 = [];
A2I = [];
A1T = [];
A2T = [];
for i=1:N
  Ar = RandomZAxisRotate(max_delta_angle, 0)';
%   A1 = [A1; RandomZAxisRotate(max_delta_angle, 0)'];
  At = RandomGroundTranslation6D(0, max_delta_translation)';
  A1 = [A1; Ar];
  A1T = [A1T; At];
  %A1(end,:) = AddNoiseToTransform6D(A1(end,:)', noise_angular, noise_translation)';
  A1I = [A1I; InverseTransform6D(A1(i,:)')];
  A2 = [A2; A1toA2(A', Ar')];
  A2T = [A2T; A1toA2(A', At')];
  %A2(end,:) = AddNoiseToTransform6D(A2(end,:)', noise_angular, noise_translation)';
  A2I = [A2I; InverseTransform6D(A2(i,:)')];
end

%% ===========================================
% Test Calibration Math with generated data.
C0 = [A1 A2]
C1 = [A1T A2T];
size(C0);
% A1
% A2
% A1T
% A2T
% A_cal = calibrate_turtlebot(C0, C1)
% A
% q = aa2quat(A');
% fprintf('\n %f degrees about [%f %f %f]\n\n',...
%         180 / pi * 2.0 * acos(q(1)),...
%         q(2:4) / norm(q(2:4)));
% Compute angular error
% error_aa = rotm2aa(inv(aa2rotm(A(1:3)')) * aa2rotm(A_cal(1:3)'))
% [thetax, thetay, thetaz] = rotm2eulerangles(inv(aa2rotm(A(1:3)')))
% [thetaxt, thetayt, thetazt] = rotm2eulerangles(aa2rotm(A_cal(1:3)'))
% r_err = norm(error_aa) / pi * 180
% 
% % Compute translation error
% t_err = norm(A(4:6) - A_cal(4:6))
