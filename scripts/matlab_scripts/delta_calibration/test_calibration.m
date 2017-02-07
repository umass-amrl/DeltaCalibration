function [t_err, r_err] = genTestCalibrate(N,angle,noise)
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