function [t_err, r_err] = genTestPartialCal(N,angle,noise)
clc;
A = RandomTransform6D(3.14159, .5)';
% A = [0 0 1.5 0 0 0 0 0];
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
U1t = [];
U2t = [];
uz = [0 0 1];
ux = [1 0 0];
uy = [0 1 0];
u0 = [0 0 0];
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
  
  a1x_ux = StripRotation(a1x,ux);  
  a1x_uy = StripRotation(a1x,uy);
  a1y_ux = StripRotation(a1y, ux);
  a1y_uy = StripRotation(a1y, uy);
  
  A1 = [A1; a1x_ux];
  A1 = [A1; a1y_ux];
  A1 = [A1; a1x_uy];
  A1 = [A1; a1y_uy];
  
  a1x
  a1x_ux
  a1y
  a1y_ux
  a1x
  a1x_uy
  a1y
  a1y_uy
  
  A2 = [A2; a2x];
  A2 = [A2; a2y];
  A2 = [A2; a2x];
  A2 = [A2; a2y];
  
  U1t = [U1t ; ux];
  U1t = [U1t ; ux];
  U1t = [U1t ; uy];
  U1t = [U1t ; uy];
  
  U2t = [U2t ; u0];
  U2t = [U2t ; u0];
  U2t = [U2t ; u0];
  U2t = [U2t ; u0];
  
  %A2(end,:) = AddNoiseToTransform6D(A2(end,:)', noise_angular, noise_translation)';
end

% ===========================================
% % Test Calibration Math with generated data.
C0 = [A1 A2];
Ut = [U1t U2t];
dlmwrite('generated_deltas.txt', C0, ' ');
dlmwrite('generated_uncertaintiest.txt', Ut, ' ');
size(C0);
A_cal = calibrate_data(C0)
A
q = aa2quat(A');
fprintf('\n %f degrees about [%f %f %f]\n\n',...
        180 / pi * 2.0 * acos(q(1)),... 
        q(2:4) / norm(q(2:4)));
% Compute angular error
error_aa = rotm2aa(inv(aa2rotm(vpa(A(1:3)'))) * aa2rotm(vpa(A_cal(1:3)')))
% [thetax, thetay, thetaz] = rotm2eulerangles(inv(aa2rotm(A(1:3)')))
% [thetaxt, thetayt, thetazt] = rotm2eulerangles(aa2rotm(A_cal(1:3)'))
r_err = norm(error_aa) / pi * 180

% Compute translation error
t_err = norm(A(4:6) - A_cal(4:6))
