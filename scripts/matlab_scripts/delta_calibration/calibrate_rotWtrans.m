
function Rz = calibrate_rotWtrans(C0)
clc;
testa = [];
testax = [];
groupa = [];
groupax = [];
filename = '';
debug = 2;
tTrue = [.17; .01; .51];
switch 9
  case 1
    dataset = 'boxes1';
  case 2
    dataset = 'boxes2';
  case 3
    dataset = 'corner1';
  case 4
    dataset = 'corner2';
  case 5
    dataset = 'corner3';
  case 6
    dataset = 'corner4';
  case 7
    dataset = 'test';
  case 8
    dataset = 'twisted2';
  case 9
    dataset = 'box_room2';
  case 10
    dataset = 'wide_horz_long';
end
%dataset = strcat(file, '_', int2str(5))
% dataset = file
% fprintf('Using dataset %s\n\n', file);

% C0 = dlmread(strcat(dataset, '_translate.pose'), '\t')
%%C1 = dlmread(strcat('data/', dataset, '_2.pose'));

% Make sure the trajectories have the same number of data points.
N = size(C0, 1);
%N = min(size(C0, 1), size(C1, 1));
%C0 = C0(1:N, :);
%C1 = C1(1:N, :);

% Extract Angle-axis data
AA0 = C0(:,1:3);
AA1 = C0(:,9:11);



% Extract angle magnitude data
A0 = zeros(N,1);
A1 = zeros(N,1);
for i = 1:N
  A0(i) = norm(AA0(i,:));
  A1(i) = norm(AA1(i,:));
end


keys = C0(:, 8:8);
%  Filter the data to ignore inconsistent steps
% deleted = [];
% for i = N:-1:1
%   if abs(A0(i)-A1(i)) > 1/180*pi
%     fprintf('Deleting record %d, delta=%f\n', i, 180/pi*abs(A0-A1));
%     AA0(i,:) = [];
%     AA1(i,:) = [];
%     A0(i) = [];
%     A1(i) = [];
%     C0(i,:) = [];
%     %C1(i,:) = [];
%     N = N - 1;
%     deleted = [deleted; i];
%   end
% end

if debug == 1
  figure

  plot(180/pi*[A0 A1 abs(A0-A1)], 'linewidth', 2);
  axis([1 N 0 180/pi*max([A0; A1])]);
  xlabel('Timestep', 'fontsize', 15);
  ylabel('Rotation (degrees)', 'fontsize', 15);
  [legh, objh, outh, outm] = legend('Angle(R0)', 'Angle(R1)', ...
      '|Angle(R0)-Angle(R1)|');
  set(objh, 'linewidth', 2);
end


% Extract Translations
T0 = C0(:, 4:6);
NT0 = sqrt(sum(abs(T0).^2,2));
T1 = C0(:, 12:14);
NT1 = sqrt(sum(abs(T1).^2,2));


if debug == 2
  figure% deleted = [];
  plot([NT0 NT1 abs(NT0-NT1)], 'linewidth', 2);
  axis([1 N 0 max([NT0; NT1])]);
  xlabel('Timestep', 'fontsize', 15);
  ylabel('Translation (meters)', 'fontsize', 15);
  [legh, objh, outh, outm] = legend('Trans(T0)', 'Trans(T1)', ...
      '|Trans(T0)-Trans(T1)|');
  set(objh, 'linewidth', 2);
end

% Extract Times, velocities, and error metrics
t0 = C0(:, 7:7);
t1 = C0(:, 15:15);
deltat0 = t0(2:end) - t0(1:end-1);
aV0 = abs(A0(2:end)) ./ deltat0;
deltat1 = t1(2:end) - t1(1:end-1);
aV1 = abs(A1(2:end)) ./ deltat1;
tV0 = NT0(2:end) ./ deltat0;
tV1 = NT1(2:end) ./ deltat1;

errorAng = 180/pi*[abs(A0-A1)];
errorAng = errorAng(2:end);

% Calculating rotation using Translation
p1 = zeros(3,3);
p2 = zeros(3,3);
t2 = [];
t1 = [];
pairT = [];
for i = 1:N
  t2 = T1(i,:)
  t1 = T0(i,:)
  a = T1(i,:)' * T1(i,:)
  b = T1(i,:)' * T0(i,:)
%   p1 = p1 + a;
%   p2 = p2 + b;
  t1 = [t1; a];
  t2 = [t2; b];
end
Rt = pinv(t1) * t2
Rt = pinv(Rt)
[thetax,thetay,thetaz] = rotm2eulerangles(Rt)
thetax  * (180 / pi)
thetay * (180 / pi)
thetaz * (180 / pi)
R = Rt;
Rz = thetaz;
for i = 1:N
%   r0 = aa2rotm(AA0(i, :)');
%   r1 = aa2rotm(AA1(i, :)')
%   r3 = RR' * r0 * RR
% % %   r4 = R' * r0 * R
%   t2 = T1(i,:)
%   t1 = T0(i,:);
% % % % %   left = RR*t2' + T
% % % % %   right = r1 * T + t1'
%   left = RR*t2'
%   right = t1'
end