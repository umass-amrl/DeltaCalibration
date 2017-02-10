
function trans = calibrate(C0)
clc;
testa = [];
testax = [];
groupa = [];
groupax = [];
filename = '';
debug = 1;
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
deleted = [];
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


if debug
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

%Build M1, M2 matrices
M1 = [];
M2 = [];
partialR = [];
rResidual = [];
for i = 1:N
  q0 = aa2quat(AA0(i, :)');
  q1 = aa2quat(AA1(i, :)');
  M1 = [M1; M1FromQuat(q1)];
  M2 = [M2; M2FromQuat(q0)];
  M = M1 - M2;
[E V] = eig(M'*M);
q = E(:,1);
q = q * sign(q(1));
R = quat2rotm(q);
Raa = quat2aa(q);
partialR = [partialR;Raa];
residual = R * quat2rotm(aa2quat(AA1(i, :)')) - quat2rotm(aa2quat(AA0(i, :)'));
rResidual = [rResidual;abs(norm(residual)) ];
end

% Compute Rotation matrix
M = M1 - M2;
[U, S, V] = svds(M);
S
[E V] = eig(M'*M);
q = E(:,1);
q = q * sign(q(1));
R = quat2rotm(q);
M4 = M1'*M1 + M2'*M2 - M1'*M2 - M2'*M1;


% Fill up alpha and beta form matrices
ATA = zeros(3,3);
ATB = zeros(3,1);
I = eye(3);
tResidual = []
bs = [];
partialT = [];
pairT = [];
for i = 1:N
  q0 = aa2quat(AA0(i, :)');
  a = I - quat2rotm(q0);
  b = T0(i,:)' - R * T1(i,:)';
  bs = [bs; norm(b')];
  ATA = ATA + (a'*a);
  ATB = ATB + a'*b;
  T = inv(ATA) * ATB;
  partialT = [partialT; T'];
  %Calculating translations pairwise
  if i > 1
      pATA = zeros(3,3);
      pATB = zeros(3,1);
      q0_1 = aa2quat(AA0(i - 1, :)');
      q0_2 = aa2quat(AA0(i, :)');
      a_1 = I - quat2rotm(q0_1);
      b_1 = T0(i-1,:)' - R * T1(i-1,:)';
      a_2 = I - quat2rotm(q0_2);
      b_2 = T0(i,:)' - R * T1(i,:)';
      pATA = pATA + (a_1'* a_1);
      pATA = pATA + (a_2' * a_2);
      pATB = pATB + a_1'*b_1;
      pATB = pATB + a_2'*b_2;
      part = inv(pATA) * pATB;
      pairT = [pairT; part']
      
  end
  T1(i,:)';
  TDiff = norm(T0(i,:)'-(R * T1(i,:)') - (a * T));
  tResidual = [tResidual; TDiff'];
  ManualTDiff = norm(T0(i,:)'-(R * T1(i,:)') - (a * T));
end
[U, S, V] = svds(ATA);
S
rcond(ATA)
% Compute T
if rcond(ATA) < 1e-6
  fprintf('WARNING: ATA is ill-defined. eig(ATA):\n');
  [E V] = eig(ATA);
end
inv(ATA);
ATB;
T = inv(ATA) * ATB;

RQuat = rotm2quat(R);
Raa = quat2aa(RQuat);
fprintf('R:\n');
disp(R);
fprintf('\n %f degrees about [%f %f %f]\n\n',...
        180 / pi * 2.0 * acos(q(1)),...
        q(2:4) / norm(q(2:4)));

cTotal = [];
cNorm = [];
partialR;
for i=1:N
  c = cross(AA0(i,:), AA1(i,:));
  cNorm = [cNorm; mean(c)];
  Rt = partialR(i,:);
  Raa;
  RtN = Rt / norm(Rt);
  diffRR = 180/pi *abs(atan2(norm(cross(Raa/norm(Raa),Raa/norm(Raa))),dot(Raa/ norm(Raa),Raa/norm(Raa))));
 
  diffCR = 180/pi * abs(atan2(norm(cross(RtN,Raa/norm(Raa))),dot(RtN,Raa/norm(Raa))));
  diffCR_2 = abs(diffCR - 180);
  
  c = c/norm(c);
  if diffCR_2 < diffCR
    cTotal = [cTotal; c];
  else
    cTotal = [cTotal; c]; 
  end
  %disp(c);
end
trans = [Raa T'];
%axisDiff = bsxfun(@minus,cTotal(2:end, :),Raa/norm(Raa)); 
%axisDiff_Norm = sqrt(sum(abs(axisDiff).^2,2))
axisDiff_Norm = cTotal(2:end, :);    
% file;
% fileID = fopen(strcat(file, '.txt'),'w');
% fprintf(fileID, '%f ', Raa);
% fprintf(fileID, '%f ', T);
% fprintf('T:\n');
% disp(T)


