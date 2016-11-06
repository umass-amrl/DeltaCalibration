dataset = 'overlap_p1';
dataset1 = dataset;

dataset = strcat(dataset, '_', int2str(5))
fprintf('Using dataset %s\n\n', dataset);

dataset1 = strcat(dataset1, '_', int2str(5))
fprintf('Using dataset %s\n\n', dataset1);

C0 = dlmread(strcat('data/', dataset, '.pose'), '\t');
C1 = dlmread(strcat('data/', dataset1, '.traj'), '\t');
B = reshape(C1, [], 16);
%B = C1;
keys = C0(:, 8:8);
frames = B(:, 8:8);
N = size(B, 1);
kN = size(C0, 1);

% Extract Angle-axis data
AA0 = B(:,1:3);
AA1 = B(:,9:11);
% Extract Translations
T0 = B(:, 4:6);
NT0 = sqrt(sum(abs(T0).^2,2));
T1 = B(:, 12:14);
NT1 = sqrt(sum(abs(T1).^2,2));

point = [1;1;1];
point1 = [1;1;1];
traj1 = [];
traj2 = [];
for i = 1:N
    q0 = aa2quat(AA0(i, :)');
    q1 = aa2quat(AA1(i, :)');
    t0 = T0(i,:)';
    t1 = T1(i,:)';
    r0 = quat2rotm(q0);
    r1 = quat2rotm(q1);
    
    point = r0 * point;
    point1 = r1 * point1;
    point = point + t0;
    point1 = point1 + t1;
    traj1 = [traj1; point'];
    traj2 = [traj2; point1'];
end
% Extract Angle-axis data
kAA0 = C0(:,1:3);
kAA1 = C0(:,9:11);
% Extract Translations
kT0 = C0(:, 4:6);
NT0 = sqrt(sum(abs(T0).^2,2));
kT1 = C0(:, 12:14);
NT1 = sqrt(sum(abs(T1).^2,2));
kpoint = [1;1;1];
kpoint1 = [1;1;1];
ktraj1 = [];
ktraj2 = [];
for i = 1:kN
    kq0 = aa2quat(kAA0(i, :)');
    kq1 = aa2quat(kAA1(i, :)');
    kt0 = kT0(i,:)';
    kt1 = kT1(i,:)';
    kr0 = quat2rotm(kq0);
    kr1 = quat2rotm(kq1);
    
    kpoint = kr0 * kpoint;
    kpoint1 = kr1 * kpoint1;
    kpoint = kpoint + kt0;
    kpoint1 = kpoint1 + kt1;
    ktraj1 = [ktraj1; kpoint'];
    ktraj2 = [ktraj2; kpoint1'];
end

% keyVels1 = [];
% keyVels2 = [];
% for i = 1:size(keys,1)
%     key1 = keys(i) + 1
%     keyVels1 = [keyVels1; traj1(key1)];
%     keyVels2 = [keyVels2; traj2(key1)];
% end

plot3(traj1(:, 1:1), traj1(:, 2:2), traj1(:, 3:3))
hold
plot3(traj2(:, 1:1), traj2(:, 2:2), traj2(:, 3:3))
% scatter3(keyVels1(:, 1:1), keyVels1(:, 2:2), keyVels1(:, 3:3))
% scatter3(keyVels2(:, 1:1), keyVels2(:, 2:2), keyVels2(:, 3:3))