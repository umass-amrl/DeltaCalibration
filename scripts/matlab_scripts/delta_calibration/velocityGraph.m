dataset = 'wide_horz_long';
dataset1 = 'wide_horz_long';

dataset = strcat(dataset, '_', int2str(5))
fprintf('Using dataset %s\n\n', dataset);

dataset1 = strcat(dataset1, '_', int2str(5))
fprintf('Using dataset %s\n\n', dataset1);

C0 = dlmread(strcat('data/', dataset, '.pose'), '\t');
C1 = dlmread(strcat('data/', dataset1, '.velocity'), '\t');
keys = C0(:, 8:8);
N = size(C1, 1);

vel1 = C1(:,2:2);
vel2 = C1(:,3:3);
keyVels1 = [];
keyVels2 = [];
for i = 1:size(keys,1)
    key1 = keys(i) + 1
    if key1 < size(vel1, 1)
        keyVels1 = [keyVels1; vel1(key1)];
        keyVels2 = [keyVels2; vel2(key1)];
    end
end
keys = keys + 1
figure
plot(abs(vel1))
hold
plot(abs(vel2))
scatter(keys, abs(keyVels1))
scatter(keys, abs(keyVels2))
xlabel('Frame', 'fontsize', 15)
ylabel('Residual Speed', 'fontsize', 15)
