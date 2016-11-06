clc;

% % Vary number of deltas
% T_ERR = [];
% R_ERR = [];
% groups = [];
% for i = 5:5:100
%     for j = 1:1000
%         [t_err, r_err] = genTest(i, 15, 1.0);
%         T_ERR = [T_ERR; t_err];
%         R_ERR = [R_ERR; r_err];
%         groups = [groups; i]
%     end
% end
% figure
% boxplot(R_ERR, groups)
% xlabel('Number of Deltas', 'fontsize', 15)
% ylabel('Rotational Error', 'fontsize', 15)
% angleFile = 'numDeltasVsRot';
% set(gca,'YScale','log')
% set(gca,'XTick', 0:5:100);
% saveas(gcf,angleFile);
% close;
% figure
% boxplot(T_ERR, groups)
% xlabel('Number of Deltas', 'fontsize', 15)
% ylabel('Translation Error', 'fontsize', 15)
% angleFile = 'numDeltasVsTrans';
% set(gca,'YScale','log')
% set(gca,'XTick', 0:5:100);
% saveas(gcf,angleFile);
% close;

% Vary size of deltas
% T_ERR = [];
% R_ERR = [];
% groups = [];
% for i = 1:5:45
%     for j = 1:100
%         [t_err, r_err] = genTest(20, i, 1.0);
%         T_ERR = [T_ERR; t_err];
%         R_ERR = [R_ERR; r_err];
%         groups = [groups; i]
%     end
% end
% figure
% boxplot(R_ERR, groups)
% xlabel('Size of Deltas', 'fontsize', 15)
% ylabel('Rotational Error', 'fontsize', 15)
% angleFile = 'sizeDeltasVsRot';
% %set(gca,'XTick', 0:5:45);
% set(gca, 'XTickLabel', 0:5:45);
% set(gca,'YScale','log')
% 
% saveas(gcf,angleFile);

% Vary angular noise
T_ERR = [];
R_ERR = [];
groups = [];
for i = 1:1:3
    z = .0001 * (10^i);
    for j = 1:1000
        [t_err, r_err] = genTest(20, 15, z);
        T_ERR = [T_ERR; t_err];
        R_ERR = [R_ERR; r_err];
        groups = [groups; z]
    end
end
figure
%boxplot(R_ERR, groups)
xlabel('Size of Noise', 'fontsize', 15)
ylabel('Rotational Error', 'fontsize', 15)
angleFile = 'sizeNoiseVsRot';
%set(gca,'YScale','log')
%set(gca,'XTick', 0:10:1);
saveas(gcf,angleFile);
% figure
boxplot(T_ERR, groups)