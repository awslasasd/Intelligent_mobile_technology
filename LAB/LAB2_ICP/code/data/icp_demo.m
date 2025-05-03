ply_0 = pcread('0.ply');
ply_1 = pcread('1.ply');

figure;
pcshowpair(ply_0, ply_1, 'MarkerSize', 50);

[tform, ply_1] = pcregistericp(ply_1, ply_0);

figure;
pcshowpair(ply_0, ply_1, 'MarkerSize', 50);

% 使用pcregistericp函数应用ICP算法。
% 这个函数会自动计算第二个点云（ply_1）相对于第一个点云（ply_0）的最优变换（包括旋转和平移），
% 以便最小化两个点云之间的差异。
% 返回的tform是一个变换对象，包含了从ply_1到ply_0的变换信息。
% 同时，ply_1会被更新为应用了这个变换之后的点云，即变换后的ply_1更好地与ply_0对齐。



% clear; clc;
% 
% % 初始点云
% ply_0 = pcread('0.ply');
% 
% for i = 1:9
%     % 读取当前点云
%     filename = sprintf('%d.ply', i);
%     curr_ply = pcread(filename);
% 
%     % 可视化原始配对点云
%     fig = figure;
%     pcshowpair(ply_0, curr_ply, 'MarkerSize', 50);
%     title(sprintf('Original ply_0 and ply_%d', i));
% 
%     % 保存可视化结果
%     saveas(fig, sprintf('Original_ply_0_and_ply_%d.png', i));
% 
%     % 使用ICP算法对齐点云
%     [tform, aligned_ply] = pcregistericp(curr_ply, ply_0);
% 
%     % 可视化对齐后的配对点云
%     fig = figure;
%     pcshowpair(ply_0, aligned_ply, 'MarkerSize', 50);
%     title(sprintf('Aligned ply_0 and ply_%d', i));
% 
%     % 保存可视化结果
%     saveas(fig, sprintf('Aligned_ply_0_and_ply_%d.png', i));
% 
%     % 更新ply_0为当前点云，为下一次迭代准备
%     ply_0 = curr_ply;
% end
% 
% for i = 0:9
%     % 读取当前点云
%     filename = sprintf('%d.ply', i);
%     curr_ply = pcread(filename);
% 
%     % 可视化当前点云
%     fig = figure;
%     pcshow(curr_ply, 'MarkerSize', 50);
%     title(sprintf('ply_%d', i));
% 
%     % 保存可视化结果
%     saveas(fig, sprintf('ply_%d.png', i));
% end

