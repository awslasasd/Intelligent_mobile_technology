clc;
clear;

% 参数设置
data_path = '../data/';
cloud_num = 9;
reserve_ratio = 0.93;
merge_threshold = 0.01;

% 初始化
target_cloud = pcread(fullfile(data_path, '0.ply'));
matlab_map = target_cloud;                    % MATLAB库函数的地图
custom_map = target_cloud;                    % 自定义ICP的地图
transform_init = affine3d(eye(4));           % 初始变换矩阵
custom_trajectory = zeros(cloud_num + 1, 3); % 自定义ICP的轨迹
matlab_trajectory = zeros(cloud_num + 1, 3); % MATLAB库函数的轨迹

% 主循环：点云匹配和地图构建
for i = 1:cloud_num
    % 读取当前帧点云
    source_cloud = pcread(fullfile(data_path, sprintf('%d.ply', i)));
    
    % 自定义ICP匹配
    [R, t, aligned_cloud] = ICP(target_cloud, source_cloud, reserve_ratio);
    
    % MATLAB库函数ICP匹配
    [transform_init, aligned_cloud_matlab] = pcregistericp(source_cloud, matlab_map, ...
        'Metric', 'pointToPoint', ...
        'InitialTransform', transform_init, ...
        'MaxIterations', 100, ...
        'Tolerance', [0.01, 0.001]);
    
    % 更新点云地图
    matlab_map = pcmerge(matlab_map, aligned_cloud_matlab, merge_threshold);
    custom_map = pcmerge(custom_map, aligned_cloud, merge_threshold);
    
    % 记录轨迹
    custom_trajectory(i+1,:) = t';
    matlab_trajectory(i+1,:) = transform_init.Translation;
    
    % 更新目标点云
    target_cloud = aligned_cloud;
    
    % fprintf('处理第 %d/%d 帧点云...\n', i, cloud_num);
end

% 最终轨迹和地图
traj_plot(custom_map, pointCloud(custom_trajectory), pointCloud(matlab_trajectory), 10);

% 计算轨迹误差
trajectory_diff = custom_trajectory - matlab_trajectory;
total_error = sum(sqrt(trajectory_diff(:,1).^2 + trajectory_diff(:,2).^2));
fprintf('轨迹总误差: %.4f\n', total_error);

function [R_f, t_f, p_transformed] = ICP(targetPC, sourcePC, reserveRatio)
    % 设置最大迭代次数
    maxIter = 100;  

    % 提取点云数据，每个点作为列向量
    targetPts = targetPC.Location';
    sourcePts = sourcePC.Location';
    
    % 点云点数
    numPts = length(targetPts);
    
    % 第一次迭代
    % 计算初始质心
    centerTarget = mean(targetPts, 2);
    centerSource = mean(sourcePts, 2);
    
    % 初始参数
    R_current = eye(3);
    R_f = eye(3);
    t_current = centerTarget - centerSource;
    t_f = t_current;
    
    % 第一次旋转结果
    sourcePts = R_current * sourcePts + t_current;
    
    % 迭代参数初始化
    iteration = 0;
    err_prev = inf;

    for iteration = 1 : maxIter
        % 对于每个源点，找到目标点云中距离最近的点
        distances = zeros(1, numPts);
        indices   = zeros(1, numPts);
        for i = 1:numPts
            diff = targetPts - repmat(sourcePts(:, i), 1, numPts);
            dists = sqrt(sum(diff.^2, 1));
            [distances(i), indices(i)] = min(dists);
        end
        
        % 根据匹配结果，重新排列目标点云的对应点
        matchedTarget = targetPts(:, indices);
        
        % 根据保留比例删除距离较大的匹配对
        numKeep = floor(numPts * reserveRatio);
        [~, sortIdx] = sort(distances);
        targetFiltered = matchedTarget(:, sortIdx(1:numKeep));
        sourceFiltered = sourcePts(:, sortIdx(1:numKeep));
        
        % 计算选中点集的质心
        centerTarget_filtered = mean(targetFiltered, 2);
        centerSource_filtered = mean(sourceFiltered, 2);
        
        % 去质心化
        targetZeroMean = targetFiltered - centerTarget_filtered;
        sourceZeroMean = sourceFiltered - centerSource_filtered;
        
        % 计算协方差矩阵
        W = zeros(3, 3);
        for i = 1:numKeep
            W = W + sourceZeroMean(:, i) * targetZeroMean(:, i)';
        end
        
        % SVD 分解求最优旋转
        [U, ~, V] = svd(W);
        R_current = V * U';
        
        % 求最优平移
        t_current = centerTarget_filtered - R_current * centerSource_filtered;
        
        % 累积变换
        R_f = R_current * R_f;
        t_f = R_current * t_f + t_current;
        
        % 更新源点云坐标
        sourcePts = R_current * sourcePts + t_current;
        
        % 计算本次迭代匹配对的总误差
        err = 0;
        for i = 1:numKeep
            err = err + norm(targetFiltered(:, i) - sourceFiltered(:, i));
        end
        if abs(err - err_prev) < 1e-6
            break;
        end
        err_prev = err;
    end
    % disp(iteration);
    
    % 输出旋转矩阵、平移向量和变换后的点云
    p_transformed = pointCloud(sourcePts');
end

function [] = traj_plot(target_cloud, source_cloud, aligned_cloud, i)
    % 创建图形
    fig = figure;
    
    % 设置点云显示参数
    markerSize = 50;
    
    pcloud = pcmerge(source_cloud, target_cloud, 0.01);
    pcshowpair( pcloud, aligned_cloud, 'MarkerSize', markerSize);
    title('最终轨迹对比');
    print(fig, fullfile('result', 'Final_trajectory_comparison.png'), '-dpng');
end