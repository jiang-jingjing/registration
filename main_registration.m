%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function to register points to a surface based on ICP
% for fine tune the final registration of probe to head 
% created 2025.01.24
% Jingjing Jiang jing.jing.jiang@outlook.com
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load head model from Matlab
load(fullfile(toolboxdir("images"),"imdata","BrainMRILabeled","images","vol_001.mat"));
V = vol;
isovalue = 0.5;
[~,fixedSurface] = extractIsosurface(V,isovalue);
headModel = fixedSurface;
% generate points for testing (as the position sensors)
numPointsPerCircle = 8;
radius1 = 14;
radius2 = 22;
[moving_pts_original,originalPoints] = generatePoints(headModel, numPointsPerCircle, radius1, radius2);

% apply transformation and add noise
R_true = axang2rotm([0.5 0.5 0.5 pi/6]); % smaller rotation
t_true = [4 5 6]';
moving_pts_transformed = moving_pts_original * R_true + t_true';

noise_level = 0 
noise = randn(size(moving_pts_transformed)) * noise_level;  
moving_pts = moving_pts_transformed + noise;

% ICP using pcregistericp
target_pc = pointCloud(headModel);
moving_pc = pointCloud(moving_pts);

regParams = struct('MaxIterations', 500, 'Tolerance', [1e-5, 1e-5],'MaxInlierDistance',10); 

% initial guess
% %Using the original points as initial guess.
%  T_initial.T = [R_true  t_true
%     0 0 0 1]';  

% % other initial guess.
% R_init = axang2rotm([0.6 0.5 0.6 pi/2]);
% t_init = [0.5 1 -0.5]'   ;
% T_initial.T = [R_init  t_init  ;
%     0 0 0 1]';

translation_vector = (mean(moving_pts_original) - mean(moving_pts))'; % Transpose to make it a column vector
tform_initial = affine3d([eye(3), translation_vector; 0 0 0 1]');

tform = pcregistericp(moving_pc, target_pc, 'InitialTransform', tform_initial, ...
    'MaxIterations', regParams.MaxIterations, 'Tolerance', regParams.Tolerance,'MaxInlierDistance',regParams.MaxInlierDistance);

T = tform.T;
R_icp = T(1:3, 1:3);
t_icp = T(1:3, 4);

moving_pts_reg = pctransform(moving_pc, tform).Location;
moving_pc_reg = pointCloud(moving_pts_reg);
moving_pc_reg.Color = repmat([0 255 0], moving_pc_reg.Count, 1); % Green

% visualization
figure;
%     pcshow(target_pc); hold on; % No color arguments here
ax = pcshow(target_pc); hold on; % Get axes handle
% Set target point cloud color and transparency
target_pc.Color = repmat([0.7 0.7 1], target_pc.Count, 1); % Light blue/grayish
alpha(ax, 0.3); % Set transparency (alpha value between 0 and 1)

plot3(originalPoints(:,1), originalPoints(:,2), originalPoints(:,3), 'ro', 'MarkerSize', 8);
plot3(moving_pts_original(:,1), moving_pts_original(:,2), moving_pts_original(:,3), 'yo', 'MarkerSize', 8);
plot3(moving_pts_transformed(:,1), moving_pts_transformed(:,2), moving_pts_transformed(:,3), 'mo', 'MarkerSize', 8);%Points transformed with rough transformation
pcshow(moving_pc_reg); % No color arguments here


axis equal; view(3); grid on;
title('Point-to-Surface ICP with Projected Sparse Points');
h_le = legend('Target Surface','Original points circles','Original Sparse Points','Points with rough transformation','Registered Points');
h_le.TextColor = 'w';
hold off;


%% distance errors
metric_ori.distance= vecnorm(moving_pts_original-moving_pc_reg.Location,2,2 );
num_point = length(metric_ori.distance);
metric_ori.distance = sqrt(sum(metric_ori.distance.^2)./num_point)
metric.distance= vecnorm(moving_pts_original-moving_pts_transformed,2,2 );
metric.distance= sqrt(sum(metric.distance.^2)./num_point)