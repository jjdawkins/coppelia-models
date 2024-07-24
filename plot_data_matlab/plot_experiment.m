%clc; clear all; close all;
save = 0; % save or do not save plots

% load rosbag
bagReader = ros2bag("rosbag2_2024_07_19-13_08_42/");

% get all topics
bagReader.AvailableTopics

% get the /rover/est_param data as a table
est_param_msgs = readMessages(select(bagReader, 'Topic', '/rover/est_param'));

%access each message as est_param{i}.data

% Plot the Param Data
est_param_t = [];
est_param = [];
for i = 1:length(est_param_msgs)
    vec = est_param_msgs{i}.data;
    est_param_t = [est_param_t; vec(1)];
    est_param = [est_param; vec(2:end)'];
end

% num params
param_names = ["$m$", "$j_z$", "$k$", "$c_{rr}$", "$c_{af}$", "$c_{\Sigma}$", "$c_{\Delta}$"];
p = size(est_param, 2);

% PLOT ALL PARAMS
close all;
f = figure;
f.Position = [334 240 883 1068];
axes = tiledlayout(p, 1);
% keep the figure tight
axes.Padding = 'compact';
for i = 1:p
    nexttile;
    plot(est_param_t, est_param(:, i), 'LineWidth', 1.50);
    title(param_names(i), 'Interpreter', 'latex', "FontSize", 20);
    ylabel("Value", "Interpreter", "latex", "FontSize", 15);
    xlabel("Time (s)", "Interpreter", "latex", "FontSize", 15);
    grid on;
end

% PLOT SELECTED PARAMS
param_select = [2, 5, 6, 7];
% make tiled laout plot of each param
f = figure;
f.Position = [334 240 883 1068];
axes = tiledlayout(length(param_select), 1);
% keep the figure tight
axes.Padding = 'compact';
for i = 1:length(param_select)
    nexttile;
    plot(est_param_t, est_param(:, param_select(i)), 'LineWidth', 1.50);
    title(param_names(param_select(i)), 'Interpreter', 'latex', "FontSize", 20);
    ylabel("Value", "Interpreter", "latex", "FontSize", 15);
    xlabel("Time (s)", "Interpreter", "latex", "FontSize", 15);
    grid on;
end


% get the /rover/act_vel data and /rover/ref_vel data
act_vel_msgs = readMessages(select(bagReader, 'Topic', '/rover/act_vel'));
ref_vel_msgs = readMessages(select(bagReader, 'Topic', '/rover/ref_vel'));

% Plot the Vel Data
act_vel_t = [];
act_vel = [];
for i = 1:length(act_vel_msgs)
    vec = act_vel_msgs{i}.data;
    act_vel_t = [act_vel_t; vec(1)];
    act_vel = [act_vel; vec(2:end)'];
end

ref_vel_t = [];
ref_vel = [];
for i = 1:length(ref_vel_msgs)
    vec = ref_vel_msgs{i}.data;
    ref_vel_t = [ref_vel_t; vec(1)];
    ref_vel = [ref_vel; vec(2:end)'];
end

vel_names = ["$\dot{x}$", "$\dot{\psi}$", "$\dot{y}$"];

% PLOT ALL VELS
f = figure;
f.Position = [334 240 883 1068];
axes = tiledlayout(length(vel_names), 1);
axes.Padding = 'compact';
for i = 1:length(vel_names)
    nexttile;
    plot(act_vel_t, act_vel(:, i), 'LineWidth', 1.50);
    hold on;
    plot(ref_vel_t, ref_vel(:, i), 'LineWidth', 1.50);
    title(vel_names(i), 'Interpreter', 'latex', "FontSize", 20);
    ylabel("Value", "Interpreter", "latex", "FontSize", 15);
    xlabel("Time (s)", "Interpreter", "latex", "FontSize", 15);
    legend("Actual", "Reference", "Interpreter", "latex", "FontSize", 15);
    grid on;
end


% Plot Vel Error
vel_error_names = ["$\Delta \dot{x}$", "$\Delta \dot{\psi}$", "$\Delta \dot{y}$"];
vel_error = act_vel - ref_vel;
% add filter
a = 1;
b = ones(1, 10)/10;
vel_error = filter(b, a, vel_error);
f = figure;
f.Position = [334 240 883 1068];
axes = tiledlayout(length(vel_names), 1);
axes.Padding = 'compact';
for i = 1:length(vel_names)
    nexttile;
    plot(act_vel_t, vel_error(:, i), 'LineWidth', 1.50);
    title(vel_error_names(i), 'Interpreter', 'latex', "FontSize", 20);
    ylabel("Value", "Interpreter", "latex", "FontSize", 15);
    xlabel("Time (s)", "Interpreter", "latex", "FontSize", 15);
    grid on;
    yline(0, 'r--', 'LineWidth', 1.50);
end


% PLOT POSITION
% get the /qualisys/rover/pose data (PoseStamped)
pose_msgs = readMessages(select(bagReader, 'Topic', '/qualisys/rover/pose'));

pose_t = []; 
pose_position = []; % column with x, y, z

for i=1:length(pose_msgs)
    pose_t = [pose_t; header_to_time(pose_msgs{i}.header)];
    pose_position = [pose_position; pose_msgs{i}.pose.position.x, pose_msgs{i}.pose.position.y, pose_msgs{i}.pose.position.z];
end

% PLOT POSITION in 2D
f = figure;
f.Position = [334 240 883 1068];
plot(pose_position(:, 1), pose_position(:, 2), 'LineWidth', 1.50);
title("Rover Position (Motion Capture)", "FontSize", 20);
xlabel("X Position (m)")
ylabel("Y Position (m)")
% make the plot square
axis equal;
% add a slider to change time range










function time = header_to_time(header)
    % convert header to time
    time = double(header.stamp.sec) + double(header.stamp.nanosec) * 1e-9;
end