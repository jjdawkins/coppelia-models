close all;

% get all rosbags in rosbag folder
folders = dir('rosbags');
desired_plots = 1;

plot_count = 0;
for i = 1:length(folders)
  if plot_count >= desired_plots
    break
  end
  folder = folders(i);
  if folder.isdir && ~strcmp(folder.name, '.') && ~strcmp(folder.name, '..')
    bagFiles = dir(fullfile('rosbags', folder.name, '*.db3'));
    for j = 1:length(bagFiles)
      bagFilePath = fullfile('rosbags', folder.name, bagFiles(j).name)
      visualize_rosbag(bagFilePath);
      plot_count = plot_count +1;
    end
  end
end


function out = visualize_rosbag(bagFilePath)

% Load the ROS2 bag file
bag = ros2bagreader(bagFilePath);

% print the topics in the bag file
topics = bag.AvailableTopics

% scale the amperage and turn values
amperage_scale = 15;
turn_scale = 0.35;

% First plot the /rover/mocap/odom topic POSITION %%%%%%%%%%%%%%
topic = '/rover/mocap/odom';
msgs = readMessages(select(bag, 'Topic', topic));

msgs{1}.pose.pose.position

positions = []; % this will be a 3xn

for i = 1:length(msgs)
  new_position = [msgs{i}.pose.pose.position.x, msgs{i}.pose.pose.position.y, msgs{i}.pose.pose.position.z];
  positions = [positions, new_position'];
end

f = figure
f.Position = [68 145 950 1121];
position_titles = ["$x$", "$y$", "$z$"];
tiledlayout(3,1)
sgtitle('NSAID Ground Vehicle Position', 'Interpreter', 'latex', 'FontSize', 18)
for i = 1:3
  nexttile
  plot(positions(i,:), "LineWidth", 2)
  title(position_titles(i), 'Interpreter', 'latex', 'FontSize', 14)
  grid on
  xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14)
end

% now plot velocities
f = figure
f.Position = [68 145 950 1121];
velocities = diff(positions, 1, 2);
velocity_titles = ["$\dot{x}$", "$\dot{y}$", "$\dot{z}$"];
tiledlayout(3,1)
sgtitle('NSAID Ground Vehicle Velocity', 'Interpreter', 'latex', 'FontSize', 18)
for i = 1:3
  nexttile
  plot(velocities(i,:), "LineWidth", 2)
  title(velocity_titles(i), 'Interpreter', 'latex', 'FontSize', 14)
  grid on
  xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14)
end

% now plot the twist message

twists = []; % this will be a 3xn
for i = 1:length(msgs)
  new_twist = [msgs{i}.twist.twist.linear.x, msgs{i}.twist.twist.linear.y, msgs{i}.twist.twist.angular.z];
  twists = [twists, new_twist'];
end

lims = {[0.5, 1.5], [-0.3, 0.3], [0.5, 1.5]};

f = figure
f.Position = [68 145 950 1121];
twist_titles = ["$\dot{x}$", "$\dot{y}$", "$\dot{z}$"];
tiledlayout(3,1)
sgtitle('NSAID Ground Vehicle Twist', 'Interpreter', 'latex', 'FontSize', 18)
for i = 1:3
  nexttile
  plot(twists(i,:), "LineWidth", 2)
  title(twist_titles(i), 'Interpreter', 'latex', 'FontSize', 14)
  grid on
  xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14)
  ylim(lims{i})
end


% now plot the time delay between messages
times = [];
for i = 1:length(msgs)
  this_t = double(msgs{i}.header.stamp.sec) + double(msgs{i}.header.stamp.nanosec) *1e-9
  times = [times, this_t];
end

time_diffs = diff(times);

time_diffs


f = figure
f.Position = [68 145 950 1121];
tiledlayout(1,1)
sgtitle('NSAID Ground Vehicle Time Delay', 'Interpreter', 'latex', 'FontSize', 18)
nexttile
plot(time_diffs, "LineWidth", 2)
title('Time Delay', 'Interpreter', 'latex', 'FontSize', 14)
grid on
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14)



return



% Select the topic you want to plot
topic = '/rover/est_param';

% Read the messages from the selected topic
msgs = readMessages(select(bag, 'Topic', topic));

est_param_time = []; %this will be a 1xn
est_param_data = []; % this will be a 7xn

% loop over each message and extract the data
for i = 1:length(msgs)
  est_param_time = [est_param_time, msgs{i}.data(1)];
  est_param_data = [est_param_data, msgs{i}.data(2:end)];
end

close all;
n_p = size(est_param_data, 1)
param_names = ["$m$", "$j_z$", "$k_t$", "$c_{rr}$", "$c_{\alpha f}$", "$c_{\Sigma}$", "$c_{\Delta}$"];
param_units = ["kg", "kg m^2", "N m", "N m s", "N/rad", "N m", "N m"];

title_options = {'Interpreter', 'latex', 'FontSize', 18};
label_options = {'Interpreter', 'latex', 'FontSize', 14};


% PLOT THE ESTIMATED PARAMETERS VALUES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f = figure
f.Position = [68 145 950 1121];
tiledlayout(n_p,1)
% make overall title for the plot
sgtitle('NSAID Ground Vehicle Controller Estimated Parameters', title_options{:})
for i = 1:n_p
  nexttile
  plot(est_param_time, est_param_data(i,:), "LineWidth", 2)
  title(param_names(i), title_options{:})
  grid on
  xlabel('Time (s)', label_options{:})
end

% save in rosbag folder
saveas(f, replace(bagFilePath, '.db3', '_est_param.png'))

% Read the /rover/ref_vel and /rover/act_vel topics
ref_vel_msgs = readMessages(select(bag, 'Topic', '/rover/ref_vel'));
act_vel_msgs = readMessages(select(bag, 'Topic', '/rover/act_vel'));

% Extract the time and data from the messages
ref_vel_time = [];
ref_vel_data = [];
for i = 1:length(ref_vel_msgs)
  ref_vel_time = [ref_vel_time, ref_vel_msgs{i}.data(1)];
  ref_vel_data = [ref_vel_data, ref_vel_msgs{i}.data(2:end)];
end

act_vel_time = [];
act_vel_data = [];
for i = 1:length(act_vel_msgs)
  act_vel_time = [act_vel_time, act_vel_msgs{i}.data(1)];
  act_vel_data = [act_vel_data, act_vel_msgs{i}.data(2:end)];
end

% run a low pass filter on actual velocity
filter_order = 10;
filter_kernel = ones(1, filter_order) / filter_order;

% for i = 1:3
%   act_vel_data(i,:) = conv(act_vel_data(i,:), filter_kernel, 'same');
% end



velocity_titles = ["$\dot{x}$", "$\dot{\psi}$", "$\dot{y}$"];
velocity_units = ["m/s", "rad/s", "m/s"];
% PLOT THE REFERENCE AND ACTUAL VELOCITIES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f=figure
f.Position = [68 145 950 1121];
tiledlayout(3,1)
for i=1:3
  nexttile
  hold on
  title(velocity_titles(i), title_options{:})
  plot(act_vel_time, act_vel_data(i,:),  'LineWidth', 1.5)
  plot(ref_vel_time, ref_vel_data(i,:), 'LineWidth', 1.5)
  grid on
  legend('Actual Velocity', 'Reference Velocity', 'Location', 'Best')
  xlabel('Time (s)', label_options{:})
  ylabel(velocity_units(i), label_options{:})
  ylim([0.5, 1.5])
end

% save in rosbag folder
saveas(f, replace(bagFilePath, '.db3', '_velocities.png'))

% PLOT THE DELTA VELOCITIES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta_v = ref_vel_data - act_vel_data;

delta_v_titles = ["$\Delta \dot{x}$", "$\Delta \dot{\psi}$"];

f=figure
f.Position = [68 145 950 1121];
tiledlayout(2,1)
for i=1:2
  nexttile
  hold on
  plot( ref_vel_time, delta_v(i,:),  'LineWidth', 1.5)
  title(delta_v_titles(i), title_options{:})
  yline(0, 'LineWidth', 1.5)
  grid on
  xlabel('Time (s)', label_options{:})
  ylabel(velocity_units(i), label_options{:})
  % ylim  = [-0.5, 0.5]
  ylim([-0.5, 0.5])
end

% save in rosbag folder
saveas(f, replace(bagFilePath, '.db3', '_delta_velocities.png'))


% PLOT THE MOTOR AMPERAGE AND STEER ANGLE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cmd_vel_msgs = readMessages(select(bag, 'Topic', '/rover/cmd_vel'));
cmd_vel_data = [];
for i = 1:length(cmd_vel_msgs)
  data_i = [cmd_vel_msgs{i}.linear.x * amperage_scale; cmd_vel_msgs{i}.angular.z * turn_scale];
  cmd_vel_data = [cmd_vel_data, data_i];
end


cmd_vel_titles = ["Motor Amperage, $I$", "Steer Angle, $\delta$"];
cmd_vel_units = ["A", "rad"];

f=figure
tiledlayout(2,1)
for i=1:2
  nexttile
  hold on
  plot(cmd_vel_data(i,:), 'LineWidth', 1.5)
  title(cmd_vel_titles(i), title_options{:})
  ylabel(cmd_vel_units(i), label_options{:})
  grid on
  xlabel('Time (s)', label_options{:})
end

% save in rosbag folder
saveas(f, replace(bagFilePath, '.db3', '_cmd_vel.png'))


% save the .mat with est_param_time, est_param_data, ref_vel_time, ref_vel_data, act_vel_time, act_vel_data, delta_v, cmd_vel_data to the rosbag folder
save(replace(bagFilePath, '.db3', '.mat'), 'est_param_time', 'est_param_data', 'ref_vel_time', 'ref_vel_data', 'act_vel_time', 'act_vel_data', 'delta_v', 'cmd_vel_data')


end