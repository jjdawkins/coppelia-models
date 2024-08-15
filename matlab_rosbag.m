% Specify the path to the ROS2 bag file
%bagFilePath = 'rosbag2_2024_08_14-13_16_33/rosbag2_2024_08_14-13_16_33_0.db3';
%bagFilePath = 'rosbag2_2024_08_14-13_48_27/rosbag2_2024_08_14-13_48_27_0.db3';
bagFilePath = 'rosbag2_2024_08_14-13_53_16/rosbag2_2024_08_14-13_53_16_0.db3';

% Load the ROS2 bag file
bag = ros2bagreader(bagFilePath);

% print the topics in the bag file
topics = bag.AvailableTopics

% scale the amperage and turn values
amperage_scale = 15;
turn_scale = 0.35;

% Select the topic you want to plot
topic = '/rover/est_param';

% Read the messages from the selected topic
msgs = readMessages(select(bag, 'Topic', topic));

time = []; %this will be a 1xn
data = []; % this will be a 7xn

% loop over each message and extract the data
for i = 1:length(msgs)
  time = [time, msgs{i}.data(1)];
  data = [data, msgs{i}.data(2:end)];
end

close all;
n_p = size(data, 1)
param_names = ["$m$", "$j_z$", "$k_t$", "$c_{rr}$", "$c_{\alpha f}$", "$c_{\Sigma}$", "$c_{\Delta}$"];
param_units = ["kg", "kg m^2", "N m", "N m s", "N/rad", "N m", "N m"];

title_options = {'Interpreter', 'latex', 'FontSize', 18};
label_options = {'Interpreter', 'latex', 'FontSize', 14};


% PLOT THE ESTIMATED PARAMETERS VALUES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f = figure
f.Position = [68 145 950 1121];
tiledlayout(n_p,1)
% make overall title for the plot
sgtitle('NSAID Ground Vehicel Controller Estimated Parameters', title_options{:})
for i = 1:n_p
  nexttile
  plot(time, data(i,:), "LineWidth", 2)
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
for i=1:3
  act_vel_data(i,:) = filter(ones(1,10)/10, 1, act_vel_data(i,:));
end


velocity_titles = ["$\dot{x}$", "$\dot{\psi}$", "$\dot{y}$"];
velocity_units = ["m/s", "rad/s", "m/s"];
% PLOT THE REFERENCE AND ACTUAL VELOCITIES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f=figure
tiledlayout(3,1)
for i=1:3
  nexttile
  hold on
  title(velocity_titles(i), title_options{:})
  plot( time, act_vel_data(i,:),  'LineWidth', 1.5)
  plot(time, ref_vel_data(i,:), 'LineWidth', 1.5)
  grid on
  legend('Actual Velocity', 'Reference Velocity', 'Location', 'Best')
  xlabel('Time (s)', label_options{:})
  ylabel(velocity_units(i), label_options{:})
end

% save in rosbag folder
saveas(f, replace(bagFilePath, '.db3', '_velocities.png'))

% PLOT THE DELTA VELOCITIES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta_v = ref_vel_data - act_vel_data;

delta_v_titles = ["$\Delta \dot{x}$", "$\Delta \dot{\psi}$"];

f=figure
tiledlayout(2,1)
for i=1:2
  nexttile
  hold on
  plot( time, delta_v(i,:),  'LineWidth', 1.5)
  title(delta_v_titles(i), title_options{:})
  yline(0, 'LineWidth', 1.5)
  grid on
  xlabel('Time (s)', label_options{:})
  ylabel(velocity_units(i), label_options{:})
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