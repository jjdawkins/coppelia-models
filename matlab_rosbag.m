% Specify the path to the ROS2 bag file
bagFilePath = '/home/ew-admin/coppelia-models/rosbag2_2024_08_14-13_16_33/rosbag2_2024_08_14-13_16_33_0.db3';

bagFilePath = '/home/ew-admin/coppelia-models/rosbag2_2024_08_14-13_48_27/rosbag2_2024_08_14-13_48_27_0.db3'

% Load the ROS2 bag file
bag = ros2bagreader(bagFilePath);

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


% Plot the data
figure
tiledlayout(7,1)
for i = 1:7
  nexttile
  plot(time, data(i,:))
end


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

% Plot the reference and actual velocity data
figure
tiledlayout(2,1)
for i=1:2
  nexttile
  plot(time, ref_vel_data(i,:), 'r')
  hold on
  grid on
  plot( time, act_vel_data(i,:), 'b')
  legend('Reference Velocity', 'Actual Velocity')
end



