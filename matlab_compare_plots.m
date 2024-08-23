close all;

rosbag_folders = dir('rosbags');


% get the all mat file paths in a cell array
mat_file_paths = {};
for i = 1:length(rosbag_folders)
    folder = rosbag_folders(i);
    bagFiles = dir(fullfile('rosbags', folder.name, '*.mat'));
    for j = 1:length(bagFiles)
        mat_file_paths{end+1} = fullfile('rosbags', folder.name, bagFiles(j).name);
    end
end


comparison_select = [4, 5];

data = {};
% load the selected mat files as data
for i = 1:length(comparison_select)
    data{i} = load(mat_file_paths{comparison_select(i)});
end

data_label = ["Adaptive", "Non-Adaptive"];

plot_colors = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30", "#4DBEEE", "#A2142F"];

velocity_titles = ["$\dot{x}$", "$\dot{\psi}$", "$\dot{y}$"];
velocity_units = ["m/s", "rad/s", "m/s"];

title_options = {'Interpreter', 'latex', 'FontSize', 18};
label_options = {'Interpreter', 'latex', 'FontSize', 14};

% plot act_velocities vs time for each dataset into a single plot
f=figure;
f.Position = [413 103 1480 1144];
t=tiledlayout(3,1);
for j = 1:3
    nexttile
    for i = 1:2
        hold on
        plot(data{i}.act_vel_time, data{i}.act_vel_data(j,:), "LineWidth", 1.5, 'DisplayName', data_label(i))
        if i == 2
            title(velocity_titles(j), title_options{:})
            grid on
            xlabel('Time (s)', label_options{:})
            ylabel(velocity_units(j), label_options{:})
            plot(data{i}.ref_vel_time, data{i}.ref_vel_data(j,:), "LineWidth", 4, 'DisplayName', 'Reference')       
        end
    end
    legend('Location', 'Best')
    ylim([0.5, 1.5])
end

hold off
% make tight layout!
t.TileSpacing = 'compact';
t.Padding = 'compact';


delta_v_titles = ["$\Delta \dot{x}$", "$\Delta \dot{\psi}$"];

% plot delta velocities
f = figure;
f.Position = [413 103 1480 1144];
t = tiledlayout(2,1);
for j = 1:2
    nexttile
    for i = 1:2
        hold on
        plot(data{i}.ref_vel_time, data{i}.delta_v(j,:), "LineWidth", 2, 'DisplayName', data_label(i))
        if i == 1
            title(delta_v_titles(j), title_options{:})
            grid on
            xlabel('Time (s)', label_options{:})
            ylabel(velocity_units(j), label_options{:})
            yline(0, 'LineWidth', 2, 'DisplayName', 'Reference')
        end
    end
    ylim([-0.5, 0.5])
    legend('Location', 'Best')
end




