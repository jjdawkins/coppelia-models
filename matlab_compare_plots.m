close all; clear all; clc;

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

mat_file_paths

% adaptive then non-adaptive


% SIN wave experiment
comparison_select = [3, 4];
file_prefix = 'sin_exp_comparison_';
image_path = '/Users/allan/projects/matlab/lyapunov_sos_rover/latex/images/';


%const experiment
comparison_select = [8, 7];
file_prefix = 'const_exp_comparison_';
image_path = '/Users/allan/projects/matlab/lyapunov_sos_rover/latex/images/';



save = 1;


data = {};
% load the selected mat files as data
for i = 1:length(comparison_select)
    data{i} = load(mat_file_paths{comparison_select(i)});
end

data_label = ["NS-AVTC", "VTC"];

plot_colors = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30", "#4DBEEE", "#A2142F"];

param_names = {"$m$", "$J_z$", "$K_t$", "$C_{rr}$", "$C_{\alpha f}$", "$C_{\Sigma}$", "$C_{\Delta}$"};
param_units = {"$kg$", "$kg \cdot m^2$", "$N/A$", "$N \cdot s/m$", "$N/rad$", "$N/rad$", "$N/rad$"};
velocity_titles = ["$\dot{x}$", "$\dot{\psi}$", "$\dot{y}$"];
delta_v_titles = ["$\Delta \dot{x}$", "$\Delta \dot{\psi}$"];
time_label = "Time(s)";

velocity_units = {'$m/s$', '$rad/s$', '$m/s$'};


title_options = {'Interpreter', 'latex', 'FontSize', 22};
label_options = {'Interpreter', 'latex', 'FontSize', 20};
plot_options = {'LineWidth', 2};
tiled_options = {'TileSpacing', 'compact', 'Padding', 'compact'};
legend_options = {'fontsize', 15, 'Location', 'best'};

image_options = {'-dpdf', '-bestfit', '-r0 '};

% plot act_velocities vs time for each dataset into a single plot
f=figure;
f.Position = [215 603 619 659];
t=tiledlayout(2,1, tiled_options{:});
for j = 1:2
    nexttile
    for i = 1:2
        hold on
        % find out whent the
        plot(data{i}.act_vel_time, data{i}.act_vel_data(j,:), plot_options{:}, 'DisplayName', data_label(i))
        if i == 2
            title(velocity_titles(j), title_options{:})
            grid on
            xlabel(time_label, label_options{:})
            ylabel(velocity_units(j), label_options{:})
            plot(data{i}.ref_vel_time, data{i}.ref_vel_data(j,:), plot_options{:}, 'DisplayName', 'Reference', 'LineStyle', '--', 'Color', 'black')       
        end
    end
    legend(legend_options{:})
    ylim([0.5, 1.8])
    % cut it off at the shortest data length
    xlim([0, min(data{1}.act_vel_time(end), data{2}.act_vel_time(end))])
end

% save the plot
if save(1)
    print(f, fullfile(image_path, strcat(file_prefix, 'velocities.pdf')), image_options{:})
end


delta_v_titles = ["$\Delta \dot{x}$", "$\Delta \dot{\psi}$"];

% plot delta velocities
f = figure;
f.Position = [274 687 618 556];
t = tiledlayout(2,1, tiled_options{:});
for j = 1:2
    ax = nexttile;
    for i = 1:2
        hold on
        plot(data{i}.ref_vel_time, data{i}.delta_v(j,:),'DisplayName', data_label(i), plot_options{:})
        if i == 1
            title(delta_v_titles(j), title_options{:})
            grid on
            xlabel(time_label, label_options{:})
            ylabel(velocity_units(j), label_options{:})
            yline(0, plot_options{:}, 'LineStyle', '--', 'Color', 'black', 'DisplayName', 'Zero Error')
        end
    end
    ylim([-01.0, 1.0])
    legend(legend_options{:})
    % cut it off at the shortest data length
    xlim([0, min(data{1}.act_vel_time(end), data{2}.act_vel_time(end))])
end

% save the plot
if (save)
    print(f, fullfile(image_path, strcat(file_prefix, 'delta_velocities.pdf')), image_options{:})
end


% PLOT THE PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Break up the params into 2 separate groups
p_group_1 = [1 3 4];
p_group_2 = [2 5 6 7];

% norm each group of parameters
est_param_norm  = zeros(size(data{1}.est_param_data));
for i=1:size(data{1}.est_param_data, 2)
    new_vector = zeros(7,1);
    new_vector(p_group_1) = data{1}.est_param_data(p_group_1,i)./norm(data{1}.est_param_data(p_group_1,i));
    
    new_vector(p_group_2) = data{1}.est_param_data(p_group_2,i)./norm(data{1}.est_param_data(p_group_2,i));

    est_param_norm(:,i) = new_vector;
end

% print the final parameter values
fprintf('Final Parameter Values\n')
data{1}.est_param_data(:,end)

% plot the parameters for the adaptive case! NORMALIZED
f = figure;
m = 7;
f.Position = [715 529 648 854];
tiledlayout(m,1, tiled_options{:})
for i = 1:m
    ax = nexttile;
    plot(data{1}.est_param_time, est_param_norm(i,:), plot_options{:})
    hold on
    grid on
    title(param_names{i}, title_options{:}, 'FontSize', 20)
    if i==m
    xlabel(time_label, label_options{:})
    end
    %ylabel(param_units{i}, label_options{:})
    ylim_pad = 0.05;
    % set Ylim to be 5% above and below the min and max values
    % cut off the plot at the shortest data length
    xlim([0, min(data{1}.est_param_time(end), data{2}.est_param_time(end))])
end

if (save)
    print(f, fullfile(image_path, strcat(file_prefix, 'adaptive_params_norm.pdf')), image_options{:})
end

% plot the parameters for the adaptive case - not normalized
f = figure;
m = 7;
f.Position = [715 529 648 854];
tiledlayout(m,1, tiled_options{:})
for i = 1:m
    ax = nexttile;
    plot(data{1}.est_param_time, data{1}.est_param_data(i,:), plot_options{:})
    hold on
    grid on
    title(param_names{i}, title_options{:}, 'FontSize', 20)
    if i==m
    xlabel(time_label, label_options{:})
    end
    ylabel(param_units{i}, label_options{:})
    ylim_pad = 0.05;
    % set Ylim to be 5% above and below the min and max values
    % cut off the plot at the shortest data length
    xlim([0, min(data{1}.est_param_time(end), data{2}.est_param_time(end))])
end

if (save)
    print(f, fullfile(image_path, strcat(file_prefix, 'adaptive_params.pdf')), image_options{:})
end