close all; clearvars; clc;

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
comparison_select = [23, 17, 21];
comparison_select = [40, 39, 37];
comparison_select = [47, 43, 44, 46];

file_prefix = 'sin_exp_comparison_';
image_path = '/Users/allan/projects/matlab/lyapunov_sos_rover/latex/images/';



%const experiment
% comparison_select = [8, 7, 13];
% file_prefix = 'const_exp_comparison_';
% image_path = '/Users/allan/projects/matlab/lyapunov_sos_rover/latex/images/';


image_path = '/Users/allan/projects/matlab/lyapunov_sos_rover/latex/images/ral_modified';
%image_path = 'temp_images';

save = 1;


data = {};
% load the selected mat files as data
for i = 1:length(comparison_select)
    data{i} = load(mat_file_paths{comparison_select(i)});
end

data_label = ["AVTC", "VTC", "VTC-I", "ADRC"];

plot_colors = ["#D95319", "#EDB120", "#0072BD", "#7E2F8E", "#77AC30", "#4DBEEE", "#A2142F"];

param_names = {"$m$", "$J_z$", "$K_t$", "$C_{rr}$", "$C_{\alpha f}$", "$C_{\Sigma}$", "$C_{\Delta}$"};
param_units = {"$kg$", "$kg \cdot m^2$", "$N/A$", "$N \cdot s/m$", "$N/rad$", "$N/rad$", "$N/rad$"};
velocity_titles = ["$\dot{x}$ vs Time", "$\dot{\psi}$ vs Time", "$\dot{y}$ vs Time"];
delta_v_titles = ["$\Delta \dot{x}$ vs Time", "$\Delta \dot{\psi}$ vs Time"];
time_label = "Time(s)";

velocity_units = {'$\dot{x} \quad (m/s)$', '$\dot{\psi} \quad (rad/s)$', '$\dot{y} \quad (m/s)$'};

title_options = {'Interpreter', 'latex', 'FontSize', 22};
label_options = {'Interpreter', 'latex', 'FontSize', 20};
plot_options = {'LineWidth', 2};
tiled_options = {'TileSpacing', 'compact', 'Padding', 'compact'};
legend_options = {'fontsize', 14, 'Location', 'northeast'};

% run through all data{i}.act_vel_data and remove any point that is >50% different from the running average
ind = find(data{2}.act_vel_data(2,:)<0.7)
ind = ind(end)
data{2}.act_vel_data(2,ind) = (data{2}.act_vel_data(2,ind-1) + data{2}.act_vel_data(2,ind+1))/2;

% do the same for delta_v
ind = find(data{2}.delta_v(2,:)>0.3)
ind = ind(end)
data{2}.delta_v(2,ind) = (data{2}.delta_v(2,ind-1) + data{2}.delta_v(2,ind+1))/2;





% plot act_velocities vs time for each dataset into a single plot
ylims = {[1.0, 2.2], [0.7, 1.6]};
f = figure;
f.Position = [215 603 619 659];
t = tiledlayout(2,1, tiled_options{:});
for j = 1:2
    ax = nexttile;
    hold on;
    plot_handles = gobjects(1, length(comparison_select)+1); % Preallocate array for plot handles
    for i = 1:length(comparison_select)
        this_vel = data{i}.act_vel_data(j,:);
        plot_handles(i) = plot(data{i}.act_vel_time, this_vel, plot_options{:}, 'DisplayName', data_label(i));
    end
    for i=1:1
        % plot reference velocity
        plot_handles(end) = plot(data{i}.ref_vel_time, data{i}.ref_vel_data(j,:), plot_options{:}, 'DisplayName', 'Reference', 'LineStyle', '--', 'Color', 'black');
        title(velocity_titles(j), title_options{:});
        grid on;
        xlabel(time_label, label_options{:});
        ylabel(velocity_units(j), label_options{:});
    end
    if j == 1
        legend(plot_handles(1:4), legend_options{:}); % Reorder legend entries
    end
    ylim(ylims{j});
    xlim([0, min(data{1}.act_vel_time(end), data{2}.act_vel_time(end))]);
    uistack(findobj(gca, 'DisplayName', 'AVTC'), 'top');
    uistack(findobj(gca, 'DisplayName', 'Reference'), 'top');
end

% save the plot
if save(1)
    saveFigure(f, image_path, file_prefix, 'velocities');
end

% plot delta velocities
ylims = {[-0.4, 0.6], [-0.4, 0.3]};
f = figure;
f.Position = [274 687 618 556];
t = tiledlayout(2,1, tiled_options{:});
for j = 1:2
    ax = nexttile;
    hold on;
    plot_handles = gobjects(1, length(comparison_select)+1); % Preallocate array for plot handles
    for i = 1:length(comparison_select)
        plot_handles(i) = plot(data{i}.ref_vel_time, -data{i}.delta_v(j,:), 'DisplayName', data_label(i), plot_options{:});
        if i == 1
            plot_handles(4) = yline(0, plot_options{:}, 'LineStyle', '--', 'Color', 'black', 'DisplayName', 'Zero Error');
            title(delta_v_titles(j), title_options{:});
            grid on;
            xlabel(time_label, label_options{:});
            ylabel(velocity_units(j), label_options{:});
        end
    end
    ylim(ylims{j});
    xlim([0, min(data{1}.act_vel_time(end), data{2}.act_vel_time(end))]);
    if j == 1
        legend(plot_handles(1:4), legend_options{:}); % Reorder legend entries
    end
end
uistack(findobj(gca, 'DisplayName', 'AVTC'), 'top');
uistack(findobj(gca, 'DisplayName', 'Reference'), 'top');

% save the plot
if save
    saveFigure(f, image_path, file_prefix, 'delta_velocities');
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
    saveFigure(f, image_path, file_prefix, 'adaptive_params_norm')
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
    saveFigure(f, image_path, file_prefix, 'adaptive_params')

end


function saveFigure(f, image_path, file_prefix, file_name)
    f.PaperPositionMode = 'manual';
    f.PaperUnits = 'points';
    f.PaperSize = f.Position(3:4);
    f.PaperPosition = [0 0 f.Position(3:4)];
    print (f, fullfile(image_path, strcat(file_prefix, file_name, '.pdf')), '-dpdf', '-r0');
end
