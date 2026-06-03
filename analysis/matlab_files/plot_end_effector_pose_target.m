% plot_end_effector_pose_target.m
% Plots every UR3 end-effector target/actual CSV file whose name ends with ee.csv.

clear; % Clear old workspace variables before this analysis run.
close all; % Close old figure windows so only the new plots remain.
clc; % Clear the command window for this run.

script_file = mfilename('fullpath'); % Find this script path independently of the current MATLAB folder.
if isempty(script_file) % Handle selected-line execution where mfilename can be empty.
    script_dir = pwd; % Use the current MATLAB folder as the script folder fallback.
else % Handle normal execution from a saved .m file.
    script_dir = fileparts(script_file); % Resolve the analysis/matlab_files folder.
end % Finish script folder detection.

repo_analysis_dir = '/home/taylan/ur3_ws/src/ur3_end_effector_stabilization/analysis'; % Try the real repo analysis folder first.
required_cols = {'time', ... % Required time column.
    'target_x_base', 'actual_x_base', ... % Required base-frame X target/actual columns.
    'target_y_base', 'actual_y_base', ... % Required base-frame Y target/actual columns.
    'target_z_base', 'actual_z_base', ... % Required base-frame Z target/actual columns.
    'target_roll_world_rad', 'actual_roll_world_rad', ... % Required world-frame roll target/actual columns.
    'target_pitch_world_rad', 'actual_pitch_world_rad', ... % Required world-frame pitch target/actual columns.
    'target_yaw_world_rad', 'actual_yaw_world_rad'}; % Required world-frame yaw target/actual columns.
analysis_candidates = {repo_analysis_dir, ... % Candidate 1: source checkout analysis folder on this machine.
    fullfile(script_dir, '..'), ... % Candidate 2: parent folder when script is under analysis/matlab_files.
    fullfile(script_dir, 'analysis'), ... % Candidate 3: analysis subfolder when script is copied to repo root.
    fullfile(pwd, 'analysis'), ... % Candidate 4: analysis subfolder under the current MATLAB folder.
    pwd}; % Candidate 5: current MATLAB folder when it is already the CSV folder.
analysis_dir = find_ee_csv_analysis_dir(analysis_candidates, required_cols); % Select the first folder containing a compatible file ending with ee.csv.
if isempty(analysis_dir) % Stop with a clear error when no matching CSV exists.
    searched_dirs_text = sprintf('  %s\n', analysis_candidates{:}); % Format the searched directories for the error text.
    error('Compatible end-effector CSV file ending with ee.csv was not found. Searched folders:\n%s', searched_dirs_text); % Report missing experiment output.
end % Finish analysis folder selection.

output_dir = fullfile(analysis_dir, 'matlab_files', 'end_effector_pose_figures'); % Store PNG files under the selected analysis tree.
if ~exist(output_dir, 'dir') % Check whether the figure output folder exists.
    mkdir(output_dir); % Create the figure output folder.
end % Finish output folder preparation.

show_figures = true; % true opens MATLAB figure windows.
save_png_files = true; % true saves one PNG per compatible CSV file.
plot_angles_in_degrees = true; % true converts RPY plots from radians to degrees.
wrap_actual_angles_near_target = true; % true removes visual +/-pi jumps around the target angle.

plot_specs = { ... % Define the six target/actual plots.
    'target_x_base', 'actual_x_base', 'X position - base frame', 'm', false; ... % Plot 1: base-frame X position.
    'target_y_base', 'actual_y_base', 'Y position - base frame', 'm', false; ... % Plot 2: base-frame Y position.
    'target_z_base', 'actual_z_base', 'Z position - base frame', 'm', false; ... % Plot 3: base-frame Z position.
    'target_roll_world_rad', 'actual_roll_world_rad', 'Roll - world frame', 'rad', true; ... % Plot 4: world-frame roll angle.
    'target_pitch_world_rad', 'actual_pitch_world_rad', 'Pitch - world frame', 'rad', true; ... % Plot 5: world-frame pitch angle.
    'target_yaw_world_rad', 'actual_yaw_world_rad', 'Yaw - world frame', 'rad', true}; % Plot 6: world-frame yaw angle.

csv_files = dir(fullfile(analysis_dir, '*ee.csv')); % List only experiment CSV files whose names end with ee.csv.
if isempty(csv_files) % Stop when the selected analysis folder contains no matching experiment files.
    error('No CSV file ending with ee.csv was found in: %s', analysis_dir); % Report the exact folder without producing empty figures.
end % Finish ee.csv file existence check.
valid_file_count = 0; % Count ee.csv files that match the expected logger schema.
for file_index = 1:numel(csv_files) % Process each ee.csv file separately.
    csv_path = fullfile(csv_files(file_index).folder, csv_files(file_index).name); % Build the full CSV path.
    T = readtable(csv_path); % Read the CSV as a MATLAB table.

    if ~has_required_columns(T, required_cols) % Check whether this CSV was produced by the end-effector logger.
        fprintf('Skipped: %s ends with ee.csv but does not contain the expected end-effector target/actual columns.\n', csv_files(file_index).name); % Report malformed experiment CSV files.
        continue; % Move to the next CSV without plotting this one.
    end % Finish schema compatibility check.

    valid_file_count = valid_file_count + 1; % Increment the compatible CSV counter.
    [~, csv_name, ~] = fileparts(csv_files(file_index).name); % Extract the file name without extension.
    [controller_name, world_name] = experiment_labels(csv_name); % Derive readable controller and world names from the CSV file name.
    display_name = [controller_name, ' - ', world_name]; % Build the experiment label shown in the figure title.
    time_s = T.time - T.time(1); % Start the time axis at zero seconds.

    fig_visibility = 'on'; % Show figures by default.
    if ~show_figures % Honor the option to suppress visible figures.
        fig_visibility = 'off'; % Generate figures in the background.
    end % Finish figure visibility selection.

    fig = figure('Name', ['UR3 end-effector target vs actual - ', display_name], 'NumberTitle', 'off', 'Visible', fig_visibility, 'Units', 'normalized', 'OuterPosition', [0.0 0.0 1.0 1.0]); % Open each figure at full-screen size so long left-side labels have enough space.
    set(fig, 'PaperPositionMode', 'auto'); % Preserve the full-screen figure dimensions when exporting the PNG file.
    ax = zeros(6, 1); % Store subplot axes for linked x-axis control.

    for plot_index = 1:6 % Draw the six requested plots.
        ax(plot_index) = subplot(6, 1, plot_index); % Create the subplot for this signal.
        hold(ax(plot_index), 'on'); % Keep target and actual lines on the same axes.

        target_col = plot_specs{plot_index, 1}; % Read the target column name.
        actual_col = plot_specs{plot_index, 2}; % Read the actual column name.
        signal_title = plot_specs{plot_index, 3}; % Read the y-axis signal name.
        unit_label = plot_specs{plot_index, 4}; % Read the default unit label.
        is_angle_signal = plot_specs{plot_index, 5}; % Read whether this signal is angular.

        target_data = T.(target_col); % Extract target samples.
        actual_data = T.(actual_col); % Extract actual samples.

        if is_angle_signal && wrap_actual_angles_near_target % Apply wrapping only to angle plots when requested.
            actual_data = target_data + atan2(sin(actual_data - target_data), cos(actual_data - target_data)); % Wrap actual to the equivalent angle nearest target.
        end % Finish angle wrapping.

        if is_angle_signal && plot_angles_in_degrees % Convert angle plots to degrees when requested.
            target_data = target_data * 180.0 / pi; % Convert target radians to degrees.
            actual_data = actual_data * 180.0 / pi; % Convert actual radians to degrees.
            unit_label = 'deg'; % Update the y-axis unit label.
        end % Finish angle unit conversion.

        target_line = plot(ax(plot_index), time_s, target_data, '--', 'LineWidth', 1.4, 'Color', [0.85 0.0 0.0]); % Draw the target line first so it stays below the actual line.
        actual_line = plot(ax(plot_index), time_s, actual_data, 'LineWidth', 1.4, 'Color', [0.0 0.45 0.74]); % Draw the actual line last so it remains visible above the target line.
        grid(ax(plot_index), 'on'); % Enable grid lines for readability.
        ylabel(ax(plot_index), sprintf('%s\n(%s)', signal_title, unit_label), 'Interpreter', 'none'); % Label each subplot with signal and unit.

        if plot_index == 1 % Put the title and legend on the first subplot only.
            title(ax(plot_index), ['UR3 End-Effector Target vs Actual - ', display_name], 'Interpreter', 'none'); % Add controller and world names to the figure title.
            legend(ax(plot_index), [actual_line, target_line], {'Actual', 'Target'}, 'Location', 'best'); % Keep actual first in the legend while drawing it above target.
        end % Finish first-subplot formatting.

        if plot_index == 6 % Put the x label only on the bottom subplot.
            xlabel(ax(plot_index), 'Time (s)'); % Label the shared time axis.
        end % Finish x-axis labeling.
    end % Finish all six subplots.

    linkaxes(ax, 'x'); % Link the subplot time axes.
    drawnow; % Complete the full-screen subplot layout before exporting the figure.

    if save_png_files % Save the figure when PNG output is enabled.
        png_path = fullfile(output_dir, [safe_file_name(controller_name), '_', safe_file_name(world_name), '_end_effector_pose_target_actual.png']); % Build a PNG name containing the controller and world names.
        exportgraphics(fig, png_path, 'Resolution', 150); % Export the full-screen figure at a readable PNG resolution.
        fprintf('Saved: %s\n', png_path); % Report the saved PNG path.
    end % Finish PNG saving.
end % Finish processing all CSV files.

if valid_file_count == 0 % Treat a folder with no compatible ee.csv files as an error.
    error('No CSV file ending with ee.csv contains the expected end-effector columns in: %s', analysis_dir); % Stop when compatible experiment data is missing.
end % Finish compatible file count check.

fprintf('Created separate end-effector target/actual plots for %d ee.csv file(s).\n', valid_file_count); % Report successful per-experiment plotting.

function ok = has_required_columns(T, required_cols) % Return whether the table has every required column.
    ok = all(ismember(required_cols, T.Properties.VariableNames)); % Check all required column names.
end % Finish required-column helper.

function selected_dir = find_ee_csv_analysis_dir(candidate_dirs, required_cols) % Find the first candidate folder containing a compatible file ending with ee.csv.
    selected_dir = ''; % Start with no selected folder.
    for candidate_index = 1:numel(candidate_dirs) % Visit each candidate folder.
        candidate_dir = candidate_dirs{candidate_index}; % Read the current candidate folder.
        if exist(candidate_dir, 'dir') ~= 7 % Skip folders that do not exist.
            continue; % Move to the next candidate folder.
        end % Finish folder existence check.
        candidate_csv_files = dir(fullfile(candidate_dir, '*ee.csv')); % List only experiment CSV files ending with ee.csv.
        for candidate_file_index = 1:numel(candidate_csv_files) % Inspect each candidate ee.csv file.
            candidate_csv_path = fullfile(candidate_csv_files(candidate_file_index).folder, candidate_csv_files(candidate_file_index).name); % Build the candidate CSV path.
            candidate_table = readtable(candidate_csv_path); % Read the candidate CSV for column inspection.
            if has_required_columns(candidate_table, required_cols) % Select the folder if this CSV has the logger schema.
                selected_dir = candidate_dir; % Store the matching folder.
                return; % Stop after the first matching folder.
            end % Finish candidate CSV schema check.
        end % Finish CSV inspection for this folder.
    end % Finish candidate folder search.
end % Finish analysis-folder helper.

function [controller_name, world_name] = experiment_labels(csv_name) % Convert the experiment CSV name into readable controller and world labels.
    lower_name = lower(csv_name); % Use a lowercase copy for case-insensitive filename matching.
    if contains(lower_name, 'ct_pi_pd') % Match the most specific computed-torque PI+PD controller token first.
        controller_name = 'Computed Torque + PI+PD'; % Use the readable computed-torque PI+PD controller name.
    elseif contains(lower_name, 'ct_pid') % Match the computed-torque PID controller token before the generic PID token.
        controller_name = 'Computed Torque + PID'; % Use the readable computed-torque PID controller name.
    elseif contains(lower_name, 'ct_pd') % Match the computed-torque PD controller token before the generic PD token.
        controller_name = 'Computed Torque + PD'; % Use the readable computed-torque PD controller name.
    elseif contains(lower_name, 'pi_pd') % Match the standalone PI+PD controller token.
        controller_name = 'PI+PD'; % Use the readable PI+PD controller name.
    elseif contains(lower_name, 'pid') % Match the standalone PID controller token.
        controller_name = 'PID'; % Use the readable PID controller name.
    elseif contains(lower_name, 'pd') % Match the standalone PD controller token.
        controller_name = 'PD'; % Use the readable PD controller name.
    else % Handle future files whose controller token is not recognized.
        controller_name = strrep(csv_name, '_', ' '); % Fall back to a readable form of the original CSV name.
    end % Finish controller-name selection.

    if contains(lower_name, 'bumpy') % Detect data recorded in the bumpy map.
        world_name = 'bumpy.world'; % Use the exact bumpy world file name in figure labels.
    elseif contains(lower_name, 'extreme') % Detect data recorded in the extreme disturbance map.
        world_name = 'extreme_disturbance.world'; % Use the exact extreme disturbance world file name in figure labels.
    else % Handle future files whose map token is not recognized.
        world_name = 'unknown world'; % Make the missing map classification explicit in the figure label.
    end % Finish world-name selection.
end % Finish experiment-label helper.

function safe_name = safe_file_name(name_text) % Make a file name safe for PNG output.
    safe_name = regexprep(name_text, '[^A-Za-z0-9_-]', '_'); % Replace unsafe characters with underscores.
    safe_name = regexprep(safe_name, '_+', '_'); % Collapse repeated underscores.
    safe_name = regexprep(safe_name, '^_|_$', ''); % Remove leading or trailing underscores.
end % Finish safe-name helper.
