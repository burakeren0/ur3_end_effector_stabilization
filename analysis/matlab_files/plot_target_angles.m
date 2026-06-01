% plot_target_angles.m
% Read joint angle CSV and plot 6 target/actual angle curves for UR3 robot.

csvFile = 'target_actual_angles.csv';
if ~isfile(csvFile)
    csvFile = 'target_angles_log.csv';
end

if ~isfile(csvFile)
    error('Dosya bulunamadı: %s', csvFile);
end

T = readtable(csvFile);

joint_names = {
    'shoulder_pan_joint (q1)',
    'shoulder_lift_joint (q2)',
    'elbow_joint (q3)',
    'wrist_1_joint (q4)',
    'wrist_2_joint (q5)',
    'wrist_3_joint (q6)'
};

legacy_target_cols = arrayfun(@(i) sprintf('q%d', i), 1:6, 'UniformOutput', false);
new_target_cols = strcat('target_', {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'});
new_actual_cols = strcat('actual_', {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'});

has_legacy = all(ismember(['time',legacy_target_cols], T.Properties.VariableNames));
has_new = all(ismember(['time', new_target_cols, new_actual_cols], T.Properties.VariableNames));

if ~has_legacy && ~has_new
    error('CSV dosyası beklenen sütun adlarına sahip değil. Beklenen: time + q1..q6 veya time + target_*/actual_*');
end

fig = figure('Name', 'UR3 Target vs Actual Joint Angles', 'NumberTitle', 'off', 'Visible', 'on');

for joint = 1:6
    ax(joint) = subplot(6,1,joint); %#ok<AGROW>
    hold on;

    if has_new
        target_data = T.(new_target_cols{joint});
        actual_data = T.(new_actual_cols{joint});
        plot(T.time, target_data, 'LineWidth', 1.5, 'Color', [0.0 0.45 0.74]);
        plot(T.time, actual_data, '--', 'LineWidth', 1.5, 'Color', [0.85 0.33 0.10]);
        legend({'Target', 'Actual'}, 'Location', 'best');
    else
        target_data = T.(legacy_target_cols{joint});
        plot(T.time, target_data, 'LineWidth', 1.5, 'Color', [0.2 0.4 0.8]);
    end

    grid on;
    ylabel(sprintf('%s\n(rad)', joint_names{joint}), 'Interpreter', 'none');
    if joint == 1
        title('UR3 Target vs Actual Joint Angles', 'FontSize', 12, 'FontWeight', 'bold');
    end
    if joint == 6
        xlabel('Zaman (s)');
    end
end

try
    linkaxes(ax, 'x');
catch ME
    warning('linkaxes uygulanamadı: %s', ME.message);
end

% q6 için y-axis skalasını genişlet
ylim(ax(6), [-0.1 0.1]);

% Alternatif olarak grafik kaydetmek isterseniz aşağıdaki satırı açabilirsiniz:
% saveas(fig, 'ur3_target_actual_angles_plot.png');
