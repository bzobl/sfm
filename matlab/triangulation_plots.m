%% Plot Triangulation times
fig = 1;

font_size = 19;
font_size_tick = 17;
line_width = 2;

%% Allocation time of GPU memory
f = figure(fig);
clf;
subplot(2, 1, 1);
hold on; grid on;

plot(n_keypoints, time_gpu_create_copied, 'k', 'LineWidth', line_width);
plot(n_keypoints, time_gpu_create_mapped, 'k--', 'LineWidth', line_width);

set(gca, 'FontSize',font_size_tick);
legend('global memory', 'zero-copy memory', 'Location', 'southeast');
title('Time for memory allocation', 'FontSize', font_size);
xlabel('number of keypoints', 'FontSize', font_size);
ylabel('time [ms]', 'FontSize', font_size);

% CONSTANT MEMORY
subplot(2, 1, 2);
hold on; grid on;

plot(n_keypoints, time_gpu_create_const_copied, 'k', 'LineWidth', line_width);
plot(n_keypoints, time_gpu_create_const_mapped, 'k--', 'LineWidth', line_width);

set(gca, 'FontSize',font_size_tick);
legend('global memory', 'zero-copy memory', 'Location', 'southeast');
title('Time for memory allocation (using constant memory)', 'FontSize', font_size);
xlabel('number of keypoints', 'FontSize', font_size);
ylabel('time [ms]', 'FontSize', font_size);

fig = fig + 1;

%% Processing time 
figure(fig);
clf;
subplot(2, 1, 1);
hold on; grid on;

plot(n_keypoints, time_cpu_process_copied, 'b', 'LineWidth', line_width);
plot(n_keypoints, time_gpu_process_copied, 'k', 'LineWidth', line_width);
plot(n_keypoints, time_gpu_process_mapped, 'k--', 'LineWidth', line_width);

set(gca, 'FontSize',font_size_tick);
legend('CPU', 'GPU', 'GPU (zero-copy)', 'Location', 'northwest');
title('Time for triangulation', 'FontSize', font_size);
xlabel('number of keypoints', 'FontSize', font_size);
ylabel('time [ms]', 'FontSize', font_size);

% CONSTANT MEMORY
subplot(2, 1, 2);
hold on; grid on;

plot(n_keypoints, time_cpu_process_const_copied, 'b', 'LineWidth', line_width);
plot(n_keypoints, time_gpu_process_const_copied, 'k', 'LineWidth', line_width);
plot(n_keypoints, time_gpu_process_const_mapped, 'k--', 'LineWidth', line_width);

set(gca, 'FontSize',font_size_tick);
legend('CPU', 'GPU', 'GPU (zero-copy)', 'Location', 'northwest');
title('Time for triangulation (using constant memory)', 'FontSize', font_size);
xlabel('number of keypoints', 'FontSize', font_size);
ylabel('time [ms]', 'FontSize', font_size);

fig = fig + 1;

%% Speedup

speedup_copied = time_cpu_process_copied ./ time_gpu_process_copied;
speedup_mapped = time_cpu_process_copied ./ time_gpu_process_mapped;
speedup_const_copied = time_cpu_process_copied ./ time_gpu_process_const_copied;
speedup_const_mapped = time_cpu_process_copied ./ time_gpu_process_const_mapped;

figure(fig);
clf;
hold on; grid on;

plot(n_keypoints, speedup_copied, 'b', 'LineWidth', line_width);
plot(n_keypoints, speedup_mapped, 'b--', 'LineWidth', line_width);
plot(n_keypoints, speedup_const_copied, 'k', 'LineWidth', line_width);
plot(n_keypoints, speedup_const_mapped, 'k--', 'LineWidth', line_width);

set(gca, 'FontSize',font_size_tick);
legend('global / no constant memory', 'zero-copy / no constant memory', 'global / constant memory', 'zero-copy / constant memory', 'Location', 'southeast');
title('Speedup for triangulation', 'FontSize', font_size);
xlabel('number of keypoints', 'FontSize', font_size);
ylabel('time [ms]', 'FontSize', font_size);

fig = fig + 1;