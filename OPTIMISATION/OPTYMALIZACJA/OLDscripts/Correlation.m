% Load or generate example data (replace with your actual datasets)
% Assume reality_time, reality_position, reality_velocity are from reality
% and sim_time, sim_position, sim_velocity are from simulation

% Example data generation (replace with actual data loading)
reality_time = linspace(0, 20, 1000);  % Time in seconds
reality_position = sin(reality_time);  % Position data
reality_velocity = cos(reality_time);  % Velocity data

sim_time = linspace(10, 30, 1000);  % Time in seconds (simulated time shifted)
sim_position = sin(sim_time - 5);   % Simulated position data (shifted)
sim_velocity = cos(sim_time - 5);  % Simulated velocity data (shifted)

% Plot original datasets
figure;
subplot(2, 1, 1);
plot(reality_time, reality_position, 'b', sim_time, sim_position, 'r');
xlabel('Time (s)');
ylabel('Position');
title('Original Datasets: Reality (blue) vs. Simulation (red)');
legend('Reality', 'Simulation');

subplot(2, 1, 2);
plot(reality_time, reality_velocity, 'b', sim_time, sim_velocity, 'r');
xlabel('Time (s)');
ylabel('Velocity');
title('Velocity: Reality (blue) vs. Simulation (red)');
legend('Reality', 'Simulation');

% Compute cross-correlation of velocity signals
[correlation, lag] = xcorr(reality_velocity, sim_velocity);

% Find the time shift that maximizes correlation
[~, idx] = max(abs(correlation));
optimal_lag = lag(idx);

% Shift simulation data to align with reality data
aligned_sim_time = sim_time - optimal_lag;  % Apply time shift

% Interpolate simulation data onto reality time points
aligned_sim_position = interp1(aligned_sim_time, sim_position, reality_time);
aligned_sim_velocity = interp1(aligned_sim_time, sim_velocity, reality_time);

% Plot aligned datasets
figure;
subplot(2, 1, 1);
plot(reality_time, reality_position, 'b', reality_time, aligned_sim_position, 'r');
xlabel('Time (s)');
ylabel('Position');
title('Aligned Datasets: Reality (blue) vs. Aligned Simulation (red)');
legend('Reality', 'Aligned Simulation');

subplot(2, 1, 2);
plot(reality_time, reality_velocity, 'b', reality_time, aligned_sim_velocity, 'r');
xlabel('Time (s)');
ylabel('Velocity');
title('Velocity: Reality (blue) vs. Aligned Simulation (red)');
legend('Reality', 'Aligned Simulation');
