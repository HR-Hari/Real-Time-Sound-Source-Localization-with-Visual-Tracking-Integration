%% AoA Estimation from Two Mic Recordings
pkg load signal

% Constants
Fs = 44100;           % Sampling frequency
v = 343;              % Speed of sound (m/s)
d = 0.3;              % Mic spacing (meters)

%% 1. Load Two Microphone Recordings
[file1, path1] = uigetfile({'*.wav'}, 'Select Mic1 WAV File');
if isequal(file1, 0)
    error('No mic1 file selected. Exiting.');
end

[file2, path2] = uigetfile({'*.wav'}, 'Select Mic2 WAV File');
if isequal(file2, 0)
    error('No mic2 file selected. Exiting.');
end

[x1, Fs1] = audioread(fullfile(path1, file1));
[x2, Fs2] = audioread(fullfile(path2, file2));

if Fs1 ~= Fs2
    error('Sampling rates of the two files do not match.');
end

% Resample to expected Fs if needed
if Fs1 ~= Fs
    x1 = resample(x1(:,1), Fs, Fs1);
    x2 = resample(x2(:,1), Fs, Fs2);
else
    x1 = x1(:,1); % Convert to mono
    x2 = x2(:,1);
end

% Truncate to equal length
N = min(length(x1), length(x2));
x1 = x1(1:N);
x2 = x2(1:N);

%% 2. Estimate TDoA from Cross-Correlation
[cc, lags] = xcorr(x2, x1);
[~, idx] = max(abs(cc));
estimated_samples = lags(idx);
tdoa_est = estimated_samples / Fs;

fprintf("Estimated TDoA: %.8f seconds\n", tdoa_est);

%% 3. Estimate AoA
arg = (tdoa_est * v) / d;
arg = max(min(arg, 1), -1);  % Clamp to [-1, 1]
angle_est = asind(arg);

fprintf("Estimated Angle of Arrival: %.2f degrees\n", angle_est);

%% 4. Plot Cross-Correlation
figure;
plot(lags / Fs, cc, 'k');
xlabel('Lag (seconds)');
ylabel('Cross-Correlation');
title('Cross-Correlation between Mic2 and Mic1');
grid on;

%% 5. Plot Microphone Layout and Estimated AoA
mic1 = [-d/2, 0];
mic2 = [d/2, 0];
midpoint = [0, 0];

r = 0.3;  % Arrow length
theta_rad = deg2rad(angle_est);
source_dir = [r * sind(angle_est), r * cosd(angle_est)];

figure;
plot([mic1(1), mic2(1)], [mic1(2), mic2(2)], 'ko-', 'LineWidth', 2); hold on;
quiver(midpoint(1), midpoint(2), source_dir(1), source_dir(2), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.05);
text(mic1(1)-0.01, mic1(2)-0.01, 'Mic1');
text(mic2(1)+0.01, mic2(2)-0.01, 'Mic2');
text(0.05, 0.15, sprintf('AoA = %.1f¡ã', angle_est));
axis equal;
xlabel('X (meters)');
ylabel('Y (meters)');
title('Microphone Layout and Estimated AoA');
grid on;

