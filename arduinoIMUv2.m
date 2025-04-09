 % Sunita Agarwala
 % 13 Feb 2025
 % arduinoIMUv2.m

 % The objective of this script is to read data directly from Arduino IDE
 % serial port (without using an arduino object in MATLAB. The hope is that
 % this will increase the sampling rate. 

 % This code was able to read acceleration (only) data from the BN055 IMU
 % for 30 s (detecting both normal motion and impacts). Then in plots in
 % the time domain, performs a FFT, and also plots the frequency content

 % As of 02/24/25 this code is also able to classify experimental trials as
 % either "walking" or "impact" based on a 400 Hz at amp = 1.0 cutoff freq

 clc
 close all 
 clear

 %% Declarations
% Define serial port
port = "/dev/cu.usbmodem1101"; 
baudRate = 115200;
s = serialport(port, baudRate);
% s.Timeout = 2; % Increase timeout to allow data to be read
% s.InputBufferSize = 10000; % Increase buffer size to prevent data loss

dataValues = []; % Store numerical values
timestamps = []; % Store timestamps

%% Read Data & Display for "10" sec - No plotting

tic; % Start timer
while toc < 10 % Run for 30 seconds (modify as needed)

    % Read serial data
    data = readline(s);
    values = str2double(strsplit(strtrim(data), ','));

    % % Check if data is empty or invalid
    % if isempty(data)
    %     warning("Empty data received, skipping...");
    %     continue;
    % end

    % Store data
    if ~isnan(values) % Ensure valid number
        dataValues = [dataValues; values]; % Append to array
        timestamps = [timestamps; toc]; % Store time
    end

    disp(data)

end

%% Data Post Processing
% dataValues = [x, y, z]
% dataValues = dataValues*(3.3/1023); % convert to 3.3 V scale
% netAccVector = sqrt(dataValues(:,1).^2 + dataValues(:,2).^2 + dataValues(:,3).^2)


%% Time Course Plotting
figure(1)
plot(timestamps,dataValues(:,1))
hold on
plot(timestamps,dataValues(:,2))
hold on
plot(timestamps,dataValues(:,3))
xlabel('Time (s)')
ylabel('Acceleration (V)')
% ylim([min(dataValues) max(dataValues)])
xlim([min(timestamps),max(timestamps)])
title('Time Course Acceleration Data')
legend('x-axis','y-axis','z-axis','location','southeast')


%% Filtering & FFT

% Variable declarations
L = length(timestamps);
Fs = 1000; % sampling frequency

% % nyquistFreq = Fs/2;
% % cutoffFreq = 5/nyquistFreq; % cutoff freq for LP 5 Hz
% % [b,a] = butter(4,cutoffFreq); % returns Transfer Function [b = numerator coeff, a = denominator coeff]
% % filtX = filtfilt(b,a,dataValues(:,1));
% % 

% x-axis
fft_X = fft(dataValues(:,1));
P2_X = abs(fft_X/L);
P1_X = P2_X(1:L/2+1);
P1_X(2:end-1) = 2*P1_X(2:end-1);

% y-axis
fft_Y = fft(dataValues(:,2));
P2_Y = abs(fft_Y/L);
P1_Y = P2_Y(1:L/2+1);
P1_Y(2:end-1) = 2*P1_Y(2:end-1);

% z-axis
fft_Z = fft(dataValues(:,3));
P2_Z = abs(fft_Z/L);
P1_Z = P2_Z(1:L/2+1);
P1_Z(2:end-1) = 2*P1_Z(2:end-1);

f = Fs/L*((0:(L/2)));
totalFreq = P1_X + P1_Y + P1_Z;

%% Plot Time Course & Frequency Data
figure(2)
plot(f,P1_X)
hold on
plot(f,P1_Y)
hold on
plot(f,P1_Z)
hold on
plot(f,totalFreq,'LineWidth',2)
xlabel('Frequency (Hz)')
ylabel('Amplitude')
title('Freq Content Acceleration Data (All Directions)')
legend('X','Y','Z','Total')

%% Classify trial
% Find 400 Hz frequency and figure out its amplitude - if it is above 1,
% classify as an "impact" trial
% Example frequency and amplitude vectors

% Define target frequency and amplitude threshold
target_freq = 400;
amplitude_threshold = .5;

% Find the index of the target frequency in freqVector
idx = find(f >= target_freq, 1);

% Check if the frequency exists and its amplitude is greater than 1
if ~isempty(idx(1)) && totalFreq(idx) > amplitude_threshold
    disp('Impact Detected');
else
    disp('No Impact Detected');
end



