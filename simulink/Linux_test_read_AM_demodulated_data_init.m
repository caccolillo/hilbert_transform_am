clc;
clear all;

% Read integer samples from file
demod_audio_raw = load('output_data.txt');  % Nx1 vector of integers

% Convert from ap_fixed<16,4> to double
% ap_fixed<16,4> -> scale = 2^(16-4) = 4096
SCALE = 4096.0;
demod_audio_scaled = double(demod_audio_raw) / SCALE;

% Sample rate
sample_rate = 1470;   % Hz
sample_rate_int = round(sample_rate);

num_samples = length(demod_audio_scaled);

% Normalize to [-1, 1]
audio_normalized = demod_audio_scaled / max(abs(demod_audio_scaled));

% Save WAV
audiowrite('demod_output.wav', audio_normalized, sample_rate_int);

fprintf('WAV file saved as demod_output.wav\n');
fprintf('Sample rate: %d Hz\n', sample_rate_int);
fprintf('Duration: %.2f seconds\n', num_samples / sample_rate_int);
