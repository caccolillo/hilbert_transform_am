clear all;

% Read integer samples from file
demod_audio_raw = load('demod_samples.txt');  % Nx1 vector of integers

% Convert from ap_fixed<16,4> to double
SCALE = 4096.0;
demod_audio_scaled = double(demod_audio_raw) / SCALE;

% Create time vector based on sample rate
sample_rate = 6.66607e3;  % Sample rate in Hz
sample_rate_int = round(sample_rate);  % Convert to integer (6666 Hz)
num_samples = length(demod_audio_scaled);
time_vector = (0:num_samples-1)' / sample_rate;  % Time in seconds

% Create structure in the format required by From Workspace block
demod_audio.time = time_vector;
demod_audio.signals.values = demod_audio_scaled;
demod_audio.signals.dimensions = 1;  % Scalar signal (1D)

% Save as WAV file
% Normalize to range [-1, 1] if needed
audio_normalized = demod_audio_scaled / max(abs(demod_audio_scaled));
audiowrite('demod_output.wav', audio_normalized, sample_rate_int);

fprintf('WAV file saved as demod_output.wav\n');
fprintf('Sample rate: %d Hz\n', sample_rate_int);
fprintf('Duration: %.2f seconds\n', num_samples / sample_rate_int);