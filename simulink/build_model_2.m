% Simulink Envelope Detection using Hilbert Transform
% This script creates a Simulink model for envelope detection
% Compatible with MATLAB 2021b
bdclose all

clear; close all; clc;

%% Model Parameters
model_name = 'envelope_detection_hilbert';
Fs = 2e6;               % Sampling frequency (Hz) - 2 MHz for 500 kHz carrier
Ts = 1/Fs;              % Sample time

% Hilbert transformer parameters
N = 32;                 % Filter length (32-point)
delay = N/2;            % Group delay (16 samples)

% Design Parks-McClellan Hilbert transformer
hilbert_coeffs = firpm(N-1, [0.1 0.9], [1 1], 'hilbert');
hilbert_filter = dfilt.dffir(hilbert_coeffs);

% Lowpass filter for envelope smoothing
lpf_order = 50;
cutoff_freq = 5000;     % Hz
downsample_factor = 20;
lpf_coeffs = fir1(lpf_order, cutoff_freq/(Fs/downsample_factor/2), 'low');
lpf_filter = dfilt.dffir(lpf_coeffs);

%% Create Simulink Model
% Close model if already open
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Create new model
new_system(model_name);
open_system(model_name);

%% Add Blocks

% 1. Signal Source (Sine Wave for AM modulation envelope)
add_block('simulink/Sources/Sine Wave', [model_name '/Modulation']);
set_param([model_name '/Modulation'], ...
    'Amplitude', '0.8', ...
    'Frequency', '50', ...
    'SampleTime', num2str(Ts), ...
    'Position', [50 50 80 80]);

% 2. Bias for AM (adds DC offset)
add_block('simulink/Math Operations/Bias', [model_name '/Bias']);
set_param([model_name '/Bias'], ...
    'Bias', '1', ...
    'Position', [120 53 150 77]);

% 3. Carrier Signal (Sine Wave)
add_block('simulink/Sources/Sine Wave', [model_name '/Carrier']);
set_param([model_name '/Carrier'], ...
    'Amplitude', '1', ...
    'Frequency', '500000', ...
    'SampleTime', num2str(Ts), ...
    'Position', [50 150 80 180]);

% 4. Product (AM Modulation)
add_block('simulink/Math Operations/Product', [model_name '/AM_Modulator']);
set_param([model_name '/AM_Modulator'], ...
    'Position', [200 85 230 115]);

% 5. Hilbert Transform (Digital Filter Design block)
add_block('dsparch4/Digital Filter Design', [model_name '/Hilbert_FIR']);
%set_param([model_name '/Hilbert_FIR'], ...
%    'FilterObject', 'hilbert_filter', ...
%    'Position', [280 140 330 180]);

% 6. Delay for original signal
add_block('simulink/Discrete/Integer Delay', [model_name '/Signal_Delay']);
set_param([model_name '/Signal_Delay'], ...
    'NumDelays', num2str(delay), ...
    'Position', [280 80 330 120]);

% 7. Complex signal creation (Real-Imag to Complex)
add_block('simulink/Math Operations/Real-Imag to Complex', [model_name '/To_Complex']);
set_param([model_name '/To_Complex'], ...
    'Position', [380 95 410 130]);

% 8. Magnitude (Envelope extraction)
add_block('simulink/Math Operations/Complex to Magnitude-Angle', [model_name '/Get_Magnitude']);
set_param([model_name '/Get_Magnitude'], ...
    'Output', 'Magnitude', ...
    'Position', [460 100 490 130]);

% 9. Downsample
add_block('simulink/Signal Attributes/Rate Transition', [model_name '/Downsample']);
set_param([model_name '/Downsample'], ...
    'OutPortSampleTime', num2str(Ts*downsample_factor), ...
    'Position', [540 100 570 130]);

% 10. Lowpass Filter for smoothing (Digital Filter Design block)
add_block('dsparch4/Digital Filter Design', [model_name '/Smoothing_LPF']);
%set_param([model_name '/Smoothing_LPF'], ...
%    'FilterObject', 'lpf_filter', ...
%    'Position', [620 100 670 140]);

% 11. Scope for AM Signal
add_block('simulink/Sinks/Scope', [model_name '/AM_Signal_Scope']);
set_param([model_name '/AM_Signal_Scope'], ...
    'Position', [250 30 280 60]);

% 12. Scope for Envelope (Unsmoothed)
add_block('simulink/Sinks/Scope', [model_name '/Envelope_Raw_Scope']);
set_param([model_name '/Envelope_Raw_Scope'], ...
    'Position', [520 50 550 80]);

% 13. Scope for Final Envelope (Smoothed)
add_block('simulink/Sinks/Scope', [model_name '/Envelope_Final_Scope']);
set_param([model_name '/Envelope_Final_Scope'], ...
    'Position', [720 100 750 130]);

% 14. To Workspace blocks for analysis
add_block('simulink/Sinks/To Workspace', [model_name '/AM_Signal_Out']);
set_param([model_name '/AM_Signal_Out'], ...
    'VariableName', 'am_signal', ...
    'SaveFormat', 'Array', ...
    'Position', [250 200 310 230]);

add_block('simulink/Sinks/To Workspace', [model_name '/Envelope_Out']);
set_param([model_name '/Envelope_Out'], ...
    'VariableName', 'envelope', ...
    'SaveFormat', 'Array', ...
    'Position', [720 150 780 180]);

%% Connect Blocks

% Modulation path
add_line(model_name, 'Modulation/1', 'Bias/1');
add_line(model_name, 'Bias/1', 'AM_Modulator/1');

% Carrier path
add_line(model_name, 'Carrier/1', 'AM_Modulator/2');

% AM signal splits to: scope, delay, Hilbert filter, and workspace
add_line(model_name, 'AM_Modulator/1', 'AM_Signal_Scope/1');
add_line(model_name, 'AM_Modulator/1', 'Signal_Delay/1');
add_line(model_name, 'AM_Modulator/1', 'Hilbert_FIR/1');
add_line(model_name, 'AM_Modulator/1', 'AM_Signal_Out/1');

% Form complex signal
add_line(model_name, 'Signal_Delay/1', 'To_Complex/1');  % Real part
add_line(model_name, 'Hilbert_FIR/1', 'To_Complex/2');   % Imaginary part

% Extract envelope
add_line(model_name, 'To_Complex/1', 'Get_Magnitude/1');

% Envelope path
add_line(model_name, 'Get_Magnitude/1', 'Envelope_Raw_Scope/1');
add_line(model_name, 'Get_Magnitude/1', 'Downsample/1');

% Smooth and output
add_line(model_name, 'Downsample/1', 'Smoothing_LPF/1');
add_line(model_name, 'Smoothing_LPF/1', 'Envelope_Final_Scope/1');
add_line(model_name, 'Smoothing_LPF/1', 'Envelope_Out/1');

%% Configure Model Settings
set_param(model_name, 'StopTime', '0.01');  % 10 ms simulation time
set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'FixedStep', num2str(Ts));
set_param(model_name, 'Solver', 'FixedStepDiscrete');

%% Configure Scopes
% Configure AM Signal Scope
scope_config = get_param([model_name '/AM_Signal_Scope'], 'ScopeConfiguration');
scope_config.OpenAtSimulationStart = true;
scope_config.Name = 'AM Signal';

% Configure Raw Envelope Scope
scope_config = get_param([model_name '/Envelope_Raw_Scope'], 'ScopeConfiguration');
scope_config.OpenAtSimulationStart = true;
scope_config.Name = 'Raw Envelope';

% Configure Final Envelope Scope
scope_config = get_param([model_name '/Envelope_Final_Scope'], 'ScopeConfiguration');
scope_config.OpenAtSimulationStart = true;
scope_config.Name = 'Smoothed Envelope';

%% Auto-arrange blocks
Simulink.BlockDiagram.arrangeSystem(model_name);

%% Save Model
save_system(model_name);

%% Display Information
fprintf('Simulink model "%s" created successfully!\n', model_name);
fprintf('\nModel Parameters:\n');
fprintf('  Sampling frequency: %d Hz\n', Fs);
fprintf('  Hilbert FIR length: %d samples\n', N);
fprintf('  Group delay: %d samples (%.2f ms)\n', delay, delay/Fs*1000);
fprintf('  Downsample factor: %d\n', downsample_factor);
fprintf('  LPF cutoff: %d Hz\n', cutoff_freq);
fprintf('\nTo run the simulation:\n');
fprintf('  1. Click the Run button in Simulink\n');
fprintf('  2. View the three scopes to see:\n');
fprintf('     - AM Signal Scope: Original AM modulated signal\n');
fprintf('     - Raw Envelope Scope: Unsmoothed envelope\n');
fprintf('     - Smoothed Envelope Scope: Final processed envelope\n');
fprintf('  3. Data will be saved to workspace variables:\n');
fprintf('     - am_signal: Original AM signal\n');
fprintf('     - envelope: Final smoothed envelope\n');

%% Optional: Run simulation automatically
% Uncomment the next line to run simulation automatically
% sim(model_name);