% ANC_Simulation.m | Phases 1-4: LMS ANC via Simulink
% Author: Ojas Vaidya

clear; clc; close all;

fprintf('=== ANC Simulation  |  Phases 1–4 ===\n');

%% 1. Time setup
Fs       = 8000;                            
duration = 5;                               
t        = (0:1/Fs:duration-1/Fs)';        
N_total  = length(t);
fprintf('Fs = %d Hz  |  Duration = %d s  |  Samples = %d\n\n', ...
        Fs, duration, N_total);

%% 2. Clean voice (am modulated harmonics so it sounds more natural)
f_fund    = 300;          
am_rate   = 4;            
am_depth  = 0.3;          

envelope    = 1 - am_depth * sin(2*pi*am_rate*t);
clean_voice = envelope .* ( sin(2*pi*f_fund*t)         ...
                           + 0.5*sin(2*pi*2*f_fund*t)  ...
                           + 0.2*sin(2*pi*3*f_fund*t) );

%% 3. Ref mic signal (noise + basic mems thermal floor)
rng(42);
noise_variance   = 0.5;
x_clean          = sqrt(noise_variance) * randn(N_total, 1);  
mems_sigma       = 1e-4;
x                = x_clean + mems_sigma * randn(N_total, 1);  

%% 4. Primary path (200-tap exp RIR, approx 150ms rt60)
rir_len    = 200;                           
rir_tap    = (0:rir_len-1)';
decay_rate = 6.9 / (0.150 * Fs);           

room_impulse_response = exp(-decay_rate * rir_tap) .* randn(rir_len, 1);
room_impulse_response(1) = 1;              
room_impulse_response    = room_impulse_response / norm(room_impulse_response);

primary_noise = filter(room_impulse_response, 1, x_clean);
d             = clean_voice + primary_noise;   

%% 5. Causality check
[~, peak_idx]    = max(abs(room_impulse_response));
primary_delay_ms = (peak_idx-1) / Fs * 1000;
fprintf('Causality check:\n');
fprintf('  Primary path peak delay: %.2f ms  (%d samples)\n\n', ...
        primary_delay_ms, peak_idx-1);

%% 6. Sanity plots
figure('Name','Phase 2: Signal Sanity Check','NumberTitle','off');
subplot(3,1,1);
plot(t(1:500), clean_voice(1:500),'k','LineWidth',1);
title('Clean Voice (AM-Modulated Harmonics)'); ylabel('Amplitude'); grid on;
subplot(3,1,2);
plot(t(1:500), x(1:500),'Color',[0.5 0.5 0.5],'LineWidth',0.8);
title('Reference Mic x(n)  [Noise + MEMS Thermal]'); ylabel('Amplitude'); grid on;
subplot(3,1,3);
plot(t(1:500), d(1:500),'r','LineWidth',0.8);
title('Primary Mic d(n)  [Voice + Room Noise]');
xlabel('Time (s)'); ylabel('Amplitude'); grid on;

%% 7. Format data for Simulink
sim_d = [t, d];
sim_x = [t, x];

%% 8. Run simulink
disp('Running Simulink Model (ANC_model.slx)...');
out = sim('ANC_model');
disp('Simulink complete. Running Phase 4 analysis...');


%% Phase 4 - Analysis & Validation

%% 4.1 Extract output
if     exist('out','var')           && isprop(out,'cleaned_voice')
    e = out.cleaned_voice;
elseif exist('cleaned_voice','var') && isstruct(cleaned_voice)
    e = cleaned_voice.signals.values;
elseif exist('cleaned_voice','var') && isa(cleaned_voice,'timeseries')
    e = cleaned_voice.Data;
elseif exist('cleaned_voice','var')
    e = cleaned_voice;
else
    error('Cannot find Simulink output "cleaned_voice". Check To Workspace block.');
end
e = e(:);

N            = min(N_total, length(e));
e            = e(1:N);
voice_target = clean_voice(1:N);
d_corrupted  = d(1:N);
t_trim       = t(1:N);

%% 4.2 Segmental SNR (25ms frames)
frame_len   = round(0.025 * Fs);   
frame_shift = round(0.010 * Fs);   

noise_before    = d_corrupted - voice_target;
snr_before_seg  = segmental_snr(voice_target, noise_before, frame_len, frame_shift);

half_idx        = round(N/2);
e_conv          = e(half_idx:end);
voice_conv      = voice_target(half_idx:end);
noise_after     = e_conv - voice_conv;
snr_after_seg   = segmental_snr(voice_conv, noise_after, frame_len, frame_shift);

% keeping legacy metric just in case
snr_before_leg  = 10*log10(var(voice_target)  / var(noise_before));
snr_after_leg   = 10*log10(var(voice_conv)    / var(noise_after));

fprintf('─── Phase 4: SNR Results ─────────────────────────────────\n');
fprintf('  [Legacy whole-signal]\n');
fprintf('  Before: %6.2f dB  |  After: %6.2f dB  |  Gain: %6.2f dB\n', ...
        snr_before_leg, snr_after_leg, snr_after_leg - snr_before_leg);
fprintf('  [Segmental SNR — 25 ms frames, ITU-T P.56]\n');
fprintf('  Before: %6.2f dB  |  After: %6.2f dB  |  Gain: %6.2f dB\n\n', ...
        snr_before_seg, snr_after_seg, snr_after_seg - snr_before_seg);

%% 4.3 Fixed point Q15 check
try
    e_q15     = double(fi(e_conv, 1, 16, 15));
    n_q15     = e_q15 - voice_conv;
    snr_q15   = segmental_snr(voice_conv, n_q15, frame_len, frame_shift);
    q_penalty = snr_after_seg - snr_q15;
    fprintf('  [Fixed-Point Q15]\n');
    fprintf('  SegSNR with Q15: %6.2f dB  |  Penalty: %.2f dB\n\n', ...
            snr_q15, q_penalty);
    fp_ok = true;
catch ME
    fprintf('  [Fixed-Point] Skipped (%s)\n\n', ME.message);
    fp_ok = false;  e_q15 = [];  snr_q15 = NaN;
end

%% 4.4 Welch PSD
win_len  = 512;  overlap = 256;  nfft_psd = 1024;
[psd_d, f_psd] = pwelch(d_corrupted,  hamming(win_len), overlap, nfft_psd, Fs);
[psd_e, ~    ] = pwelch(e,            hamming(win_len), overlap, nfft_psd, Fs);
[psd_v, ~    ] = pwelch(voice_target, hamming(win_len), overlap, nfft_psd, Fs);

%% 4.5 Plots

% time domain stuff
figure('Name','Phase 4: Time Domain — Converged State','NumberTitle','off');
idx_z = t_trim >= 4.5 & t_trim <= 4.6;
plot(t_trim(idx_z), d_corrupted(idx_z), 'r',   'LineWidth',0.8); hold on;
plot(t_trim(idx_z), e(idx_z),           'b',   'LineWidth',1.2);
plot(t_trim(idx_z), voice_target(idx_z),'k--', 'LineWidth',1.2);
title('Time Domain: Corrupted vs. Cleaned vs. Target  [4.50–4.60 s]');
xlabel('Time (s)'); ylabel('Amplitude');
legend('Corrupted d(n)','Cleaned e(n)','Target Voice','Location','best');
grid on;

% psd plot
figure('Name','Phase 4: Welch PSD','NumberTitle','off');
plot(f_psd, 10*log10(psd_d),'r',  'LineWidth',1.2); hold on;
plot(f_psd, 10*log10(psd_e),'b',  'LineWidth',1.5);
plot(f_psd, 10*log10(psd_v),'k--','LineWidth',1.2);
title('Welch PSD: Noise Floor Reduction');
xlabel('Frequency (Hz)'); ylabel('PSD (dB/Hz)');
legend('Corrupted d(n)','Cleaned e(n)','Target Voice','Location','best');
xlim([0 2000]); grid on;

% fixed point overlay if toolbox is there
if fp_ok
    figure('Name','Phase 4: Fixed-Point Q15 Degradation','NumberTitle','off');
    t_conv = t_trim(half_idx : half_idx + length(voice_conv) - 1);
    ns = min(500, length(voice_conv));
    plot(t_conv(1:ns), voice_conv(1:ns), 'k--','LineWidth',1.2); hold on;
    plot(t_conv(1:ns), e_conv(1:ns),     'b',  'LineWidth',1.2);
    plot(t_conv(1:ns), e_q15(1:ns),      'm:', 'LineWidth',1.5);
    title(sprintf('Fixed-Point Q15 vs Float  (Q15 SegSNR = %.2f dB)', snr_q15));
    xlabel('Time (s)'); ylabel('Amplitude');
    legend('Target','Float e(n)','Q15 e(n)','Location','best'); grid on;
end

%% Local functions
function seg = segmental_snr(sig, noise, flen, fhop)
% calculates 25ms frame-averaged snr, clipping to reasonable db limits
    N = length(sig);  n0 = 1;  v = [];
    while n0 + flen - 1 <= N
        idx = n0 : n0+flen-1;
        ps  = var(sig(idx));  pn = var(noise(idx));
        if ps > 0 && pn > 0
            v(end+1) = 10*log10(ps/pn); %#ok<AGROW>
        end
        n0 = n0 + fhop;
    end
    seg = mean(max(min(v, 35), -10));
end