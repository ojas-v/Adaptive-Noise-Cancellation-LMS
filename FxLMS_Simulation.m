% FxLMS_Simulation.m | Phase 5: Filtered-X LMS + Secondary Path
% Author: Ojas Vaidya

clear; clc; close all;
fprintf('=== FxLMS Simulation  |  Phase 5 ===\n\n');

%% 1. Environment setup
Fs       = 8000;
duration = 15;
t        = (0:1/Fs:duration-1/Fs)';
N_total  = length(t);
rng(42);   % set master seed once

%% 2. Signal generation

% am voice
f_fund   = 300;
am_rate  = 4;
am_depth = 0.3;
envelope    = 1 - am_depth * sin(2*pi*am_rate*t);
clean_voice = envelope .* ( sin(2*pi*f_fund*t)        ...
                           + 0.5*sin(2*pi*2*f_fund*t) ...
                           + 0.2*sin(2*pi*3*f_fund*t) );

% ref mic (noise + thermal)
noise_variance = 0.5;
x_clean        = sqrt(noise_variance) * randn(N_total, 1);
mems_sigma     = 1e-4;
x              = x_clean + mems_sigma * randn(N_total, 1);

% err mic independent noise
error_mic_noise = mems_sigma * randn(N_total, 1);

%% 3. Primary acoustic path
rir_len    = 200;
rir_tap    = (0:rir_len-1)';
decay_rate = 6.9 / (0.150 * Fs);

room_path    = exp(-decay_rate * rir_tap) .* randn(rir_len, 1);
room_path(1) = 1;
room_path    = room_path / norm(room_path);

primary_noise = filter(room_path, 1, x_clean);
d             = clean_voice + primary_noise;

%% 4. Secondary path (truth model)
true_secondary_path      = [0, 0, 0.1, 0.8, 0.3, -0.2, 0.05];
secondary_delay_samples  = 2;

%% 5. Causality check
% make sure primary delay >= secondary so controller has time to react
[~, peak_idx]          = max(abs(room_path));
primary_delay_samples  = peak_idx - 1;

causality_margin = primary_delay_samples - secondary_delay_samples;
fprintf('Causality Check\n');
fprintf('  Primary  delay: %d samples  (%.3f ms)\n', ...
        primary_delay_samples,  primary_delay_samples/Fs*1000);
fprintf('  Secondary delay: %d samples  (%.3f ms)\n', ...
        secondary_delay_samples, secondary_delay_samples/Fs*1000);
if causality_margin >= 0
    fprintf('  Margin: +%d samples  →  CAUSAL  ✓\n\n', causality_margin);
else
    fprintf('  Margin: %d samples   →  ACAUSAL WARNING ✗\n', causality_margin);
    fprintf('  Consider increasing filter look-ahead or adjusting path delays.\n\n');
end

%% 6. Offline secondary path ID via NLMS
num_taps_S  = 16;
mu_S_nlms   = 0.5;     
epsilon_S   = 1e-6;
N_sysid     = 20000;   

rng(123);              
probe_signal  = randn(N_sysid, 1);
mic_recording = filter(true_secondary_path, 1, probe_signal);

S_hat    = zeros(num_taps_S, 1);
e_sysid  = zeros(N_sysid, 1);

for n = num_taps_S : N_sysid
    xv         = probe_signal(n:-1:n-num_taps_S+1);
    yv         = S_hat' * xv;
    e_sysid(n) = mic_recording(n) - yv;
    mu_norm_S  = mu_S_nlms / (epsilon_S + xv'*xv);
    S_hat      = S_hat + mu_norm_S * e_sysid(n) * xv;
end

% calculate erle to see how good the model is
erle_sysid = 10*log10(var(mic_recording(num_taps_S:end)) / ...
                      var(e_sysid(num_taps_S:end)));
fprintf('Secondary Path Identification\n');
fprintf('  ERLE of S_hat model: %.2f dB\n', erle_sysid);
fprintf('  (≥ 20 dB = adequate for ANC deployment)\n\n');

% export for Simulink
assignin('base','S_hat',               S_hat);
assignin('base','true_secondary_path', true_secondary_path);

%% 7. NLMS stability bounds
num_taps_W = 64;
x_prime    = filter(S_hat, 1, x);            
P_x_prime  = var(x_prime(1:1000));           
mu_max     = 1 / (num_taps_W * P_x_prime);
mu_W       = 0.05 * mu_max;                  % keeping it conservative at 5%

fprintf('─── NLMS Stability Analysis ──────────────────────────────\n');
fprintf('  Filter taps N        : %d\n',    num_taps_W);
fprintf('  P_x_prime (est.)     : %.6f\n',  P_x_prime);
fprintf('  Theoretical µ_max    : %.6f\n',  mu_max);
fprintf('  Selected µ  (5%%)    : %.6f\n',  mu_W);
fprintf('  Stability margin     : %.1f%%\n\n', (1 - mu_W/mu_max)*100);

%% 8. NLMS-FxLMS loop
epsilon_W        = 1e-6;

W                = zeros(num_taps_W, 1);
x_buffer         = zeros(num_taps_W, 1);
x_prime_buffer   = zeros(num_taps_W, 1);
y_speaker_buffer = zeros(length(true_secondary_path), 1);

e_fxlms          = zeros(N_total, 1);
weight_norm_log  = zeros(N_total, 1);   

disp('Running NLMS-FxLMS real-time loop...');
tic;

for n = 1:N_total

    % 1. shift samples in buffer
    x_buffer       = [x(n);       x_buffer(1:end-1)];
    x_prime_buffer = [x_prime(n); x_prime_buffer(1:end-1)];

    % 2. anti-noise output
    y_anti_noise = W' * x_buffer;

    % 3. push through physical sec path
    y_speaker_buffer      = [y_anti_noise; y_speaker_buffer(1:end-1)];
    y_physical_mic        = true_secondary_path * y_speaker_buffer;

    % 4. get actual error
    e_fxlms(n) = d(n) - y_physical_mic + error_mic_noise(n);

    % 5. adapt weights
    mu_adaptive = mu_W / (epsilon_W + x_prime_buffer'*x_prime_buffer);
    W = W + mu_adaptive * e_fxlms(n) * x_prime_buffer;

    % 6. tracking convergence
    weight_norm_log(n) = norm(W);

end

loop_time = toc;
fprintf('Loop done: %.3f s  (%.1f µs/sample)\n\n', ...
        loop_time, loop_time/N_total*1e6);

%% 9. Fixed-point Q15 test
try
    W_q15     = double(fi(W, 1, 16, 15));
    q_err     = W - W_q15;
    q_snr_W   = 20*log10(norm(W) / norm(q_err));

    idx_last2 = (N_total - 2*Fs + 1) : N_total;
    x_bq = zeros(num_taps_W,1);
    y_bq = zeros(length(true_secondary_path),1);
    e_q15_arr = zeros(length(idx_last2),1);
    k = 0;
    for n = idx_last2
        k = k+1;
        x_bq = [x(n); x_bq(1:end-1)];
        yq   = W_q15' * x_bq;
        y_bq = [yq; y_bq(1:end-1)];
        yp   = true_secondary_path * y_bq;
        e_q15_arr(k) = d(n) - yp;
    end

    frame_len   = round(0.025*Fs);
    frame_shift = round(0.010*Fs);
    voice_last2 = clean_voice(idx_last2);
    n_q15_arr   = e_q15_arr - voice_last2;
    snr_q15_seg = segmental_snr(voice_last2, n_q15_arr, frame_len, frame_shift);

    fprintf('─── Fixed-Point Q15 ──────────────────────────────────────\n');
    fprintf('  Weight vector SNR (float vs Q15): %.2f dB\n', q_snr_W);
    fprintf('  Output SegSNR with Q15 weights:   %.2f dB\n\n', snr_q15_seg);
    fp_ok = true;
catch ME
    fprintf('[Fixed-Point] Skipped: %s\n\n', ME.message);
    fp_ok = false;  snr_q15_seg = NaN;  q_snr_W = NaN;
end

%% 10. SegSNR Validation
frame_len   = round(0.025*Fs);
frame_shift = round(0.010*Fs);

noise_before   = d - clean_voice;
snr_before_seg = segmental_snr(clean_voice, noise_before, frame_len, frame_shift);

half_idx       = round(N_total/2);
e_conv         = e_fxlms(half_idx:end);
voice_conv     = clean_voice(half_idx:end);
noise_conv     = e_conv - voice_conv;
snr_after_seg  = segmental_snr(voice_conv, noise_conv, frame_len, frame_shift);

% legacy
snr_before_leg = 10*log10(var(clean_voice) / var(noise_before));
snr_after_leg  = 10*log10(var(voice_conv)  / var(noise_conv));

fprintf('─── Phase 5: SNR Validation ──────────────────────────────\n');
fprintf('  [Legacy whole-signal]\n');
fprintf('  Before: %6.2f dB  |  After: %6.2f dB  |  Gain: %6.2f dB\n', ...
        snr_before_leg, snr_after_leg, snr_after_leg - snr_before_leg);
fprintf('  [Segmental SNR — 25 ms frames]\n');
fprintf('  Before: %6.2f dB  |  After: %6.2f dB  |  Gain: %6.2f dB\n\n', ...
        snr_before_seg, snr_after_seg, snr_after_seg - snr_before_seg);

%% 11. Monte carlo robustness sweep (500 trials)
N_mc_trials = 500;
N_mc        = round(1 * Fs);   
t_mc        = (0:N_mc-1)' / Fs;
env_mc      = 1 - am_depth * sin(2*pi*am_rate*t_mc);
voice_mc    = env_mc .* (sin(2*pi*f_fund*t_mc) + 0.5*sin(2*pi*2*f_fund*t_mc) ...
                         + 0.2*sin(2*pi*3*f_fund*t_mc));
snr_mc      = zeros(N_mc_trials, 1);
rng(0);

fprintf('Running Monte Carlo (%d trials) ... ', N_mc_trials);
for trial = 1:N_mc_trials

    mu_trial    = mu_max * (0.01 + 0.09*rand());   
    nvar_trial  = 0.2  + 0.8*rand();               

    x_mc        = sqrt(nvar_trial) * randn(N_mc, 1);
    xp_mc       = filter(S_hat, 1, x_mc);
    rlen_mc     = min(20, rir_len);
    pn_mc       = filter(room_path(1:rlen_mc), 1, x_mc);
    d_mc        = voice_mc + pn_mc;

    W_mc  = zeros(num_taps_W,1);
    xb_mc = zeros(num_taps_W,1);
    xpm   = zeros(num_taps_W,1);
    ys_mc = zeros(length(true_secondary_path),1);
    e_mc  = zeros(N_mc, 1);

    for n = 1:N_mc
        xb_mc = [x_mc(n); xb_mc(1:end-1)];
        xpm   = [xp_mc(n); xpm(1:end-1)];
        y_mc  = W_mc' * xb_mc;
        ys_mc = [y_mc; ys_mc(1:end-1)];
        yp_mc = true_secondary_path * ys_mc;
        e_mc(n) = d_mc(n) - yp_mc;
        mu_ad  = mu_trial / (epsilon_W + xpm'*xpm);
        W_mc   = W_mc + mu_ad * e_mc(n) * xpm;
    end

    hf = round(N_mc/2);
    if hf >= frame_len
        ns_mc = e_mc(hf:end) - voice_mc(hf:end);
        snr_mc(trial) = segmental_snr(voice_mc(hf:end), ns_mc, frame_len, frame_shift);
    else
        snr_mc(trial) = NaN;
    end
end

snr_mc = snr_mc(~isnan(snr_mc));
fprintf('Done.\n');
fprintf('─── Monte Carlo Results (%d valid trials) ─────────────────\n', length(snr_mc));
fprintf('  Mean     : %6.2f dB\n',  mean(snr_mc));
fprintf('  Std      : %6.2f dB\n',  std(snr_mc));
fprintf('  5th pct  : %6.2f dB  (worst-case bound)\n', prctile(snr_mc,5));
fprintf('  95th pct : %6.2f dB  (best-case bound)\n\n',  prctile(snr_mc,95));

%% 12. Plots

% fig 1 time domain
figure('Name','Phase 5: FxLMS Time Domain','NumberTitle','off');
idx_z = t >= 4.5 & t <= 4.6;
plot(t(idx_z), d(idx_z),           'r',   'LineWidth',0.8); hold on;
plot(t(idx_z), e_fxlms(idx_z),     'b',   'LineWidth',1.2);
plot(t(idx_z), clean_voice(idx_z), 'k--', 'LineWidth',1.2);
title('FxLMS: Corrupted vs Cleaned vs Target  [4.50–4.60 s]');
xlabel('Time (s)'); ylabel('Amplitude');
legend('Corrupted d(n)','Cleaned e(n)','Target Voice','Location','best');
grid on;

% fig 2 convergence
figure('Name','Phase 5: Convergence Trajectory','NumberTitle','off');
subplot(2,1,1);
plot(t, e_fxlms, 'b','LineWidth',0.6); hold on;
plot(t, clean_voice,'k--','LineWidth',0.8);
title('FxLMS Output — Full 15 s');
xlabel('Time (s)'); ylabel('Amplitude');
legend('Cleaned e(n)','Target Voice','Location','best'); grid on;
subplot(2,1,2);
plot(t, weight_norm_log,'Color',[0.1 0.6 0.1],'LineWidth',1);
title('Filter Weight Norm ||W(n)|| — Convergence Indicator');
xlabel('Time (s)'); ylabel('||W||'); grid on;

% fig 3 psd
win_len=512; overlap=256; nfft_psd=1024;
[psd_d, f_psd] = pwelch(d,          hamming(win_len),overlap,nfft_psd,Fs);
[psd_e, ~    ] = pwelch(e_fxlms,    hamming(win_len),overlap,nfft_psd,Fs);
[psd_v, ~    ] = pwelch(clean_voice,hamming(win_len),overlap,nfft_psd,Fs);
figure('Name','Phase 5: Welch PSD','NumberTitle','off');
plot(f_psd,10*log10(psd_d),'r',  'LineWidth',1.2); hold on;
plot(f_psd,10*log10(psd_e),'b',  'LineWidth',1.5);
plot(f_psd,10*log10(psd_v),'k--','LineWidth',1.2);
title('Welch PSD: FxLMS Noise Floor Reduction');
xlabel('Frequency (Hz)'); ylabel('PSD (dB/Hz)');
legend('Corrupted d(n)','Cleaned e(n)','Target Voice','Location','best');
xlim([0 2000]); grid on;

% fig 4 sec path comparison
figure('Name','Phase 5: Secondary Path ID','NumberTitle','off');
stem(true_secondary_path,'k','filled','DisplayName','True S'); hold on;
stem(S_hat,'b--','filled','DisplayName','Estimated \hat{S}');
title(sprintf('Secondary Path: True vs Estimated  (ERLE = %.1f dB)', erle_sysid));
xlabel('Tap Index'); ylabel('Coefficient'); legend('Location','best'); grid on;

% fig 5 monte carlo hist
figure('Name','Phase 5: Monte Carlo Robustness','NumberTitle','off');
histogram(snr_mc,30,'FaceColor',[0.2 0.5 0.8],'EdgeColor','white');
hold on;
xline(mean(snr_mc),  'k-', 'LineWidth',2, ...
      'Label',sprintf('Mean %.1f dB',mean(snr_mc)));
xline(prctile(snr_mc,5),'r--','LineWidth',1.5, ...
      'Label',sprintf('5th pct %.1f dB',prctile(snr_mc,5)));
title('Monte Carlo SegSNR Distribution  (500 trials, µ & noise variance randomised)');
xlabel('Segmental SNR (dB)'); ylabel('Count'); grid on;

% fig 6 q15 overlay
if fp_ok
    figure('Name','Phase 5: Fixed-Point Q15 vs Float','NumberTitle','off');
    t_l2 = t(idx_last2);
    ns   = min(400, length(voice_last2));
    plot(t_l2(1:ns), voice_last2(1:ns),   'k--','LineWidth',1.2); hold on;
    plot(t_l2(1:ns), e_fxlms(idx_last2(1:ns)),'b','LineWidth',1.2);
    plot(t_l2(1:ns), e_q15_arr(1:ns),     'm:', 'LineWidth',1.5);
    title(sprintf('Q15 vs Float  (Q15 SegSNR = %.2f dB)', snr_q15_seg));
    xlabel('Time (s)'); ylabel('Amplitude');
    legend('Target','Float e(n)','Q15 Weights','Location','best'); grid on;
end

%% 13. Summary print
fprintf('\n');
fprintf('  FINAL SUMMARY\n');
fprintf('\n');
fprintf('  SegSNR Before ANC            : %6.2f dB\n', snr_before_seg);
fprintf('  SegSNR After  ANC (float)    : %6.2f dB\n', snr_after_seg);
fprintf('  Improvement                  : %6.2f dB\n', snr_after_seg - snr_before_seg);
if fp_ok
fprintf('  SegSNR After  ANC (Q15)      : %6.2f dB\n', snr_q15_seg);
fprintf('  Weight vector Q15 SNR        : %6.2f dB\n', q_snr_W);
end
fprintf('  MC mean SegSNR               : %6.2f dB\n', mean(snr_mc));
fprintf('  MC 5th-percentile            : %6.2f dB\n', prctile(snr_mc,5));
fprintf('  Secondary path ERLE          : %6.2f dB\n', erle_sysid);
fprintf('  Causality margin             : %+d samples\n', causality_margin);
fprintf('\n\n');

%% 14. Export to simulink workspace
assignin('base','S_hat',               S_hat);
assignin('base','true_secondary_path', true_secondary_path);
assignin('base','sim_d',               [t, d]);
assignin('base','sim_x',               [t, x]);
assignin('base','sim_clean_voice',     [t, clean_voice]);
disp('Workspace variables exported for FxLMS_Model.slx.');

%% Local functions
function seg = segmental_snr(sig, noise, flen, fhop)
% frame-averaged snr clipped to [−10, 35] db 
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