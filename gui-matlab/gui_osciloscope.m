%% ESP32 Real-Time Oscilloscope
clear; clc; close all;

%% ── Configuration ────────────────────────────────────────────────────────
COM_PORT     = '/dev/ttyUSB0';
BAUD_RATE    = 115200;
SAMPLE_COUNT = 256;
SAMPLE_RATE  = 1000000;
V_REF        = 3.3;        
ADC_MAX      = 4095;       
LPF_CUTOFF_HZ = 200000;
PEAK_HOLD     = true;        
FFT_YMIN_DB   = -80;
FREQ_MIN_HZ   = 1000;

% Square wave detection threshold — ratio of samples near only 2 levels
% if >70% of samples are within 10% of either min or max, treat as square
SQUARE_DETECT_THRESH = 0.70;

%% ── Zero-Phase Filter Coeff ──────────────────────────────────────────────
dt       = 1 / SAMPLE_RATE;
rc       = 1 / (2 * pi * LPF_CUTOFF_HZ);
rc_alpha = dt / (rc + dt);
rc_alpha = max(0.001, min(0.999, rc_alpha));

%% ── Window & Axis Setup ──────────────────────────────────────────────────
N         = SAMPLE_COUNT;
hannWin   = 0.5 * (1 - cos(2*pi*(0:N-1)/(N-1)));
nFFT      = N / 2;
freqResHz = SAMPLE_RATE / N;
freqAxis  = (0:nFFT-1) * freqResHz;
binMin    = max(2, floor(FREQ_MIN_HZ / freqResHz) + 1);
binMax    = nFFT - 1;

fprintf('--- Hz System Specifications ---\n');
fprintf('Frequency Bin Resolution: %.1f Hz\n', freqResHz);
fprintf('Max Observable Freq (Nyquist): %.1f Hz\n', SAMPLE_RATE / 2);
fprintf('------------------------------------\n');

%% ── Serial Initialization ────────────────────────────────────────────────
fprintf('Connecting to %s at %d baud...\n', COM_PORT, BAUD_RATE);
s = serialport(COM_PORT, BAUD_RATE);
configureTerminator(s, "LF");
s.Timeout = 5;
flush(s);

%% ── Figure & UI Layout ───────────────────────────────────────────────────
BG = [0.05 0.05 0.08]; AX = [0.03 0.03 0.06];
fig = figure('Name', 'ESP32 Oscilloscope', 'Color', BG, ...
             'Position', [100 100 1100 640], 'CloseRequestFcn', @(src,~) delete(src));

ax1 = axes(fig, 'Position', [0.07 0.55 0.90 0.38], 'Color', AX, ...
    'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5]);
grid(ax1, 'on'); hold(ax1, 'on');
tAxis = (0:N-1) / SAMPLE_RATE * 1000;
hRaw  = plot(ax1, tAxis, zeros(1,N), 'Color', [0.3 0.5 0.3], 'LineWidth', 0.8);
hFilt = plot(ax1, tAxis, zeros(1,N), 'Color', [0.2 1.0 0.4], 'LineWidth', 1.6);
ylim(ax1, [-0.1, V_REF + 0.45]); xlim(ax1, [tAxis(1), tAxis(end)]);
ylabel(ax1, 'Voltage (V)'); xlabel(ax1, 'Time (ms)');
infoText  = text(ax1, tAxis(3), V_REF + 0.32, '', ...
    'Color', [0.8 0.8 0.8], 'FontName', 'Courier New');
freqLabel = text(ax1, tAxis(end)*0.6, V_REF + 0.32, '', ...
    'Color', [0.35 0.95 0.6], 'FontName', 'Courier New', 'FontWeight', 'bold');
typeLabel = text(ax1, tAxis(end)*0.85, V_REF + 0.32, '', ...
    'Color', [0.9 0.7 0.2], 'FontName', 'Courier New', 'FontWeight', 'bold');

ax2 = axes(fig, 'Position', [0.07 0.07 0.90 0.38], 'Color', AX, ...
    'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5]);
grid(ax2, 'on'); hold(ax2, 'on');
hPeak    = plot(ax2, freqAxis, FFT_YMIN_DB*ones(1,nFFT), 'Color', [0.5 0.3 0.1], 'LineWidth', 0.8);
hFFT     = plot(ax2, freqAxis, FFT_YMIN_DB*ones(1,nFFT), 'Color', [1.0 0.6 0.1], 'LineWidth', 1.5);
hDomLine = plot(ax2, [NaN NaN], [FFT_YMIN_DB 10], '--', 'Color', [0.9 0.2 0.2], 'LineWidth', 1.2);
ylim(ax2, [FFT_YMIN_DB, 15]); xlim(ax2, [0, SAMPLE_RATE/2]);
ylabel(ax2, 'Magnitude (dBV)'); xlabel(ax2, 'Frequency (Hz)');

%% ── Filter function ──────────────────────────────────────────────────────
function y = simple_iir_zerophase(x, alpha)
    len = numel(x); yf = zeros(1, len); y = zeros(1, len);
    yf(1) = x(1);
    for n = 2:len
        yf(n) = alpha * x(n) + (1 - alpha) * yf(n-1);
    end
    y(end) = yf(end);
    for n = (len-1):-1:1
        y(n) = alpha * yf(n) + (1 - alpha) * y(n+1);
    end
end

%% ── Square wave detector ─────────────────────────────────────────────────
% Returns true if signal clusters around two levels (high/low) only
function result = is_square(voltage, thresh)
    vmin  = min(voltage);
    vmax  = max(voltage);
    vspan = vmax - vmin;
    if vspan < 0.1
        result = false;   % flat signal, no wave at all
        return;
    end
    band       = vspan * 0.15;   % 15% of swing around each rail
    near_low   = sum(voltage <= vmin + band);
    near_high  = sum(voltage >= vmax - band);
    result     = ((near_low + near_high) / numel(voltage)) >= thresh;
end

%% ── Main Processing Loop ─────────────────────────────────────────────────
samples      = zeros(1, N);
sampleIdx    = 0;
inFrame      = false;
peakSpectrum = FFT_YMIN_DB * ones(1, nFFT);
frameTimer   = tic;

try
    while ishandle(fig)
        ln = strtrim(char(readline(s)));
        if isempty(ln); continue; end

        if startsWith(ln, 'FRAME:')
            timeBetweenFrames = toc(frameTimer);
            frameTimer = tic;
            sampleIdx  = 0;
            inFrame    = true;
            samples    = zeros(1, N);

        elseif strcmp(ln, 'END') && inFrame
            if sampleIdx == N

                %% 1. Signal Prep
                voltage = samples * (V_REF / ADC_MAX);

                % detect wave type, apply filter only to sine
                square_wave = is_square(voltage, SQUARE_DETECT_THRESH);
                if square_wave
                    voltFilt = voltage;   % raw — no filter on square
                    set(typeLabel, 'String', '■ SQUARE');
                    set(typeLabel, 'Color',  [0.9 0.5 0.1]);
                else
                    voltFilt = simple_iir_zerophase(voltage, rc_alpha);
                    set(typeLabel, 'String', '~ SINE');
                    set(typeLabel, 'Color',  [0.2 0.9 0.5]);
                end

                %% 2. FFT
                acSignal = voltFilt - mean(voltFilt);
                windowed = acSignal .* hannWin;
                spectrum = fft(windowed, N);
                mag      = abs(spectrum(1:nFFT)) * (4 / N);
                mag(1)   = mag(1) / 2;
                magDB    = 20 * log10(mag + 1e-9);

                if PEAK_HOLD
                    peakSpectrum = max(peakSpectrum, magDB);
                end

                %% 3. Peak finding & interpolation
                searchRegion        = mag(binMin:binMax);
                [peakValLinear, localIdx] = max(searchRegion);
                globalBin           = localIdx + binMin - 1;

                if peakValLinear > 0.010 && globalBin > 1 && globalBin < nFFT
                    y1 = mag(globalBin - 1);
                    y2 = mag(globalBin);
                    y3 = mag(globalBin + 1);
                    denom  = 2 * (2*y2 - y1 - y3);
                    offset = 0;
                    if abs(denom) > 1e-6
                        offset = (y3 - y1) / denom;
                    end
                    domFreq = (globalBin - 1 + offset) * freqResHz;
                    set(hDomLine,  'XData', [domFreq domFreq]);
                    set(freqLabel, 'String', sprintf('► Dom. Freq: %.2f Hz', domFreq));
                else
                    set(hDomLine,  'XData', [NaN NaN]);
                    set(freqLabel, 'String', '► Dom. Freq: —');
                end

                %% 4. Graphical Update
                vpp  = max(voltFilt) - min(voltFilt);
                vrms = sqrt(mean(voltFilt .^ 2));
                set(hRaw,  'YData', voltage);
                set(hFilt, 'YData', voltFilt);
                set(hFFT,  'YData', magDB);
                set(hPeak, 'YData', peakSpectrum);
                set(infoText, 'String', sprintf('Vpp=%.2fV  Vrms=%.2fV  Data Window=%.0f ms', ...
                    vpp, vrms, timeBetweenFrames*1000));
                drawnow limitrate;
            end
            inFrame = false;

        elseif inFrame
            val = str2double(ln);
            if ~isnan(val) && sampleIdx < N
                sampleIdx          = sampleIdx + 1;
                samples(sampleIdx) = val;
            end
        end
    end
catch ME
    warning('Loop halted: %s', ME.message);
end
clear s;
