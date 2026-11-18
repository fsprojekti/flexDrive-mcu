% Extract signal and time from Simulink output
signal = out.simout;
t = out.tout;

% Sampling parameters
Fs = 1 / (t(2) - t(1));  % Sampling frequency
L = length(signal);      % Signal length

% Compute FFT
Y = fft(signal);
P2 = abs(Y/L);                 % Two-sided amplitude spectrum
P1 = P2(1:floor(L/2)+1);       % Single-sided
P1(2:end-1) = 2*P1(2:end-1);   % Double all but DC and Nyquist

% Frequency vector
f = Fs*(0:floor(L/2))/L;

% Limit to frequencies ≤ 10 Hz
idx = f <= 10;

% Smooth the spectrum for better visualization (optional)
P1_smooth = smoothdata(P1(idx), 'gaussian', 100);  % Try 'movmean' or others too

% Find dominant frequency in this range
[~, peakIdx] = max(P1_smooth);
peakFreq = f(idx);
peakAmplitude = P1_smooth(peakIdx);

% Plot
figure;
plot(f(idx), P1_smooth, 'LineWidth', 1.5)
title('FFT of Simulink Signal (0–10 Hz)')
xlabel('Frequency (Hz)')
ylabel('|Amplitude|')
grid on;
xlim([0 10])
ylim([0 max(P1_smooth)*1.1])

% Annotate peak
text(peakFreq(peakIdx), peakAmplitude, ...
    sprintf('\\leftarrow Peak at %.2f Hz', peakFreq(peakIdx)), ...
    'FontSize', 10, 'VerticalAlignment', 'bottom')

