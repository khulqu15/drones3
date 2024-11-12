figure;

subplot(3, 1, 1);
hold on;
xPlot = plot(NaN, NaN, '-o', 'DisplayName', 'X');
yPlot = plot(NaN, NaN, '-o', 'DisplayName', 'Y');
zPlot = plot(NaN, NaN, '-o', 'DisplayName', 'Z');
hold off;
title('MPU Data (X, Y, Z)');
xlabel('Waktu');
ylabel('MPU');
legend show;
grid on;

subplot(3, 1, 2);
hold on;
fftXPlot = plot(NaN, NaN, '-o', 'DisplayName', 'FFT X');
fftYPlot = plot(NaN, NaN, '-o', 'DisplayName', 'FFT Y');
fftZPlot = plot(NaN, NaN, '-o', 'DisplayName', 'FFT Z');
hold off;
title('FFT of MPU');
xlabel('Frequency index (k)');
ylabel('Magnitude');
legend show;
grid on;

subplot(3, 1, 3);
hold on;
logDecXPlot = plot(NaN, NaN, '-o', 'DisplayName', 'Log Dec X');
logDecYPlot = plot(NaN, NaN, '-o', 'DisplayName', 'Log Dec Y');
logDecZPlot = plot(NaN, NaN, '-o', 'DisplayName', 'Log Dec Z');
hold off;
title('Logarithmic Decrement of FFT');
xlabel('Frequency index (k)');
ylabel('Logarithmic Decrement');
legend show;
grid on;

updateInterval = 2;
t = timer('ExecutionMode', 'fixedRate', 'Period', updateInterval, 'TimerFcn', @updateData);

start(t);

cleanupObj = onCleanup(@() stop(t));

function updateData(~, ~)
    endpoint = "https://khafsin.hayago.id/api/imu";    
    options = weboptions('Timeout', 10, 'CertificateFilename', '');
    
    try
        data = webread(endpoint, options);
        disp(data);
        
        x = [data.x];
        y = [data.y];
        z = [data.z];
        
        timestamps = {data.created_at};
        timestamps = datetime(timestamps, 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss.SSSSSS''Z''', 'TimeZone', 'UTC');
        timestamps_num = datenum(timestamps);
        
        subplot(3, 1, 1);
        set(findobj('DisplayName', 'X'), 'XData', timestamps_num, 'YData', x);
        set(findobj('DisplayName', 'Y'), 'XData', timestamps_num, 'YData', y);
        set(findobj('DisplayName', 'Z'), 'XData', timestamps_num, 'YData', z);
        datetick('x', 'yyyy-mm-dd HH:MM', 'keepticks', 'keeplimits');
        
        fft_x = FastFourierTransform(x);
        fft_y = FastFourierTransform(y);
        fft_z = FastFourierTransform(z);
        
        subplot(3, 1, 2);
        freq_index = 0:length(fft_x)-1;
        set(findobj('DisplayName', 'FFT X'), 'XData', freq_index, 'YData', fft_x);
        set(findobj('DisplayName', 'FFT Y'), 'XData', freq_index, 'YData', fft_y);
        set(findobj('DisplayName', 'FFT Z'), 'XData', freq_index, 'YData', fft_z);
        
        logDec_x = LogarithmicDecrement(fft_x);
        logDec_y = LogarithmicDecrement(fft_y);
        logDec_z = LogarithmicDecrement(fft_z);
        
        subplot(3, 1, 3);
        set(findobj('DisplayName', 'Log Dec X'), 'XData', freq_index, 'YData', logDec_x);
        set(findobj('DisplayName', 'Log Dec Y'), 'XData', freq_index, 'YData', logDec_y);
        set(findobj('DisplayName', 'Log Dec Z'), 'XData', freq_index, 'YData', logDec_z);
        
        fuzzyCategoryX = FuzzyClassification(logDec_x);
        fuzzyCategoryY = FuzzyClassification(logDec_y);
        fuzzyCategoryZ = FuzzyClassification(logDec_z);
        
        fprintf("Status untuk X: %s\n", fuzzyCategoryX);
        fprintf("Status untuk Y: %s\n", fuzzyCategoryY);
        fprintf("Status untuk Z: %s\n", fuzzyCategoryZ);
        
        drawnow;
        
    catch ME
        disp("Error: " + ME.message);
    end
end

function Xk = FastFourierTransform(x)
    N = length(x);
    Xk_real = zeros(1, N);
    Xk_imag = zeros(1, N);

    for k = 1:N
        for n = 1:N
            angle = -2 * pi * (k-1) * (n-1) / N;
            Xk_real(k) = Xk_real(k) + x(n) * cos(angle);
            Xk_imag(k) = Xk_imag(k) + x(n) * sin(angle);
        end
    end

    Xk = sqrt(Xk_real.^2 + Xk_imag.^2);
end

function logDec = LogarithmicDecrement(fftData)
    peaks = findpeaks(fftData);
    logDec = zeros(size(fftData));
    for i = 1:length(peaks)-1
        logDec(i) = log(peaks(i) / peaks(i+1));
    end
    logDec(isnan(logDec)) = 0;
end


function category = FuzzyClassification(logDec)
    weak = max(0, min(1, (0.2 - logDec) / 0.2));
    normal = max(0, min((logDec - 0.1) / 0.2, (0.5 - logDec) / 0.2));
    strong = max(0, min((logDec - 0.4) / 0.6, 1));

    [~, idx] = max([weak, normal, strong]);
    categories = {'Lemah', 'Normal', 'Kuat'};
    category = categories{idx};
end