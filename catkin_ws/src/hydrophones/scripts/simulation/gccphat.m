function [x,y,z] = gccphat(a,b,c)
    %% INIT
    FS = 192e3;
    BUFFERSIZE = 1024;
    FREQUENCY = 30e3;
    SIGMA = 1e-1;
    MAGNITUDE = 1;
    MAX_MICS = 4;
    LENGTH_OF_PULSE = 1.3e-3;
    TIME_SHIFT = 1e-3;
    TIME_DIFFERENCE = [0 a b c];

    %% CREATE UNIT STEP
    UNIT_STEP = [];
    for i = 1:MAX_MICS
        for j = 1:BUFFERSIZE+1
            if j > (TIME_DIFFERENCE(i) + TIME_SHIFT) * FS ...
            && j < (LENGTH_OF_PULSE + TIME_DIFFERENCE(i) + TIME_SHIFT) * FS
                UNIT_STEP(i,j) = MAGNITUDE;
            else
                UNIT_STEP(i,j) = 0; 
            end
        end
    end

    %% CREATE SIGNAL
    t = 0:1/FS:BUFFERSIZE/FS;
    signal = [];
    v = SIGMA.*randn(MAX_MICS,BUFFERSIZE+1);
    for i = 1:MAX_MICS
        for j = 1:BUFFERSIZE+1
            signal(i,j) = v(i,j) + UNIT_STEP(i,j) * ...
                          sin(2*pi*FREQUENCY*t(j) + ...
                              2*pi*FREQUENCY*TIME_DIFFERENCE(i));
        end
    end

    %% ANALYZE
    n = pow2(nextpow2(BUFFERSIZE+1));
    y = [];
    for i = 1:MAX_MICS
        y(i,:) = fftshift(fft(signal(i,:),n));
    end
    f = (-n/2:n/2-1)*(FS/n);
    power = y.*conj(y)/n;
    theta = atan2(imag(y),real(y));

    %% FIND PEAK
    [top,index1] = max(y(1,:));
    [top,index2] = max(y(1,BUFFERSIZE:end));

    %% GCC-PHAT
    sol = zeros(MAX_MICS-1);
    gcc = zeros(MAX_MICS,2,2*BUFFERSIZE);
    index = zeros(MAX_MICS,2,2*BUFFERSIZE);
    for i = 2:MAX_MICS
        a = y(i,:).*conj(y(1,:));
        b = y(1,:).*conj(y(i,:));
        gcc(i,1,:) = ifft(a./abs(a));
        gcc(i,2,:) = ifft(b./abs(b));
        [top,index(i,1)] = max(abs(gcc(i,1,:)));
        [top,index(i,2)] = max(abs(gcc(i,2,:)));
        u = 1;
        if index(i,1) > BUFFERSIZE/2
            index(i,1) = -index(i,1);
            u = -1;
        end
        if index(i,2) > BUFFERSIZE/2
            index(i,2) = -index(i,2);
        end
        sol(i-1) = u*(BUFFERSIZE*2 + index(i,1) + index(i,2))/2/FS;
    end

    %% PLOT
    color = ['r' 'g' 'b' 'k'];
    names = {'First'; 'Second'; 'Third '; 'Fourth'};
    for i = 1:MAX_MICS
        subplot(MAX_MICS,3,3*i-2);
        plot(t,signal(i,:),color(i));
        clear title xlabel ylabel;
        xlabel('Time (s)');
        header = sprintf('Time Signal of %s Receiver',names{i});
        title(header);

        subplot(MAX_MICS,3,3*i-1);
        plot(f,power(i,:),color(i));
        clear title xlabel ylabel;
        xlabel('Frequency (Hz)');
        header = sprintf('Frequency of %s Receiver',names{i});
        title(header);

        subplot(MAX_MICS,3,3*i);
        plot(abs(squeeze(gcc(i,1,:))),color(i));
        clear title xlabel ylabel;
        xlabel('Time (s)');
        header = sprintf('Time Difference of %s Receiver',names{i});
        title(header);
    end

    x = sol(1);
    y = sol(2);
    z = sol(3);
end
