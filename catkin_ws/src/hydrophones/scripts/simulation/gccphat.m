function [x,y,z] = gccphat(a,b,c)
    %% INIT
    FS = 192e3;
    BUFFERSIZE = 1024;
    FREQUENCY = 30e3;
    SIGMA = 1e-2;
    MAGNITUDE = 5;
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
    gcc = zeros(MAX_MICS,2*BUFFERSIZE);
    index = zeros(MAX_MICS,2*BUFFERSIZE);
    for i = 2:MAX_MICS
        a = y(i,:).*conj(y(1,:));
        b = y(1,:).*conj(y(i,:));
        gcc(i,:) = ifft(a./abs(a));

        [top,index(i)] = max(abs(gcc(i,:)));
    end

    %% PLOT
    color = ['r' 'g' 'b' 'k'];
    names = {'First'; 'Second'; 'Third '; 'Fourth'};
    for i = 2:MAX_MICS
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

        positive = 1;
        bound = [1 1];
        interval = 5;
        interpolation = 0.001;
        if index(i) > BUFFERSIZE
            positive = -1;
            if index(i) >= 2*BUFFERSIZE-interval
                bound(2) = 0;
            end
        elseif index(i) <= interval
            bound(1) = 0;
        end
        begin = bound(1)*(index(i)-interval);
        final = bound(2)*(index(i)+interval) + (1-bound(2))*2*BUFFERSIZE;
        a = abs(squeeze(gcc(i,begin:final)));
        s = begin:final;
        u = begin:interpolation:final;
        
        subplot(MAX_MICS,3,3*i);
        b = sinc_interp(a,s,u);
        plot(u,b,color(i));
        [top,peak] = max(b);

        if positive == 1
            sol(i) = (peak*interpolation+s(1))/FS;
        else
            sol(i) = (peak*interpolation+s(1)-2*BUFFERSIZE)/FS;
        end

        clear title xlabel ylabel;
        xlabel('Time (s)');
        header = sprintf('Time Difference of %s Receiver',names{i});
        title(header);
    end

    x = sol(2);
    y = sol(3);
    z = sol(4);
end