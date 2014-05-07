clc;
clear all;

%% INIT
MAX_MICS = 4;
MAX_ARRAYS = 1000;
SPEED = 1500;
SIGMA = 3/192e4;
REPEAT = 100;
HEIGHT = 0.57;
WIDTH = 0.29;

START_TIME = cputime;

%% GENERATE POPULATION
solution = point(19.0,6.0);
population = array.empty(MAX_ARRAYS,0);
for i = 1:MAX_ARRAYS
    population(i) = array(MAX_MICS,SPEED);
    for j = 2:MAX_MICS
        x = 2*WIDTH*rand()-WIDTH;
        y = 2*HEIGHT*rand()-HEIGHT;
        population(i).receivers(j) = receiver(x,y);
    end
    population(i) = population(i).time_difference(solution);
end

%% SOLVE
estimate = zeros(MAX_ARRAYS,REPEAT,2);
avg = zeros(MAX_ARRAYS,2);
for i = 1:MAX_ARRAYS
    clc
    fprintf('SIMULATING RECEIVER ARRAY %d OUT OF %d\n',i,MAX_ARRAYS);

    A = zeros(MAX_MICS-2,1);
    B = zeros(MAX_MICS-2,1);
    C = zeros(MAX_MICS-2,1);

    for j = 1:REPEAT
        %% ADD NOISE
        for k = 2:MAX_MICS
            population(i).receivers(k).time = population(i).receivers(k).time + SIGMA.*randn();
        end

        %% OLS ESTIMATE
        receivers = population(i).receivers;
        for k = 3:MAX_MICS
            A(k) = 2*receivers(k).pos.x / (SPEED*receivers(k).time) ...
                 - 2*receivers(2).pos.x / (SPEED*receivers(2).time);
            B(k) = 2*receivers(k).pos.y / (SPEED*receivers(k).time) ...
                 - 2*receivers(2).pos.y / (SPEED*receivers(2).time);
            C(k) = SPEED*(receivers(k).time - receivers(2).time) ...
                 - ((receivers(k).pos.x)^2 + (receivers(k).pos.y)^2) ...
                 / (SPEED*receivers(k).time) + ((receivers(2).pos.x)^2 ...
                 + (receivers(2).pos.y)^2) / (SPEED*receivers(2).time);
        end
        estimate(i,j,:) = -[A B]\C;
    end

    avg(i,:) = [mean(estimate(i,:,1)) mean(estimate(i,:,2))];
    population(i).solution = point(avg(i,1),avg(i,2));
    population(i) = population(i).compute_error(solution);
end

%% FIND OPTIMAL ARRAY
E = zeros(MAX_ARRAYS,1);
for i = 1:MAX_ARRAYS
    E(i) = population(i).error;
end
[best, index] = min(E);

X = zeros(REPEAT,1);
Y = zeros(REPEAT,1);
for i = 1:REPEAT
    X(i) = estimate(index,i,1) - solution.x;
    Y(i) = estimate(index,i,2) - solution.y;
end

%% PLOT OPTIMAL ARRAY
clf;
subplot(2,1,1);
hold on;
grid on;
for i = 1:MAX_MICS
    scatter(population(index).receivers(i).pos.x, ...
            population(index).receivers(i).pos.y);
end
hold off;
title('Optimal Receiver Placement','FontSize',20,'interpreter','latex');
xlabel('X (m)','FontSize',15,'interpreter','latex');
ylabel('Y (m)','FontSize',15,'interpreter','latex');
set(gca,'Fontsize',14);

%% PLOT SOLUTIONS
subplot(2,1,2);
scatter(X,Y);
grid on;
title('Errors of Estimated Solutions of Optimal Array','FontSize',20,'interpreter','latex');
xlabel('X (m)','FontSize',15,'interpreter','latex');
ylabel('Y (m)','FontSize',15,'interpreter','latex');
set(gca,'Fontsize',14);

%% PRINT
clc;
fprintf('RECEIVERS SIMULATED: %d\n',MAX_MICS);
fprintf('ARRAYS SIMULATED: %d\n',MAX_ARRAYS);
fprintf('DISTANCE FROM SOURCE: %4.2f m\n\n',norm([solution.x solution.y]));

fprintf('SOLUTION: (%4.2f, %4.2f) m\n',solution.x,solution.y);
fprintf('BEST MEAN SOLUTION: (%4.6f, %4.6f) m\n',avg(index,1),avg(index,2));
fprintf('BEST MEAN ERROR: %3.4f %%\n\n',population(index).error*100);

fprintf('SPEED OF SOUND: %d m/s\n',SPEED);
fprintf('SIGMA: %d\n',SIGMA);
fprintf('REPETITIONS: %d\n\n',REPEAT);

fprintf('TIME ELAPSED: %4.2f s\n\n',cputime - START_TIME);
