clc;
clear all;

%% INIT
MAX_MICS = 4;
MAX_ARRAYS = 4;
SPEED = 1500;
SIGMA = 3/192e5;
REPEAT = 1000;
HEIGHT = 0.57;
WIDTH = 0.29;
START_TIME = cputime;

%% GENERATE POPULATION
solution = point(-150.0,60.0);
population = array.empty(MAX_ARRAYS,0);
for i = 1:MAX_ARRAYS
    population(i) = array(MAX_MICS,SPEED);
    for j = 2:MAX_MICS
        if MAX_MICS == 4 & i == 1
            % CENTERED Y SHAPE
            X = [0 0 -WIDTH/2 WIDTH/2];
            Y = [0 -HEIGHT/2 HEIGHT/2 HEIGHT/2];
            population(i).receivers(j) = receiver(X(j),Y(j));
        elseif MAX_MICS == 4 & i == 2
            % EQUAL DISTANCE Y SHAPE
            distance = ((WIDTH / 2)^2 + HEIGHT^2) / (2*HEIGHT);
            X = [0 0 -WIDTH/2 WIDTH/2];
            Y = [0 -distance (HEIGHT-distance) (HEIGHT-distance)];
            population(i).receivers(j) = receiver(X(j),Y(j));
        elseif MAX_MICS == 4 & i == 3
            % T SHAPE
            X = [0 0 -WIDTH/2 WIDTH/2];
            Y = [0 -HEIGHT 0 0];
            population(i).receivers(j) = receiver(X(j),Y(j));
        elseif MAX_MICS == 4 & i == 4
            % RECTANGLE
            X = [0 WIDTH WIDTH 0];
            Y = [0 0 HEIGHT HEIGHT];
            population(i).receivers(j) = receiver(X(j),Y(j));
        else
            % RANDOM
            x = WIDTH*rand()-WIDTH/2;
            y = HEIGHT*rand()-HEIGHT/2;
            population(i).receivers(j) = receiver(x,y);
        end
    end
end

%% SOLVE
estimate = zeros(MAX_ARRAYS,REPEAT,2);
avg = zeros(MAX_ARRAYS,2);
v = zeros(REPEAT,MAX_MICS);
for i = 1:REPEAT
    clc
    fprintf('SIMULATION %d OUT OF %d\n',i,REPEAT);

    A = zeros(MAX_MICS-2,1);
    B = zeros(MAX_MICS-2,1);
    C = zeros(MAX_MICS-2,1);

    v(i,:) = SIGMA.*randn(MAX_MICS,1);

    for j = 1:MAX_ARRAYS
        %% ADD NOISE
        population(j) = population(j).time_difference(solution);
        for k = 2:MAX_MICS
            population(j).receivers(k).time = population(j).receivers(k).time + v(i,k-1);
        end

        %% OLS ESTIMATE
        receivers = population(j).receivers;
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
        estimate(j,i,:) = -[A B]\C;
        
        avg(j,:) = [mean(estimate(j,:,1)) mean(estimate(j,:,2))];
        population(j).solution = point(avg(j,1),avg(j,2));
        population(j) = population(j).compute_error(solution);
    end
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
    X(i) = estimate(index,i,1);
    Y(i) = estimate(index,i,2);
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
hold on;
scatter(solution.x, solution.y, 'filled', 'r');
scatter(0, 0, 'filled', 'w');
hold off;
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
