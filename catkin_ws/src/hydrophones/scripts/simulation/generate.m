clc;
clear all;
format long g;

%% INIT
MAX_MICS = 4;
MAX_ARRAYS = 4;
SPEED = 1500;
REPEAT = 10;

HEIGHT = 1.83;   % 0.5969;
WIDTH = 0.91;    % 0.2921;
DEPTH_OF_ROBOT = 2;
DEPTH_OF_PINGER = 4.2672;

START_TIME = cputime;

%% GENERATE POPULATION
solution = point(85.00,-62.00,DEPTH_OF_PINGER);
population(MAX_ARRAYS) = array();
for i = 1:MAX_ARRAYS
    population(i) = array(MAX_MICS,SPEED);
    for j = 1:MAX_MICS
        if MAX_MICS == 4 && i == 1
            % CENTERED Y SHAPE
            X = [0 0 -WIDTH/2 WIDTH/2];
            Y = [0 -HEIGHT/2 HEIGHT/2 HEIGHT/2];
            population(i).receivers(j) = receiver(X(j),Y(j),DEPTH_OF_ROBOT);
        elseif MAX_MICS == 4 && i == 2
            % EQUAL DISTANCE Y SHAPE
            distance = ((WIDTH/2)^2 + HEIGHT^2) / (2*HEIGHT);
            X = [0 0 -WIDTH/2 WIDTH/2];
            Y = [0 -distance (HEIGHT-distance) (HEIGHT-distance)];
            population(i).receivers(j) = receiver(X(j),Y(j),DEPTH_OF_ROBOT);
        elseif MAX_MICS == 4 && i == 3
            % T SHAPE
            X = [0 0 -WIDTH/2 WIDTH/2];
            Y = [0 -HEIGHT 0 0];
            population(i).receivers(j) = receiver(X(j),Y(j),DEPTH_OF_ROBOT);
        elseif MAX_MICS == 4 && i == 4
            % RECTANGLE
            X = [0 WIDTH WIDTH 0];
            Y = [0 0 HEIGHT HEIGHT];
            population(i).receivers(j) = receiver(X(j),Y(j),DEPTH_OF_ROBOT);
        else
            % RANDOM
            x = WIDTH*rand()-WIDTH/2;
            y = HEIGHT*rand()-HEIGHT/2;
            population(i).receivers(j) = receiver(x,y,DEPTH_OF_ROBOT);
        end
    end
end

%% SOLVE
estimate = zeros(MAX_ARRAYS,REPEAT,2);
theta = zeros(REPEAT,MAX_ARRAYS);
err = zeros(MAX_ARRAYS,1);
for i = 1:REPEAT
    clc
    fprintf('SIMULATION %d OUT OF %d\n',i,REPEAT);

    A = zeros(MAX_MICS-2,1);
    B = zeros(MAX_MICS-2,1);
    C = zeros(MAX_MICS-2,1);

    for j = 1:MAX_ARRAYS
        population(j) = population(j).time_difference(solution);
        receivers = population(j).receivers;

        [t1,t2,t3] = gccphat(receivers(2).time,receivers(3).time,receivers(4).time);
        receivers(2).time = t1;
        receivers(3).time = t2;
        receivers(4).time = t3;
        t = [t1,t2,t3];

        %% OLS ESTIMATE
        for k = 3:MAX_MICS
            A(k) = 2*receivers(k).pos.x / (SPEED*t(k-1)) ...
                 - 2*receivers(2).pos.x / (SPEED*t(1));
            B(k) = 2*receivers(k).pos.y / (SPEED*t(k-1)) ...
                 - 2*receivers(2).pos.y / (SPEED*t(1));
            C(k) = SPEED*(t(k-1) - t(1)) ...
                 - ((receivers(k).pos.x)^2 + (receivers(k).pos.y)^2) ...
                 / (SPEED*t(k-1)) + ((receivers(2).pos.x)^2 ...
                 + (receivers(2).pos.y)^2) / (SPEED*t(1));
        end
        estimate(j,i,:) = -[A B]\C;
        theta(i,j) = atan2(estimate(j,i,2),estimate(j,i,1));
        
        population(j).solution = point(estimate(j,i,1),estimate(j,i,2));
        population(j) = population(j).compute_error(solution);
        err(j,i) = population(j).error;
    end
end

%% FIND OPTIMAL ARRAY
E = zeros(MAX_ARRAYS,1);
avg = zeros(MAX_ARRAYS,3);
for i = 1:MAX_ARRAYS
    avg(i,:) = [mean(estimate(i,:,1)) mean(estimate(i,:,2)) mean(theta(:,i))];
    E(i) = mean(err(i,:));
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
scatter(solution.x,solution.y,'filled','r');
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
fprintf('DISTANCE FROM SOURCE: %4.2f m\n\n',norm([solution.x solution.y solution.z]));

fprintf('SOLUTION: (%4.2f, %4.2f) m\n',solution.x,solution.y);
fprintf('BEST MEAN SOLUTION: (%4.6f, %4.6f) m\n',avg(index,1),avg(index,2));
fprintf('BEST MEAN ERROR: %3.6f %%\n\n',E(index)*100);

fprintf('AVERAGE DISTANCE ERROR: %3.2f m\n',norm(avg(index,1:2))-norm([solution.x solution.y]));
fprintf('AVERAGE THETA ERROR: %3.3f%c\n\n',(avg(index,3)-atan2(solution.y,solution.x))*180/pi,char(176));

fprintf('SPEED OF SOUND: %d m/s\n',SPEED);
fprintf('REPETITIONS: %d\n\n',REPEAT);

fprintf('TIME ELAPSED: %4.2f s\n\n',cputime - START_TIME);

%% ORCHESTRA
% load handel;
% for i = 1:3
%     sound(y,Fs);
%     pause(2.1);
% end
