clc;
clear;
clf

% Define global variables
global g %transformed maze grid

%variables needed for floodfill and dfs to output the desired path
global done
global p; 
global d; 
global MIN; 
global vis; 

%start and end positions in (x,y)
global snewx
global snewy
global enewx
global enewy


%maze boundaries
global xmin
global xmax
global ymin
global ymax

% Define maze boundaries
xmin = -10;
xmax = 10;
ymin = -10;
ymax = 10;

% Define obstacles as inequalities in the form x <= f(y)
obstacles = {
%    @(x,y) y <= x.^2 - 5;                 % Example: parabolic obstacle
%    @(x,y) y >= sin(x) - 3 && y <= 5;     % Example: sine wave obstacle
    @(x,y) (x >= -8.5 && x <= 7.5) && (y >= -8 && y <= -7.5);  % Example: rectangular obstacle
    @(x,y) (x >= 1 && x <= 1.5) && (y >= -10 && y <= -8);
    @(x,y) (x >= 7 && x <= 7.5) && (y >= -7.5 && y <= -5.5);
    @(x,y) (x >= -3 && x <= -2.5) && (y >= -7.5 && y <= -5.5);
    @(x,y) (x >= -3 && x <= 5) && (y >= -5.5 && y <= -5);
    @(x,y) (x >= -10 && x <= -8) && (y >= -5.5 && y <= -5);
    @(x,y) (x >= -10 && x <= -5) && (y >= -5.5 && y <= -5);
    @(x,y) (x >= -5.5 && x <= -5) && (y >= -5 && y <= -2);
    @(x,y) (x >= -10 && x <= -3) && (y >= 0.5 && y <= 1);
    @(x,y) (x >= -3 && x <= -2.5) && (y >= -2 && y <= 1);
    @(x,y) (x >= -3 && x <= 7.5) && (y >= -2.5 && y <= -2);
    @(x,y) (x >= 0 && x <= 7.5) && (y >= 0.5 && y <= 1);
    @(x,y) (x >= 0 && x <= 0.5) && (y >= 1 && y <= 4);
    @(x,y) (x >= -3 && x <= 10) && (y >= 3.5 && y <= 4);
    @(x,y) (x >= -10 && x <= -6) && (y >= 3.5 && y <= 4);
    @(x,y) (x >= -7.5 && x <= 7.5) && (y >= 7.5 && y <= 8);
    @(x,y) (x >= -1.5 && x <= -1) && (y >= 8 && y <= 10);
    @(x,y) (x >= 7 && x <= 7.5) && (y >= 6 && y <= 7.5);
    @(x,y) (x >= 4.5 && x <= 5) && (y >= 4 && y <= 6);
    @(x,y) (x >= -3 && x <= 1) && (y >= 5.5 && y <= 6);
    @(x,y) (x >= -8 && x <= -7.5) && (y >= 6 && y <= 8);
    @(x,y) (x >= -8 && x <= -2) && (y >= 5.5 && y <= 6);
    @(x,y) x <= -10; 
    @(x,y) x >= 10; 
    @(x,y) y <= -10; 
    @(x,y) y >= 10; 
    
};

% Define start and end points
start_point = [0, -10];
end_point = [0, 10];

% Visualization parameters
resolution = 0.1;  % Resolution for plotting obstacles
marker_size = 10;

% Plot maze boundaries
figure(1);
hold on;


g = zeros((xmax-xmin), (ymax-ymin)); %define size for the grid based on maze boundaries, fill with zeros


x_range = xmin:1:xmax;
y_range = ymin:1:ymax;
for i = 1:length(obstacles)
    for j = 1:length(x_range)

        for k = 1:length(y_range)
            if obstacles{i}(x_range(j), y_range(k)) && x_range(j)>=-10 && x_range(j)<=10 && y_range(k)>=-10 && y_range(k)<=10
                %add the transformed coordinates to the grid. The original
                %boundaries were [-10,10], transform it to [1,21] for
                %easier indexing
                newx = (x_range(j)+xmax)+1;
                newy = (y_range(k)+ymax)+1;
                g(newx, newy) = inf;
            end
        end
    end
end

%tranform start and end points as well from its original [-10,10]
%representation to a [1,21] representation so it can be plotted in g
snewx = (start_point(1)+xmax)+1;
snewy = (start_point(2)+ymax)+1;
g(snewx, snewy) = 0;


enewx = (end_point(1)+xmax)+1;
enewy = (end_point(2)+ymax)+1;
g(enewx, enewy) = 0;


%check if position (i,j) is in range of the maze and not colliding with a
%wall. This is the "sensing" part of the algorithm
function y = ok(i,j)
    global g
    global vis
    y = i >= 1 && j >= 1 && i <= 21 && j <= 21 && g(i,j) ~= inf;
    return
end


MIN = inf; %minimum path to get from start-end
d = dictionary(); %stores a key-value where key = path length, value = list of coordinates from start to end that got that path length
vis = false((xmax-xmin)*2, (ymax-ymin)*2); %visited array so we don't revisited infinite times

function floodfill(i, j, steps) %floodfill 
    global g
    if ok(i,j) == false %check if current position is in range and not colliding with a wall
        return
    end

    if steps+1 < g(i,j) || (g(i,j) == 0) %check if current path is a better path

        %update current cell and recurse to cardinal neighbors
        g(i,j) = steps;
        floodfill(i+1,j,steps+1);
        floodfill(i-1,j,steps+1);
        floodfill(i,j+1,steps+1);
        floodfill(i,j-1,steps+1);
    end
    
end

floodfill(snewx, snewy, 0);

done = false
p = [];
g(snewx, snewy) = 0;

function pdfs(i,j,bp) %DFS
    global done
    global p
    global snewx
    global snewy
    global g
    global vis
    if done == true %if found path, exit the function
        return
    end
    if ok(i,j) == false %check if current position is in range of the maze and not colliding with a wall
        return
    end
    
    bp = [[i,j];bp]; %add current cell to path
    
    if i == snewx && j == snewy %if at the start (we started at the end in this case), remember the path and exit the function
        disp("RAN")
        done = true;
        p = bp;
        return;
    end
    
    vis(i,j) = true; %set as visited

    %check if value at neighbor is one less than value at current cell and
    %if that neighbor is valid
    if (ok(i+1,j) && g(i+1,j) < g(i,j))
        pdfs(i+1,j,bp)
    end
    if (ok(i-1,j) && g(i-1,j) < g(i,j))
        pdfs(i-1,j,bp)
    end
    if (ok(i,j+1) && g(i,j+1) < g(i,j))
        pdfs(i,j+1,bp)
    end
    if (ok(i,j-1) && g(i,j-1) < g(i,j))
        pdfs(i,j-1,bp)
    end
    vis(i,j)=false; %backtrack
end

pdfs(enewx, enewy, []);
tra = p - 11; %reset the coordinates to its original representation. Go from [1,21] back to [-10,10]

% Plot obstacles
x_range = xmin:resolution:xmax;
y_range = ymin:resolution:ymax;

for i = 1:length(obstacles)
    for j = 1:length(x_range)
        for k = 1:length(y_range)
            if obstacles{i}(x_range(j), y_range(k)) && x_range(j)>-10 && x_range(j)<10 && y_range(k)>-10 && y_range(k)<10
                plot(x_range(j), y_range(k), 'r.', 'MarkerSize', 10);
            end
        end
    end
end

% Plot start and end points
plot(start_point(1), start_point(2), 'go', 'MarkerSize', marker_size, 'LineWidth', 2);  % Green circle for start
plot(end_point(1), end_point(2), 'ro', 'MarkerSize', marker_size, 'LineWidth', 2);  % Red circle for end

% Adjust axes and grid
axis equal;
grid on;
ax = gca;
ax.LineWidth = 1;
xlim([xmin, xmax]);
ylim([ymin, ymax]);
title('COSMOS 24 Cluster 11 Maze');

%split tra (the path of original coordinates) into x and y for the
%proportional controller
x=[];
y=[];
for i=1:length(tra)
    x(end+1) = tra(i,1);
    y(end+1) = tra(i,2);
end

%assign hyperparameters to optimize which points to take
x = x(1:1:end);
x(end+1) = 0;
y = y(1:1:end);
y(end+1) = 10;

px = [0]; %current x
py = [-10]; %current y
v = []; %velocity
theta = [pi/2]; %current heading
theta_want = []; %desired heading
h = .1; %step size
sum = 0;
kh = 0.3; %proportional constant
kv = 0.5; %proportional constant
w = [];
e = [2];
tol = 0.001;

for i = 1:length(x)
    e(end+1) = real(sqrt((x(i) - px(end))^2 + (y(i) - py(end))^2));
    while e(end) > 0.8
        theta_want(end+1) = atan2((y(i) - py(end)), (x(i) - px(end)));
        w(end+1) = real(kh * (theta_want(end) - theta(end)));
        theta(end+1) = theta(end) + w(end);
        disp(sqrt((x(i) - px(end))^2 + (y(i) - py(end))^2));
        e(end + 1) = real(sqrt((x(i) - px(end))^2 + (y(i) - py(end))^2));
        v(end+1) = kv * e(end); 
        costerm = cos(theta(end));
        if abs(costerm - 0) < tol
            costerm = 0;
        end
        sinterm = sin(theta(end));
        if abs(sinterm - 0) < tol
            sinterm = 0;
        end
        px(end+1) = px(end) + h * v(end) * costerm;
        py(end+1) = py(end) + h * v(end) * sinterm;
        sum = sum + 1;
        if mod(sum, 25) == 0
            pause(0.0000001)
            plot(px,py, 'LineWidth', 2.0);
        end
    end
end

plot(px,py, 'LineWidth', 2.0);