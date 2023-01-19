%% This the main code of your simulation
% 1) Define the Graph: Edges that robot can travel on
% 2) Specify travel time under autonomous and assisted operation
% 3) Specify max allowed waiting for each edge (I think we should use matrices for all these)
% 4) Operator availability function using a list of times when availability changes 
clear;
nItr = 100;
pathTime = zeros(1, nItr);
timeCai = zeros(1, nItr);
timeOur = zeros(1, nItr);
timeAllT = zeros(1, nItr);
timeNoRef = zeros(1, nItr);

nodesOur = zeros(2, nItr);
nodesAllT = zeros(2, nItr);
nodesNoRef = zeros(2, nItr);

textprogressbar('Progress: ');
for itr = 1:nItr

nRobots = 1;
% A = magic(4);
% A = A>10; % Matrix with binary values. 0 = No edge, 1 = Edge present
% A = A - diag(diag(A)); % Set diagonal to 0, meaning no self loop

nNodes = 50; %randi([30,50]); % Nodes in the graph are <1,2,...,nNodes>
xMax = 1000;
yMax = 1000;
[E, posX, posY] = getMap(nNodes, xMax, yMax);

% E = rand(nNodes)>0.1; % Will be used to create A and B
% E = E - diag(diag(E)); % Diagonal elements are 0 because self-loops are not allowed.
% for j = 1:nNodes
%     for i = 1:j
%         if abs(i-j) > ceil(nNodes/3)
%             E(i,j) = 0;
% %         else
% %             E(i,j) = E(j,i);
%         end
%     end
% end

% E = triu(E); % Trying upper diagonal matrix to prevent cycles.
% E(:,end) = 0;
f = 1; %randi([1 4]);
B = f*randi([10,50], nNodes, nNodes) .* E;
A = B + f*(randi([10,50], nNodes, nNodes) .* E);
Diff = A - B;

% E = [0 1 1 0 0 0;
%      0 0 0 1 0 0;
%      0 1 0 1 1 0;
%      0 0 0 0 1 1;
%      0 0 0 1 0 0;
%      0 0 0 0 0 0];
% B = 1*[0 4 9 0 0 0;
%      0 0 0 6 0 0;
%      0 99 0 2 3 0;
%      0 0 0 0 2 3;
%      0 0 0 2 0 0;
%      0 0 0 0 0 0];
% A = 1*[0 8 14 0 0 0;
%      0 0 0 12 0 0;
%      0 99 0 3 9 0;
%      0 0 0 0 3 99;
%      0 0 0 3 0 0;
%      0 0 0 0 0 0];


[eX,eY] = find(E);
edges = [eX eY];
edgesCai = [eX eY; eX eY+nNodes];
% edges = cell(nNodes,1);%[x y];
% for i = 1:nNodes
%     y = find(E(i,:));
%     edges(i,1) = {y};
% end
% edges = [1,2; 1,3; 2,4; 3,2; 3,4; 3,5; 4,5; 4,6; 5,4];
% teleTimes = [4, 2, 8, 2, 12, 2, 2, 3, 2]; %randi(10, [size(edges,1), 1]);
% autoTimes = [5, 5, 10, 3, 13, 3, 3, 7, 3]; %teleTimes + randi(5, [size(edges,1), 1]);
% 
% %% Preparing travel times matrices. A = autonomous times, B = teleop times
% x = edges(:,1);
% y = edges(:,2);
% idx = sub2ind(size(E), x, y);
% A = 1000*ones(nNodes);
% B = A;
% 
% A(idx) = autoTimes;
% B(idx) = teleTimes;

%% Start and Goal vertices
startVertex = 1;
goalVertex = nNodes;

%% max waiting times at each node
Dif = max(Diff');
maxWaits = randi([0, 30], [nNodes, 1]); %11*ones(nNodes,1);
pathAuto = staticDijkstras(edges, startVertex, goalVertex, A);
if isempty(pathAuto)
    disp("No path exists!")
    break;
end
maxTime = pathAuto(end,2); % To limit the search while testing. Can be removed once I fix the issue with oprAvail
 
%% Operator availability
% oprAvail = [2, 10, 15, 30, 35, 1000]; % Times when opr availability changes. Default start is not available, if first element is 0 mean we start with available.
oprAvail = f*[sort(randi(maxTime, 1, floor(maxTime/40))), maxTime];
% currA = 0;
% oprAvail = [];
% p = 0.6;
% minUpTime = 30;
% minDownTime = 10;
% for i = 1:maxTime
%     if isempty(oprAvail) || (currA == 1 && i-oprAvail(end) >= minUpTime) || (currA == 0 && i-oprAvail(end) >= minDownTime)
%         nextA = rand() < p;
%         if nextA ~= currA
%             oprAvail = [oprAvail, i];
%             currA = nextA;
%         end
%     end
% end
%% Robot path
% A robot's path is a kx4 matrix. Specified as a list of nodes to traverse, arrival times, wait times at those nodes and mode of operation. 

%% TODO: A function to compute path
% tic

% load('variables');


path = staticDijkstras(edges, startVertex, goalVertex, B);
% pathAuto = staticDijkstras(edges, startVertex, goalVertex, A);
distToGoal = staticDijkstras([edges(:,2) edges(:,1)], goalVertex, startVertex, B', 'all');
% toc
tic
pathCai = TCSPCai1998(edgesCai, goalVertex, A, B, oprAvail, maxWaits, pathAuto(end,2));
% visualizeExploration(Q, Qexp, posX, posY, 'r', 1)
timeCai(itr) = toc;
pathTime(itr) = pathCai(end,2);

tic
[pathAllT, Q, Qexp] = getRobotPathAllTime(edges, startVertex, goalVertex, A, B, oprAvail, maxWaits, pathAuto(end,2), distToGoal);
timeAllT(itr) = toc;
nodesAllT(:,itr) = [size(Q,1); size(Qexp,1)];

tic
[pathNoRef, Q, Qexp] = getRobotPathNoRefine(edges, startVertex, goalVertex, A, B, oprAvail, maxWaits, pathAuto(end,2), distToGoal);
timeNoRef(itr) = toc;
nodesNoRef(:,itr) = [size(Q,1); size(Qexp,1)];

tic
[path, Q, Qexp] = getRobotPath(edges, startVertex, goalVertex, A, B, oprAvail, maxWaits, pathAuto(end,2), distToGoal);
timeOur(itr) = toc;
nodesOur(:,itr) = [size(Q,1); size(Qexp,1)];

% disp(itr)
textprogressbar(itr/nItr*100);
% visualizeExploration(Q, Qexp, posX, posY, 'b', 2)
if ~isequal(pathCai(end,2), path(end,2))
    drawmap(xMax, yMax, posX, posY)
    pathCai
    path
    disp("Wrong!!!")
end
end

% Close the progress bar
textprogressbar('done');
fprintf('\n');

%% Plotting
plot(pathTime, timeCai, '*')
hold on
plot(pathTime, timeOur, 'o')
plot(pathTime, timeAllT, 's')
plot(pathTime, timeNoRef, '.')
xlabel('Duration of optimal path (sec)', 'FontSize', 18)
ylabel('Computation time (sec)', 'FontSize', 18)
legend('Cai 1998', 'Ours', 'All Times', 'No Refinement')
title("No. of Nodes = " + nNodes)
ax = gca;
ax.FontSize = 15;

% Fit a linear curve to the data for timeCai
p = polyfit(pathTime, timeCai, 1);

% Evaluate the curve at a fine grid of points
t = linspace(min(pathTime), max(pathTime), 100);
y = polyval(p, t);

% Plot the curve
plot(t, y, '--', 'LineWidth', 2);

% Fit a linear curve to the data for timeOur
p = polyfit(pathTime, timeOur, 1);

% Evaluate the curve at a fine grid of points
t = linspace(min(pathTime), max(pathTime), 100);
y = polyval(p, t);

% Plot the curve
plot(t, y, '--', 'LineWidth', 2);

%% Plot 2
plot(pathTime, nodesAllT(1,:), '*')
hold on
% plot(pathTime, nodesAllT(2,:), '*')
plot(pathTime, nodesNoRef(1,:), 'o')
% plot(pathTime, nodesNoRef(2,:), 'o')
plot(pathTime, nodesOur(1,:), 's')
% plot(pathTime, nodesOur(2,:), 's')
legend('All Times', 'No Refinement', 'Ours')

% g = 1;
% reshaped = reshape(nodesAllT, [2, 25, g]);
% mean_matrix = mean(reshaped, 2);
% m1 = reshape(mean_matrix, [2, g]);
% 
% reshaped = reshape(nodesNoRef, [2, 25, g]);
% mean_matrix = mean(reshaped, 2);
% m2 = reshape(mean_matrix, [2, g]);
% 
% reshaped = reshape(nodesOur, [2, 25, g]);
% mean_matrix = mean(reshaped, 2);
% m3 = reshape(mean_matrix, [2, g]);

%%
nAll = mean(nodesAllT, 2);
nNoR = mean(nodesNoRef, 2);
nOur = mean(nodesOur, 2);
hi = [nAll'; nNoR'; nOur'];
bar(hi,'stacked')
set(gca,'xticklabel',{'All Time','No Refinement','Ours'},'FontSize', 18)
legend('Nodes generated', 'Nodes explored')

%%
x = 1:g;
bar(x, m1, 'stacked');
hold on;

% Plot the second bar in each group
bar(x, m2, 'stacked');

% Plot the third bar in each group
bar(x, m3, 'stacked');

% % Plot the number of nodes explored by each algorithm
hold on
errorbar(x, nodesAllT(1,:), nodesAllT(2,:), nodesAllT(1,:), 'b', 'LineWidth', 2, 'LineStyle', 'none');
errorbar(x, nodesNoRef(1,:), nodesNoRef(2,:), nodesNoRef(1,:), 'r', 'LineWidth', 2, 'LineStyle', 'none');
errorbar(x, nodesOur(1,:), nodesOur(2,:), nodesOur(1,:), 'k', 'LineWidth', 2, 'LineStyle', 'none');
% 
% % Plot the number of nodes generated by each algorithm
% stem(x, nodesAllT(2,:), '--b', 'LineWidth', 2);
% stem(x, nodesOur(2,:), '--r', 'LineWidth', 2);
%%

function textprogressbar(c)
%% Initialization
persistent strCR;           %   Carriage return pesistent variable

% Vizualization parameters
strPercentageLength = 10;   %   Length of percentage string (must be >5)
strDotsMaximum      = 10;   %   The total number of dots in a progress bar

%% Main 

if isempty(strCR) && ~ischar(c)
    % Progress bar must be initialized with a string
%     error('The text progress must be initialized with a string');
fprintf('%s','Progress2: ');
    strCR = -1;
elseif isempty(strCR) && ischar(c)
    % Progress bar - initialization
    fprintf('%s',c);
    strCR = -1;
elseif ~isempty(strCR) && ischar(c)
    % Progress bar  - termination
    strCR = [];  
    fprintf([c '\n']);
elseif isnumeric(c)
    % Progress bar - normal progress
    c = floor(c);
    percentageOut = [num2str(c) '%%'];
    percentageOut = [percentageOut repmat(' ',1,strPercentageLength-length(percentageOut)-1)];
    nDots = floor(c/100*strDotsMaximum);
    dotOut = ['[' repmat('.',1,nDots) repmat(' ',1,strDotsMaximum-nDots) ']'];
    strOut = [percentageOut dotOut];
    
    % Print it on the screen
    if strCR == -1
        % Don't do carriage return during first run
        fprintf(strOut);
    else
        % Do it during all the other runs
        fprintf([strCR strOut]);
    end
    
    % Update carriage return
    strCR = repmat('\b',1,length(strOut)-1);
    
else
    % Any other unexpected input
    error('Unsupported argument type');
end

end

function drawmap(xMax, yMax, x, y)
%% Plot the Delaunay triangulation
xMid = floor(xMax/2);
yMid = floor(yMax/2);
figure;
% axis equal
hold on
DT = delaunay(x, y);
% rad = xMax/2.1;
% pos = [xMid-rad yMid-rad 2*rad 2*rad]; 
% rectangle('Position',pos,'Curvature',[1 1], 'FaceColor',[1 .9 .9],'EdgeColor','r')
% alpha(.5)
% 
% rad = xMax/3;
% pos = [xMid-rad yMid-rad 2*rad 2*rad]; 
% rectangle('Position',pos,'Curvature',[1 1], 'FaceColor',[0.9 1 .9],'EdgeColor','g')
% 
% 
rad = xMax/5;
pos = [xMid-rad yMid-rad 2*rad 2*rad]; 
rectangle('Position',pos,'Curvature',[1 1], 'FaceColor',[0.92 .92 1.0],'EdgeColor','b', 'LineWidth',1)
% alpha(.95)

triplot(DT, x, y, 'Color', 'k', 'LineWidth', 0.5);
hold on;

% Plot the points
plot(x, y, '.', 'Color', 'r', 'MarkerSize', 10);
plot(x(1), y(1), '.', 'Color', 'b', 'MarkerSize', 15);

% Set the axis limits
xlim([0 xMax]);
ylim([0 yMax]);

% Add a title and labels
title('City Road Network');
xlabel('X');
ylabel('Y');
end
%% Simulate the system and get finish times of all robots for all tasks
% completionTimes = getCompletionTimes(nRobots, A, B, startState, path, oprAvail);
