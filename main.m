%% This the main code of your simulation
clear; close;

textprogressbar('Progress: ');
% Define test parameters
numVertices = [150];%, 100];
distanceRange = [0, 1500];% 500, 1000];
numMaps = 1;
numTests = 1;

% Initialize variables to store results
results = cell(length(numVertices), length(distanceRange), 8); % 8 is the number of quantities we want to record

% Loop over test parameters
for i = 1:length(numVertices)
    for j = 1:size(distanceRange,1)
        % Initialize result variables for current test case
        pathTime2 = zeros(1, numMaps); % Time of final path
        timeCai2 = zeros(1, numMaps); % Computation time Cai 1998
        timeOur2 = zeros(1, numMaps);
        timeAllT2 = zeros(1, numMaps);
        timeNoRef2 = zeros(1, numMaps);

        nodesOur2 = zeros(2, numMaps);
        nodesAllT2 = zeros(2, numMaps);
        nodesNoRef2 = zeros(2, numMaps);
        
        % Loop over maps
        k = 1;
        while k <= numMaps
            nNodes = numVertices(i); %randi([30,50]); % Nodes in the graph are <1,2,...,nNodes>
            xMax = 1000;
            yMax = 1000;
            [E, posX, posY] = getMap(nNodes, xMax, yMax);
            f = 1;
            B = f*randi([10,50], nNodes, nNodes) .* E;
            A = B + f*(randi([10,50], nNodes, nNodes) .* E);
            [eX,eY] = find(E);
            edges = [eX eY];
            edgesCai = [eX eY; eX eY+nNodes];

            % Initialize more result storing variables
            pathTime = zeros(1, numTests); % Time of final path
            timeCai = zeros(1, numTests); % Computation time Cai 1998
            timeOur = zeros(1, numTests);
            timeAllT = zeros(1, numTests);
            timeNoRef = zeros(1, numTests);

            nodesOur = zeros(2, numTests);
            nodesAllT = zeros(2, numTests);
            nodesNoRef = zeros(2, numTests);
            for itr = 1:numTests
                %% Start and Goal vertices
                startVertex = randi(nNodes);
                goalVertex = 0;
                for g = 1:nNodes
                    dist = norm([posX(startVertex) posY(startVertex)] - [posX(g) posY(g)]);
                    if g ~= startVertex && dist >= distanceRange(j,1) && dist <= distanceRange(j,2)
                        goalVertex = g;
                        break;
                    end
                end
                getNewMap = 0;
                if goalVertex == 0
%                     drawmap(xMax, yMax, posX, posY, startVertex, goalVertex, distanceRange(j,1), distanceRange(j,2))
                    getNewMap = 1;
                    disp('No node in given range.')
                    break;
                end

                %% max waiting times at each node
                maxWaits = randi([0, 30], [nNodes, 1]); %11*ones(nNodes,1);
                pathAuto = staticDijkstras(edges, startVertex, goalVertex, A);
                if isempty(pathAuto)
                    disp("No path exists!")
                    break;
                end
                maxTime = pathAuto(end,2); % To limit the search while testing. Can be removed once I fix the issue with oprAvail

                %% Operator availability
                % oprAvail = [2, 10, 15, 30, 35, 1000]; % Times when opr availability changes. Default start is not available, if first element is 0 mean we start with available.
                oprAvail = f*[sort(randi(maxTime, 1, floor(maxTime/40)))];
                if mod(size(oprAvail,2),2) == 1
                    oprAvail = [oprAvail, maxTime];
                end
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
                pathCai = TCSPCai1998(edgesCai, startVertex, goalVertex, A, B, oprAvail, maxWaits, pathAuto(end,2));
                % visualizeExploration(Q, Qexp, posX, posY, 'r', 1)
                timeCai(itr) = toc;
                pathTime(itr) = pathCai(end,2);

%                 tic
%                 [pathAllT, Q, Qexp] = getRobotPathAllTime(edges, startVertex, goalVertex, A, B, oprAvail, maxWaits, pathAuto(end,2), distToGoal);
%                 timeAllT(itr) = toc;
%                 nodesAllT(:,itr) = [size(Q,1); size(Qexp,1)];

                tic
                [pathNoRef, Q, Qexp] = getRobotPathNoRefine(edges, startVertex, goalVertex, A, B, oprAvail, maxWaits, pathAuto(end,2), distToGoal);
                timeNoRef(itr) = toc;
                nodesNoRef(:,itr) = [size(Q,1); size(Qexp,1)];

                tic
                [path, Q, Qexp] = getRobotPath(edges, startVertex, goalVertex, A, B, oprAvail, maxWaits, pathAuto(end,2), distToGoal);
                timeOur(itr) = toc;
                nodesOur(:,itr) = [size(Q,1); size(Qexp,1)];

                % disp(itr)
                textprogressbar(itr/numTests*100);
                % visualizeExploration(Q, Qexp, posX, posY, 'b', 2)
                if ~isequal(pathCai(end,2), path(end,2))
                    drawmap(xMax, yMax, posX, posY)
                    pathCai
                    path
                    disp("Wrong!!!")
                end
            end
            if getNewMap == 1
                continue;
            end
            pathTime2(k) = mean(pathTime);
            timeCai2(k) = mean(timeCai);
            timeOur2(k) = mean(timeOur);
            timeAllT2(k) = mean(timeAllT);
            timeNoRef2(k) = mean(timeNoRef);

            nodesOur2(:, k) = mean(nodesOur, 2);
            nodesAllT2(:, k) = mean(nodesAllT, 2);
            nodesNoRef2(:, k) = mean(nodesNoRef, 2);

            k = k + 1;

        end
        % Store results for the particular test condition
        results{i,j,1} = mean(pathTime2);
        results{i,j,2} = mean(timeCai2);
        results{i,j,3} = mean(timeOur2);
        results{i,j,4} = mean(timeAllT2);
        results{i,j,5} = mean(timeNoRef2);
        results{i,j,6} = mean(nodesOur2, 2);
        results{i,j,7} = mean(nodesAllT2, 2);
        results{i,j,8} = mean(nodesNoRef2, 2);
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

function drawmap(xMax, yMax, x, y, s, g, minD, maxD)
%% Plot the Delaunay triangulation
xMid = floor(xMax/2);
yMid = floor(yMax/2);
figure;
axis equal
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
r1 = maxD;
pos = [x(s)-r1 y(s)-r1 2*r1 2*r1];
rectangle('Position',pos,'Curvature',[1 1], 'FaceColor',[0.92 .92 1.0],'EdgeColor','b', 'LineWidth',1)
r1 = minD;
pos = [x(s)-r1 y(s)-r1 2*r1 2*r1];
rectangle('Position',pos,'Curvature',[1 1], 'FaceColor',[1.0 .92 0.92],'EdgeColor','r', 'LineWidth',1)
% alpha(.95)

triplot(DT, x, y, 'Color', 'k', 'LineWidth', 0.5);
hold on;

% Plot the points
plot(x, y, '.', 'Color', 'r', 'MarkerSize', 10);
plot(x(s), y(s), '.', 'Color', 'b', 'MarkerSize', 15);

% Set the axis limits
xlim([0 xMax]);
ylim([0 yMax]);

% Add a title and labels
title('City Road Network');
xlabel('X');
ylabel('Y');
end

