%% This the main code of your simulation
clear; close;

% Define test parameters
numVertices = [64, 100, 225];%, 169, 225];%, 150, 200];
distanceRange = [2000, 200000];% 5000, 10000];%; 600, 1000; 0, 1000];
numMaps = 10;
numTests = 100;

% Initialize variables to store results
results = cell(12,1); % A cell to contain 8 results (times, nodes etc.) and 1 for test parameters
results{1} = zeros(length(numVertices), size(distanceRange, 1), numMaps*numTests); % Optimal Path length
results{2} = zeros(length(numVertices), size(distanceRange, 1), numMaps*numTests); % Time Cai
results{3} = zeros(length(numVertices), size(distanceRange, 1), numMaps*numTests); % Time Ours
results{4} = zeros(length(numVertices), size(distanceRange, 1), numMaps*numTests); % Time No Budget
results{5} = zeros(length(numVertices), size(distanceRange, 1), numMaps*numTests); % Time No Refinement
results{6} = zeros(length(numVertices), size(distanceRange, 1), 2); % Nodes Ours
results{7} = zeros(length(numVertices), size(distanceRange, 1), 2); % Nodes No Budget
results{8} = zeros(length(numVertices), size(distanceRange, 1), 2); % Nodes No Refinement
results{9} = numVertices;
results{10} = distanceRange;
results{11} = zeros(length(numVertices), size(distanceRange, 1), numMaps*numTests); % Time Fast
results{12} = zeros(length(numVertices), size(distanceRange, 1), numMaps*numTests); % Path Length Fast
results{13} = zeros(length(numVertices), size(distanceRange, 1), 2); % Nodes Fast method
results{14} = zeros(length(numVertices), size(distanceRange, 1), numMaps*numTests); % Stores the total wait time for the optimal path
results{15} = zeros(length(numVertices), size(distanceRange, 1), numMaps*numTests); % Stores the number of nodes in the optimal path
results{16} = zeros(length(numVertices), size(distanceRange, 1), numMaps*numTests); % Stores the time to compute edgeAvail matrix

euclid = 1; % Determines whether to use euclideam distances or driving distances
% Loop over test parameters
for i = 1:length(numVertices)
    for j = 1:size(distanceRange,1)
        disp([i,j])
        % Initialize result variables for current test case
        pathTime2 = zeros(1, numMaps*numTests); % Time of final path
        pathTimeFast2 = zeros(1, numMaps*numTests);
        timeCai2 = zeros(1, numMaps*numTests); % Computation time Cai 1998
        timeOur2 = zeros(1, numMaps*numTests);
        timeAllT2 = zeros(1, numMaps*numTests);
        timeNoRef2 = zeros(1, numMaps*numTests);
        timeFast2 = zeros(1, numMaps*numTests);


        nodesOur2 = zeros(2, numMaps);
        nodesAllT2 = zeros(2, numMaps);
        nodesNoRef2 = zeros(2, numMaps);
        nodesFast2 = zeros(2, numMaps);

        % Loop over maps
        k = 1;
        nNodes = numVertices(i); %randi([30,50]); % Nodes in the graph are <1,2,...,nNodes>
        xCentre = 1220333.064;
        yCentre = 769592.061;
        xMax = xCentre + 5000; % Now, everything is in metres
        yMax = yCentre + 5000;
        xMin = xCentre - 5000; % Now, everything is in metres
        yMin = yCentre - 5000;

        [E, D, posX, posY] = getMap(nNodes, xMax, yMax, xMin, yMin, 'del');

        % Decide whether to use new points or old ones
        if euclid == 0
            toDo = input("To use the already saved points, enter 1, to generate new points enter 2, to use euclidean distances enter 3: ")
            if toDo == 0
                error("Terminate now!")
            elseif toDo == 1
                T = readtable(['Maps/routePoints/routePoints' num2str(nNodes) '.csv']);
                posX = T.X;
                posY = T.Y;
            elseif toDo == 2
                % Save into csv file to be used by QGIS
                T = array2table([posX, posY, (1:numel(posX))']);
                T.Properties.VariableNames(1:3) = {'X','Y','id'};
                writetable(T, ['Maps/routePoints/temp/routePoints' num2str(nNodes) '.csv'])
                writetable(T(1:33,:), 'Maps/routePoints/temp/routePoints100_1_33.csv')
                writetable(T(34:66,:), 'Maps/routePoints/temp/routePoints100_34_67.csv')
                writetable(T(67:100,:), 'Maps/routePoints/temp/routePoints100_67_100.csv')
                done = input("Enter 1 when done with generating the distance matrix: ")
                if done == 0
                    error("Terminate now!")
                end
                % Go to QGIS and generate the distance matrix
                % Read the distances after processing with QGIS
                T1 = readtable('Maps/routePoints/temp/distMatrix100_1.csv');
                T2 = readtable('Maps/routePoints/temp/distMatrix100_2.csv');
                T3 = readtable('Maps/routePoints/temp/distMatrix100_3.csv');
                T = [T1; T2; T3];
                writetable(T, 'Maps/routePoints/temp/distMatrix100.csv')
            else
                euclid = 1;
            end
        end
        
        if euclid == 0 
            T = readtable(['Maps/routePoints/distMatrix' num2str(nNodes) '.csv']);
            D = T.DIST_KM * 1000;
            D = reshape(D, [nNodes, nNodes])' .* E;
        end
        while k <= numMaps % Now, the maps represent different speed and waiting restrictions.
            %  E = triu(E);
            f = 1; % Remember that distances are in metres % Let's keep time in minutes for now
            autoSpeed = 40*rand(nNodes, nNodes) + 0; % In metres per minute
            teleSpeed = autoSpeed + 35*rand(nNodes, nNodes) + 5;
            
            % Set if some nodes are blocked (e.g., road closure, construction zone etc.)
            blockedNodes = randperm(nNodes, floor(nNodes*0.1));
            for n = blockedNodes
                autoSpeed(n,:) = 0.1;
            end

            % Get edge duration matrices based on speeds
            A = f*ceil(D ./ autoSpeed);
            B = f*ceil(D ./ teleSpeed);
            [eX,eY] = find(E);
            edges = [eX eY];
            edgesCai = [eX eY; eX eY+nNodes];

            % Maximum waiting times at each node
            maxWaits = f*randi([0, 15], [nNodes, 1]); %11*ones(nNodes,1);

            oprAvail = f*getOprAvail(40, 200, 10, 100, 1000); % (minUpTime, maxUpTime, minDownTime, maxDownTime, maxTime)
            edgeAvail = getEdgeBudgetsAll(E, B, oprAvail);
                
            % Initialize more result storing variables
            pathTime = zeros(1, numTests); % Time of final path
            timeCai = zeros(1, numTests); % Computation time TDSP-CWT algorithm
            timeOur = zeros(1, numTests);
            timeAllT = zeros(1, numTests);
            timeNoRef = zeros(1, numTests);
            timeFast = zeros(1, numTests);
            pathTimeFast = zeros(1, numTests); % Time of final path under fast method

            nodesOur = zeros(2, numTests);
            nodesAllT = zeros(2, numTests);
            nodesNoRef = zeros(2, numTests);
            nodesFast = zeros(2, numTests);

            for itr = 1:numTests
                % Start and Goal vertices
                startVertex = blockedNodes(1);
                while any(ismember(blockedNodes, startVertex))
                    startVertex = randi(nNodes);
                end
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
                    getNewMap = 1;
                    disp('No node in given range.')
                    break;
                end

                pathAuto = staticDijkstras(edges, startVertex, goalVertex, A);
                if isempty(pathAuto)
                    disp("No path exists!")
                    break;
                end
                maxTime = pathAuto(end,2);

                %% Robot path
                % A robot's path is a kx4 matrix. Specified as a list of nodes to traverse, arrival times, wait times at those nodes and mode of operation.

                pathTele = staticDijkstras(edges, startVertex, goalVertex, B);
                if isempty(pathTele)
                    haha = 1;
                end
                distToGoal = staticDijkstras([edges(:,2) edges(:,1)], goalVertex, startVertex, B', 'all');
                tic
                edgeAvailTime = toc;
                if maxTime > 1000
                    aaa= 1;
                end
                results{16}(i,j,(k-1)*numTests+itr) = edgeAvailTime;

                tic
                [pathAllT, Q1, Qexp1, ~] = getRobotPath(E, startVertex, goalVertex, A, B, oprAvail, edgeAvail, maxWaits, pathAuto(end,2), distToGoal, 'allT');
                timeAllT(itr) = toc;% + edgeAvailTime;
                results{4}(i,j,(k-1)*numTests+itr) = toc;
                nodesAllT(:,itr) = [size(Q1,1); size(Qexp1,1)];

                tic % This one's not optimal (Greedy)
                [pathFast, Q4, Qexp4]  = fastestTaskDijkstras(startVertex, goalVertex, A, B, edgeAvail, maxWaits, distToGoal, 0);
                timeFast(itr) = toc;% + edgeAvailTime;
                results{11}(i,j,(k-1)*numTests+itr) = toc;
                pathTimeFast(itr) = pathFast(end,2);
                nodesFast(:,itr) = [size(Q4,1); size(Qexp4,1)];

                tic
                [pathOur, Q3, Qexp3, cc] = getRobotPath(E, startVertex, goalVertex, A, B, oprAvail, edgeAvail, maxWaits, pathAuto(end,2), distToGoal, 'our');
                timeOur(itr) = toc;% + edgeAvailTime;
                results{3}(i,j,(k-1)*numTests+itr) = toc;
                nodesOur(:,itr) = [size(Q3,1); size(Qexp3,1)];
                pathTime(itr) = pathOur(end,2);
                results{14}(i,j,(k-1)*numTests+itr) = sum(pathOur(:,3)); % Store total wait time for this instance
                results{15}(i,j,(k-1)*numTests+itr) = size(pathOur,1); % Store the number of vertices in the path.
                if size(pathOur,1) == 0
                    w = 1;
                end
            end
            if getNewMap == 1
                continue;
            end
            pathTime2((k-1)*numTests+1:k*numTests) = pathTime;
            pathTimeFast2((k-1)*numTests+1:k*numTests) = pathTimeFast;
            timeOur2((k-1)*numTests+1:k*numTests) = timeOur;
            timeAllT2((k-1)*numTests+1:k*numTests) = timeAllT;
            timeNoRef2((k-1)*numTests+1:k*numTests) = timeNoRef;
            timeFast2((k-1)*numTests+1:k*numTests) = timeFast;
            timeCai2(k) = mean(timeCai);
            timeOur2(k) = mean(timeOur);
            timeAllT2(k) = mean(timeAllT);
            timeNoRef2(k) = mean(timeNoRef);
            timeFast2(k) = mean(timeFast);


            nodesOur2(:, k) = mean(nodesOur, 2);
            nodesAllT2(:, k) = mean(nodesAllT, 2);
            nodesNoRef2(:, k) = mean(nodesNoRef, 2);
            nodesFast2(:, k) = mean(nodesFast, 2);

            k = k + 1;
        end
        % Store results for the particular test condition
        results{1}(i,j,:) = pathTime2;
        results{6}(i,j,:) = mean(nodesOur2, 2);
        results{7}(i,j,:) = mean(nodesAllT2, 2);
        results{8}(i,j,:) = mean(nodesNoRef2, 2);
        results{12}(i,j,:) = pathTimeFast2;
        results{13}(i,j,:) = mean(nodesFast2, 2);

    end
end
save("results.mat")

%% Plotting
load("results.mat")
close;
%%
figure(1);
subplot(1,2,1);
x = 1:size(results{10},1); % x values
yLim = 1.1*max(max([results{2} results{3} results{4} results{5} results{11}]));
for i = 1%:size(results{9},2)
    timeCai = mean(results{2},3);
    timeOurs = mean(results{3},3);
    timeNoBud = mean(results{4},3);
    timeNoRef = mean(results{5},3);
    timeFast = mean(results{11},3);

    % Plot the bar graph
    Y = [timeCai, timeNoBud, timeOurs, timeFast];
    hBar = bar(Y);
    % Add labels and legend
    xlabel('No. of Vertices');
    set(gca,'xticklabel',{'64', '100', '225', '400'})
    set(gca, 'YScale', 'log')
    hBar(3).FaceColor = [0.4660 0.6740 0.1880];
    y1 = Y;
    for k1 = 1:size(y1,2)
        ctr(k1,:) = bsxfun(@plus, hBar(1).XData, hBar(k1).XOffset');    % Note: ‘XOffset’ Is An Undocumented Feature, This Selects The ‘bar’ Centres
        ydt(k1,:) = hBar(k1).YData;                                     % Individual Bar Heights
        text(ctr(k1,end), ydt(k1,end), sprintfc('%.4f', ydt(k1,end)), 'HorizontalAlignment','center', 'VerticalAlignment','bottom', 'FontSize',12, 'Color','b')
    end
    set(gca,'FontSize',14)
    legend({'TCSP-CWT', 'Time-expanded A*', 'Proposed', 'Greedy'}, 'Location', 'northwest');
    ylabel('Mean Computation Time (sec)');
end

%% Computation time vs path length
subplot(1,2,2)
ii = 4;
pathOurs = results{1}(:,:,:);
pathOurs = reshape(results{1}(:,:,:), [1, numel(pathOurs)]);
timeCai = reshape(results{2}(:,:,:), [1, numel(pathOurs)]);
timeOurs = reshape(results{3}(:,:,:), [1, numel(pathOurs)]);
timeNoBud = reshape(results{4}(:,:,:), [1, numel(pathOurs)]);
timeNoRef = reshape(results{5}(:,:,:), [1, numel(pathOurs)]);
timeFast = reshape(results{11}(:,:,:), [1, numel(pathOurs)]);
timeEdge = 0;
plot(pathOurs, timeCai, '.')
hold on
plot(pathOurs, timeNoBud+timeEdge, 'o')
plot(pathOurs, timeOurs+timeEdge, '*', 'Color', [0.4660 0.6740 0.1880])
plot(pathOurs, timeFast+timeEdge, 's')
hold off
legend({'TCSP-CWT', 'Time-expanded A*', 'Proposed', 'Greedy'}, 'Location', 'northwest');
ylabel('Computation Time (sec)');
xlabel('Optimal Path Duration (min)');

set(gca,'FontSize',14)

%% Computation time with number of edges in the optimal path
figure(11);
pathOurs = results{1}(:,:,:);
pathEdges = reshape(results{15}(:,:,:), [1, numel(pathOurs)]);
timeCai = reshape(results{2}(:,:,:), [1, numel(pathOurs)]);
timeOurs = reshape(results{3}(:,:,:), [1, numel(pathOurs)]);
timeNoBud = reshape(results{4}(:,:,:), [1, numel(pathOurs)]);
timeNoRef = reshape(results{5}(:,:,:), [1, numel(pathOurs)]);
timeFast = reshape(results{11}(:,:,:), [1, numel(pathOurs)]);
timeEdge = reshape(results{16}(:,:,:), [1, numel(pathOurs)]);

plot(pathEdges, timeCai, '.')
hold on
plot(pathEdges, timeNoBud+timeEdge, 'o')
plot(pathEdges, timeOurs+timeEdge, '*', 'Color', [0.4660 0.6740 0.1880])
plot(pathEdges, timeFast+timeEdge, 's')
hold off
legend({'TCSP-CWT', 'Time-expanded A*', 'Proposed', 'Greedy'}, 'Location', 'northwest');
ylabel('Computation Time (sec)');
xlabel('Number of edges in optimal path');

set(gca,'FontSize',16)
%% Nodes Explored
figure(2);
nodesOurs = mean(results{6}(:,:,:),2);
nodesNoBud = mean(results{7}(:,:,:),2);
nodesNoRef = mean(results{8}(:,:,:),2);
nodesFast = mean(results{13}(:,:,:),2);
hi = sum([nodesNoBud; nodesNoRef; nodesOurs; nodesFast], 3);
yLim = 1.1*max(max(hi));
for i = 1:size(results{9},2)
    subplot(1,size(results{9},2),i)

    nodesOurs = reshape(mean(results{6}(i,:,:),2), [1 2]);
    nodesNoBud = reshape(mean(results{7}(i,:,:),2), [1 2]);
    nodesNoRef = reshape(mean(results{8}(i,:,:),2), [1 2]);
    nodesFast = reshape(mean(results{13}(i,:,:),2), [1 2]);

    hi = [nodesNoBud; nodesOurs; nodesFast];
    bar(hi,'stacked')

    % Add labels and legend
    title("No. of Vertices = " + results{9}(i))
    set(gca,'xticklabel',{'Time-expanded A*','Proposed', 'Greedy'})
    ylim([0 yLim])
    if i == 1 %size(results{9},2)
        ylabel('Total Nodes Generated');
        legend('Nodes not explored', 'Nodes explored')
    else
        set(gca,'YTickLabel',[]);
    end
    set(gca,'FontSize',16)
end


figure(3);
x = 1:size(results{10},1); % x values
% yLim = 1.1*max(abs(results{12}-results{1})./results{1},[],'all');
for i = 1:size(results{9},2)
    for j = 1:size(distanceRange,1)
        subplot(size(distanceRange,1),size(results{9},2),i+(j-1)*size(results{9},2))

        pathFast = results{12}(i,j,:);
        pathOurs = results{1}(i,j,:);

        % Plot the graph
        diff = (pathFast-pathOurs)./pathOurs;
        diff = sort(diff);
        a = diff(floor(numel(diff)*0.8):end);
        histogram(diff,10)
        [mean(diff), mean(diff(diff>0)), mean(a)]
        %     bar(x, [pathFast; pathOurs]');
        %     ylim([0 yLim])
        title("No. of Vertices = " + results{9}(i))
        % Add labels and legend
            xlabel('Relative Path Length');
        if i == size(results{9},2)
            ignore = 1;
        elseif i == 1
            %         legend('Fast', 'Ours');
            ylabel('Number of Instances');
        end
    end
end

%% All Compare solution (path) length
figure(4);

pathOurs = results{1}(:,:,:);
pathOurs = reshape(pathOurs, [1, numel(pathOurs)]);
pathFast = results{12}(:,:,:);
pathFast = reshape(pathFast, [1, numel(pathFast)]);

plot((pathOurs), (pathFast), 'o','MarkerFaceColor', 'k')
hold on
plot([0 max(pathFast)], [0 max(pathFast)], 'LineWidth', 3)
xlabel('Optimal Path Duration (min)');
ylabel('Greedy Path Duration (min)');
hold off
xlim([0 max(pathFast)])
ylim([0 max(pathFast)])
set(gca,'FontSize',16)

function oprAvail = getOprAvail(minUpTime, maxUpTime, minDownTime, maxDownTime, maxTime)
oprAvail = [];
t = 0;
while t < maxTime
    dt = randi([minDownTime, maxDownTime]);
    oprAvail(end+1) = t+dt;
    t = t+dt;

    ut = randi([minUpTime, maxUpTime]);
    oprAvail(end+1) = t+ut;
    t = t+ut;
end
end

function newE = getEdgeBudgets(E, B, T, oprAvail)
newE = -1*ones(size(B,1),size(B,2), T+1);
for i = 1:ceil(size(oprAvail,2)/2)
    tCurr = oprAvail(2*i-1); % This only works if oprAvaila has even number of elements, i.e., in the end we get unavailable operator
    if tCurr > T
        break;
    end
    changeTime = oprAvail(2*i);
    for t = tCurr:min(T, changeTime-1)
        % For all elements less than changeTime-t-1 add an edge
        value = changeTime-1-t; % -1 because the availabilty changes at changeTime, so we can only assist until 1 unit before
        currE = (B <= value) .* (value-B) .* E;
        newE(:,:,t+1) = currE - (B>value); % subtract (B>value) so that value becomes -1 when not available, because 0 can be available with no remaining budget
    end
end
end

function newE = getEdgeBudgetsAll(E, B, oprAvail)
newE = -1*ones(size(B,1),size(B,2), oprAvail(end));
for i = 1:ceil(size(oprAvail,2)/2)
    tCurr = oprAvail(2*i-1); % This only works if oprAvaila has even number of elements, i.e., in the end we get unavailable operator
    changeTime = oprAvail(2*i);
    for t = tCurr:changeTime-1
        % For all elements less than changeTime-t-1 add an edge
        value = changeTime-1-t; % -1 because the availabilty changes at changeTime, so we can only assist until 1 unit before
        currE = (B <= value) .* (value-B) .* E;
        newE(:,:,t+1) = currE - (B>value); % subtract (B>value) so that value becomes -1 when not available, because 0 can be available with no remaining budget
    end
end
end

function [newE, oprAvailIdx] = getEdgeBudgetsSaved(E, B, T, oprAvail, Tmax, prevEdgeAvail, oprAvailIdxMax)
newE = -1*ones(size(B,1),size(B,2), T+1);
if Tmax >= T
    newE = prevEdgeAvail(:,:,1:T+1);
    oprAvailIdx = oprAvailIdxmax;
elseif Tmax > 0
    newE(:,:,1:T+1) = prevEdgeAvail(:,:,1:T+1);
    for i = oprAvailIdxMax:ceil(size(oprAvail,2)/2)
        tCurr = oprAvail(2*i-1); % This only works if oprAvail has even number of elements, i.e., in the end we get unavailable operator
        if tCurr > T
            break;
        end
        changeTime = oprAvail(2*i);
        for t = tCurr:changeTime-1
            % For all elements less than changeTime-t-1 add an edge
            value = changeTime-1-t; % -1 because the availabilty changes at changeTime, so we can only assist until 1 unit before
            currE = (B <= value) .* (value-B) .* E;
            newE(:,:,t+1) = currE - (B>value); % subtract (B>value) so that value becomes -1 when not available, because 0 can be available with no remaining budget
        end
    end
else
    for i = 1:ceil(size(oprAvail,2)/2)
        tCurr = oprAvail(2*i-1); % This only works if oprAvail has even number of elements, i.e., in the end we get unavailable operator
        if tCurr > T
            break;
        end
        changeTime = oprAvail(2*i);
        for t = tCurr:changeTime-1
            % For all elements less than changeTime-t-1 add an edge
            value = changeTime-1-t; % -1 because the availabilty changes at changeTime, so we can only assist until 1 unit before
            currE = (B <= value) .* (value-B) .* E;
            newE(:,:,t+1) = currE - (B>value); % subtract (B>value) so that value becomes -1 when not available, because 0 can be available with no remaining budget
        end
    end
end
end

function newE = getEdgeBudgetsBlock(E, B, T, oprAvail)
newE = -1*ones(size(B,1),size(B,2), T+1);
    
for i = 1:ceil(size(oprAvail,2)/2)
    tCurr = oprAvail(2*i-1); % This only works if oprAvaila has even number of elements, i.e., in the end we get unavailable operator
    if tCurr > T
        break;
    end
    changeTime = oprAvail(2*i);
    t_range = tCurr:changeTime-1;
    value_range = changeTime-t_range-1;
    block = reshape(repmat(value_range,size(B,1)*size(B,2),1),size(B,1),size(B,2),[]); % Creates copies of value ranges.
    ECopy = repmat(E, 1,1,size(block,3));
    BCopy = repmat(B, 1,1,size(block,3));
    block = (block .* ECopy) - BCopy;
    block = max(block, -1); %max(block, -1);
    newE(:,:,tCurr+1:changeTime) = block;
end

end

function newE = edgeAvailability(B, T, oprAvail)
newE = 0*ones(size(B,1),size(B,2), T+1);
for i = 1:ceil(size(oprAvail,2)/2)
    tCurr = oprAvail(2*i-1);
    if tCurr > T
        break;
    end
    changeTime = oprAvail(2*i);
    for t = tCurr:changeTime-1
        % For all elements less than changeTime-t-1 add an edge
        value = changeTime-1-t; % -1 because the availabilty changes at changeTime, so we can only assist until 1 unit before
        newE(:,:,t+1) = B<=value;
    end
end
end
