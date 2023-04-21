%% This the main code of your simulation
clear; close;

% textprogressbar('Progress: ');
% Define test parameters
numVertices = [64, 100, 225];%, 169, 225];%, 150, 200];
distanceRange = [2000, 200000];% 5000, 10000];%; 600, 1000; 0, 1000];
numMaps = 10;
numTests = 100;

% Initialize variables to store results
results = cell(12,1); % A cell to contain 8 results (times, nodes etc.) and 1 for test parameters
% cell(length(numVertices), length(distanceRange), 8); % 8 is the number of quantities we want to record
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
                don = input("Enter 1 when done with generating the distance matrix: ")
                if don == 0
                    error("Terminate now!")
                end
                % Go to QGIS and generate the distance matrix
            else
                euclid = 1;
            end
        end
        % Read the distances after processing with QGIS
%         T1 = readtable('Maps/routePoints/temp/distMatrix100_1.csv');
%         T2 = readtable('Maps/routePoints/temp/distMatrix100_2.csv');
%         T3 = readtable('Maps/routePoints/temp/distMatrix100_3.csv');
%         T = [T1; T2; T3];
%         writetable(T, 'Maps/routePoints/temp/distMatrix100.csv')
        if euclid == 0 
            T = readtable(['Maps/routePoints/distMatrix' num2str(nNodes) '.csv']);
            D = T.DIST_KM * 1000;
            D = reshape(D, [nNodes, nNodes])' .* E;
        end
        while k <= numMaps % Now, the maps represent different speed and waiting restrictions.
            %  E = triu(E);
            f = 1; % Remember that distances are in metres % Let's keep time in minutes for now
%             aaaa = zeros(10000,1);
%             aaa = zeros(10000,1);
%             for kk = 1:10000
%                 aaaa(kk) = 40*rand() + 1; % max(1, normrnd(20, 0.5));
%                 aaa(kk) = aaaa(kk) + 20*rand() + 10; %max(1, normrnd(5, 0.5)); %10*lognrnd(0, 0.5);
%             end
%             histogram(aaaa)
%             hold on
%             histogram(aaa)
%             figure(2);
%             plot(aaa, aaaa, '.')
%             axis equal
%             hold on
%             plot(aaaa, aaaa)
            
%             autoSpeed = max(1, normrnd(20, 0.5, [nNodes, nNodes])); %10*lognrnd(0, 0.5, [nNodes, nNodes]) + 10; %40*rand(nNodes, nNodes) + 1; % In metres per minute
%             teleSpeed = autoSpeed + max(1, normrnd(5, 0.5, [nNodes, nNodes])); %+ 20*rand(nNodes, nNodes) + 10; %20*rand(nNodes, nNodes) + 10; % (3/4)*autoSpeed + 20;% + randi([10,50], nNodes, nNodes);
%            GOOD autoSpeed = 40*rand(nNodes, nNodes) + 1; % In metres per minute
%            GOOD teleSpeed = autoSpeed + 20*rand(nNodes, nNodes) + 10; %20*rand(nNodes, nNodes) + 10; % (3/4)*autoSpeed + 20;% + randi([10,50], nNodes, nNodes);
            autoSpeed = 40*rand(nNodes, nNodes) + 0; % In metres per minute
            teleSpeed = autoSpeed + 35*rand(nNodes, nNodes) + 5;
            
            % Set if some nodes are blocked (e.g., road closure, construction zone etc.)
            % We must make sure that the start node is not blocked.
            blockedNodes = 0;
%             blockedNodes = randperm(nNodes, floor(nNodes*0.1));
%             for n = blockedNodes
%                 autoSpeed(n,:) = 0.1; %randi([10,20], 1, nNodes);
%             end

            % Get edge duration matrices based on speeds
            A = f*ceil(D ./ autoSpeed);
            B = f*ceil(D ./ teleSpeed);
            %             B = f*randi([10,50], nNodes, nNodes) .* E;
            %             A = B + f*(randi([10,50], nNodes, nNodes) .* E);
            [eX,eY] = find(E);
            edges = [eX eY];
            edgesCai = [eX eY; eX eY+nNodes];

            % max waiting times at each node
            maxWaits = f*randi([0, 15], [nNodes, 1]); %11*ones(nNodes,1);

            % Operator availability
            % oprAvail = [2, 10, 15, 30, 35, 1000]; % Times when opr availability changes. Default start is not available, if first element is 0 mean we start with available.
            %             oprAvail = f*[sort(randi(600, 1, 14))]; % Should be even number of elements. So that the operator availability ends when t -> Inf
            
            oprAvail = f*getOprAvail(40, 200, 10, 100, 1000); % (minUpTime, maxUpTime, minDownTime, maxDownTime, maxTime)
%             oprAvail = f*getOprAvail(1, 5, 1, 2, 50);
            edgeAvail = getEdgeBudgetsAll(E, B, oprAvail);
                
            % Initialize more result storing variables
            pathTime = zeros(1, numTests); % Time of final path
            timeCai = zeros(1, numTests); % Computation time Cai 1998
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
                    % drawmap(E, xMax, yMax, posX, posY, startVertex, goalVertex, distanceRange(j,1), distanceRange(j,2))
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

                %                 oprAvail = f*[sort(randi(maxTime, 1, floor(maxTime/40)))];
                %                 if mod(size(oprAvail,2),2) == 1
                %                     oprAvail = [oprAvail, 2*maxTime];
                %                 end

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


                pathTele = staticDijkstras(edges, startVertex, goalVertex, B);
                if isempty(pathTele)
                    haha = 1;
                end
                % pathAuto = staticDijkstras(edges, startVertex, goalVertex, A);
                distToGoal = staticDijkstras([edges(:,2) edges(:,1)], goalVertex, startVertex, B', 'all');
                tic
%                 edgeAvail = getEdgeBudgets(E, B, maxTime, oprAvail);
                edgeAvailTime = toc;
                if maxTime > 1000
                    aaa= 1;
                end
%                 tic
%                 edgeAvail2 = allEdgeAvail(:,:,1:maxTime+1);
%                 edgeAvailTime2 = toc;
                results{16}(i,j,(k-1)*numTests+itr) = edgeAvailTime;

%                 tic
%                 edgeAvailCai = edgeAvailability(B, maxTime, oprAvail);
%                 pathCai = TCSPCai1998(edgesCai, startVertex, goalVertex, A, B, oprAvail, edgeAvailCai, maxWaits, pathAuto(end,2));
%                 % visualizeExploration(Q, Qexp, posX, posY, 'r', 1)
%                 timeCai(itr) = toc;
%                 results{2}(i,j,(k-1)*numTests+itr) = toc;
% 
                tic
                %                 [pathAllT, Q, Qexp] = getRobotPathAllTime(edges, startVertex, goalVertex, A, B, oprAvail, maxWaits, pathAuto(end,2), distToGoal);
                [pathAllT, Q1, Qexp1, ~] = getRobotPath(E, startVertex, goalVertex, A, B, oprAvail, edgeAvail, maxWaits, pathAuto(end,2), distToGoal, 'allT');
                timeAllT(itr) = toc;% + edgeAvailTime;
                results{4}(i,j,(k-1)*numTests+itr) = toc;
                nodesAllT(:,itr) = [size(Q1,1); size(Qexp1,1)];

%                 tic
%                 %                 [pathNoRef, Q, Qexp] = getRobotPathNoRefine(edges, startVertex, goalVertex, A, B, oprAvail, maxWaits, pathAuto(end,2), distToGoal);
%                 [pathNoRef, Q2, Qexp2, ~] = getRobotPath(E, startVertex, goalVertex, A, B, oprAvail, edgeAvail, maxWaits, pathAuto(end,2), distToGoal, 'noref');
%                 timeNoRef(itr) = toc;% + edgeAvailTime;
%                 results{5}(i,j,(k-1)*numTests+itr) = toc;
%                 nodesNoRef(:,itr) = [size(Q2,1); size(Qexp2,1)];

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
%                 [size(Q3,1), size(Qexp3,1), cc]
                if size(pathOur,1) == 0
                    w = 1;
                end


                % visualizeExploration(Q, Qexp, posX, posY, 'b', 2)
%                 if ~isequal(pathOur(end,2), pathAllT(end,2)) %, pathNoRef(end,2))
%                     aa = 1
%                 %                if pathFast(end,2) < pathOur(end,2) || timeFast(itr) > timeOur(itr)
% %                                     drawmap(E, xMax, yMax, posX, posY, startVertex, goalVertex, distanceRange(j,1), distanceRange(j,2))
%                 %                     drawmap(E, xMax, yMax, posX, posY, startVertex, goalVertex, 0, 0, pathOur)
%                 %                     pathCai
%                 %                     pathOur
% %                                     disp("Wrong!!!")
%                 end
                %                 drawmap(E, xMax, yMax, posX, posY, startVertex, goalVertex, 0, 0, pathOur)
                %                 [pathFast(end,2), pathOur(end,2)]
            end
            if getNewMap == 1
                continue;
            end
            pathTime2((k-1)*numTests+1:k*numTests) = pathTime;
            pathTimeFast2((k-1)*numTests+1:k*numTests) = pathTimeFast;
%             timeCai2((k-1)*numTests+1:k*numTests) = timeCai;
%             timeOur2((k-1)*numTests+1:k*numTests) = timeOur;
%             timeAllT2((k-1)*numTests+1:k*numTests) = timeAllT;
%             timeNoRef2((k-1)*numTests+1:k*numTests) = timeNoRef;
%             timeFast2((k-1)*numTests+1:k*numTests) = timeFast;
%             timeCai2(k) = mean(timeCai);
%             timeOur2(k) = mean(timeOur);
%             timeAllT2(k) = mean(timeAllT);
%             timeNoRef2(k) = mean(timeNoRef);
%             timeFast2(k) = mean(timeFast);


            nodesOur2(:, k) = mean(nodesOur, 2);
            nodesAllT2(:, k) = mean(nodesAllT, 2);
            nodesNoRef2(:, k) = mean(nodesNoRef, 2);
            nodesFast2(:, k) = mean(nodesFast, 2);

            k = k + 1;

            % disp(itr)
            %             textprogressbar(itr/numTests*100);
        end
        % Store results for the particular test condition
        results{1}(i,j,:) = pathTime2;
%         results{2}(i,j,:) = (timeCai2);
%         results{3}(i,j,:) = (timeOur2);
%         results{4}(i,j,:) = (timeAllT2);
%         results{5}(i,j,:) = (timeNoRef2);
        results{6}(i,j,:) = mean(nodesOur2, 2);
        results{7}(i,j,:) = mean(nodesAllT2, 2);
        results{8}(i,j,:) = mean(nodesNoRef2, 2);

%         results{11}(i,j,:) = (timeFast2);
        results{12}(i,j,:) = pathTimeFast2;
        results{13}(i,j,:) = mean(nodesFast2, 2);

    end
end
% Close the progress bar
% textprogressbar('done');
% fprintf('\n');

% save("variables.mat")
save("results.mat")

%% Plotting
% Computation Time (nVertices = x)
load("results.mat")
close;
%%
figure(1);
subplot(1,2,1);
x = 1:size(results{10},1); % x values
yLim = 1.1*max(max([results{2} results{3} results{4} results{5} results{11}]));
for i = 1%:size(results{9},2)
%     subplot(1,size(results{9},2),i)

%     timeCai = results{2}(:,:);
%     timeOurs = results{3}(:,:);
%     timeNoBud = results{4}(:,:);
%     timeNoRef = results{5}(:,:);
%     timeFast = results{11}(:,:);
    timeCai = mean(results{2},3);
    timeOurs = mean(results{3},3);
    timeNoBud = mean(results{4},3);
    timeNoRef = mean(results{5},3);
    timeFast = mean(results{11},3);
%     timeEdge = mean(results{16},3);

    % Plot the bar graph
%     bar([timeCai, timeNoBud, timeNoRef, timeOurs, timeFast]);
%     Y = [timeCai, timeNoBud+timeEdge, timeOurs+timeEdge, timeFast+timeEdge];
    Y = [timeCai, timeNoBud, timeOurs, timeFast];
    hBar = bar(Y);
%     ylim([0 yLim])
%     title("No. of Vertices = " + results{9}(i))
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
% figure(11);
subplot(1,2,2)
ii = 4;
pathOurs = results{1}(:,:,:);
pathOurs = reshape(results{1}(:,:,:), [1, numel(pathOurs)]);
timeCai = reshape(results{2}(:,:,:), [1, numel(pathOurs)]);
timeOurs = reshape(results{3}(:,:,:), [1, numel(pathOurs)]);
timeNoBud = reshape(results{4}(:,:,:), [1, numel(pathOurs)]);
timeNoRef = reshape(results{5}(:,:,:), [1, numel(pathOurs)]);
timeFast = reshape(results{11}(:,:,:), [1, numel(pathOurs)]);
% timeEdge = reshape(results{16}(:,:,:), [1, numel(pathOurs)]); % Time taken to compute edgeAvail matrix
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

%     hi = [nodesNoBud; nodesNoRef; nodesOurs; nodesFast];
    hi = [nodesNoBud; nodesOurs; nodesFast];
    bar(hi,'stacked')

    % Add labels and legend
    title("No. of Vertices = " + results{9}(i))
%     xlabel('Method');
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


% %% Nodes explored all
% figure(22);
% s = size(results{9},2);
% nodesOurs = sum(reshape(mean(results{6}(:,:,:),2), [s 2]), 2);
% nodesNoBud = sum(reshape(mean(results{7}(:,:,:),2), [s 2]), 2);
% nodesNoRef = sum(reshape(mean(results{8}(:,:,:),2), [s 2]), 2);
% nodesFast = sum(reshape(mean(results{13}(:,:,:),2), [s 2]), 2);
% hi = [nodesNoBud, nodesOurs, nodesFast];
% yLim = 1.1*max(max(hi));
% 
% hp = bar(hi);
% cm = ['r','g','b','y','k']; % or replace with the desired colormap
% hatchfill2(hp(1),'single','HatchAngle',-45);
% hatchfill2(hp(2),'cross','HatchAngle',45);
% hatchfill2(hp(3),'single','HatchAngle',45);
% for b = 1:numel(hp)
%     hp(b).FaceColor = 'none';
% end
% hold on
% nodesOurs = reshape(mean(results{6}(:,:,1),2), [s 1]);
% nodesNoBud = reshape(mean(results{7}(:,:,1),2), [s 1]);
% nodesNoRef = reshape(mean(results{8}(:,:,1),2), [s 1]);
% nodesFast = reshape(mean(results{13}(:,:,1),2), [s 1]);
% hi = [nodesNoBud, nodesOurs, nodesFast];
% 
% hp = bar(hi);
% hold off
% set(gca, 'YScale', 'log')
% set(gca,'xticklabel',{'64','100', '225'})
% xlabel('Number of Vertices');
% ylabel('Nodes Generated (Mean)');
% % legend('Nodes not explored', 'Nodes explored')
% legend({'Time-expanded A*', 'Proposed', 'Greedy'}, 'Location', 'northwest');
% 
%     
% % ylim([0 yLim])
% 
%% Compare solution (path) length
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
%             set(gca,'xticklabel',{'49', '64'})
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

% plot(log(pathOurs), log(pathFast), 'o')
plot((pathOurs), (pathFast), 'o','MarkerFaceColor', 'k')
hold on
plot([0 max(pathFast)], [0 max(pathFast)], 'LineWidth', 3)
xlabel('Optimal Path Duration (min)');
ylabel('Greedy Path Duration (min)');
hold off
xlim([0 max(pathFast)])
ylim([0 max(pathFast)])
set(gca,'FontSize',16)

%% OLD Path Length comparison (mean and 20%)
% axis equal
% meanOurs = mean(pathOurs);
% meanFast = mean(pathFast);
% 
% sortedOurs = sort(pathOurs, 'descend');
% sortedFast = sort(pathFast, 'descend');
% meanOurs20 = mean(sortedOurs(1:floor(numel(sortedOurs)*0.2)));
% meanFast20 = mean(sortedFast(1:floor(numel(sortedFast)*0.2)));
% 
% diff = (pathFast-pathOurs);%./pathOurs;
% sortedDiff = sortrows([diff' pathOurs' pathFast'], 1, 'descend');
% meanOursDiff20 = mean(sortedDiff(1:floor(numel(sortedOurs)*0.2), 2));
% meanFastDiff20 = mean(sortedDiff(1:floor(numel(sortedFast)*0.2), 3));
% 
% diff = (pathFast-pathOurs)./pathOurs;
% sortedDiff = sortrows([diff' pathOurs' pathFast'], 1, 'descend');
% meanOursDiff30 = mean(sortedDiff(1:floor(numel(sortedOurs)*0.3), 2));
% meanFastDiff30 = mean(sortedDiff(1:floor(numel(sortedFast)*0.3), 3));
% 
% % Plot the graph
% bar([(meanFast-meanOurs)/meanOurs; (meanFastDiff20-meanOursDiff20)/meanOursDiff20]);
% set(gca,'xticklabel',{'Mean', 'Worst 20%'})
% yLim = max(yLim, (meanFastDiff20-meanOursDiff20)/meanOursDiff20);
% title("Average and Worst 20% comparison")
% if i == 1
%     ylabel('Relative path length');
% end
% 
% 
% 
%% Violin Compare solution (path) length
% figure(5);
% 
% pathOurs = results{1}(:,:,:);
% pathOurs = reshape(pathOurs, [1, numel(pathOurs)]);
% pathFast = results{12}(:,:,:);
% pathFast = reshape(pathFast, [1, numel(pathFast)]);
% 
% % Plot the graph
% diff = (pathFast-pathOurs)./pathOurs;
% % violin(diff')
% boxplot(diff')
% % set(gca,'xticklabel',{'Mean', 'Worst 20%'})
% % yLim = max(yLim, (meanFastDiff20-meanOursDiff20)/meanOursDiff20);
% title("Performance comparison")
% if i == 1
%     ylabel('Relative path length');
% end
% done = 1;
% 
% 



%%
%% Plotting (OLD)
% % Computation Time (nVertices = x)
% figure(1);
% x = 1:size(results{10},1); % x values
% yLim = 1.1*max(max([results{2} results{3} results{4} results{5} results{11}]));
% for i = 1:size(results{9},2)
%     subplot(1,size(results{9},2),i)
% 
%     timeCai = results{2}(i,:);
%     timeOurs = results{3}(i,:);
%     timeNoBud = results{4}(i,:);
%     timeNoRef = results{5}(i,:);
%     timeFast = results{11}(i,:);
% 
%     % Plot the bar graph
%     bar(x, [timeCai; timeNoBud; timeNoRef; timeOurs; timeFast]');
%     ylim([0 yLim])
% %     title("No. of Vertices = " + results{9}(i))
%     % Add labels and legend
%     xlabel('Distance Range');
%     set(gca,'xticklabel',{'0-300', '300-600', '600-1000'})
%     if i == size(results{9},2)
%         ignore = 1;
%     elseif i == 1
%         legend('Cai', 'No Budget', 'No Refinement', 'Ours', 'Fast');
%         ylabel('Computation Time');
%     end
% end
% 
% 
% % Nodes Explored
% figure(2);
% x = ['No Budget', 'No Refinement', 'Ours']; % x values
% nodesOurs = mean(results{6}(:,:,:),2);
% nodesNoBud = mean(results{7}(:,:,:),2);
% nodesNoRef = mean(results{8}(:,:,:),2);
% nodesFast = mean(results{13}(:,:,:),2);
% hi = sum([nodesNoBud; nodesNoRef; nodesOurs; nodesFast], 3);
% yLim = 1.1*max(max(hi));
% for i = 1:size(results{9},2)
%     subplot(1,size(results{9},2),i)
% 
%     nodesOurs = reshape(mean(results{6}(i,:,:),2), [1 2]);
%     nodesNoBud = reshape(mean(results{7}(i,:,:),2), [1 2]);
%     nodesNoRef = reshape(mean(results{8}(i,:,:),2), [1 2]);
%     nodesFast = reshape(mean(results{13}(i,:,:),2), [1 2]);
% 
%     hi = [nodesNoBud; nodesNoRef; nodesOurs; nodesFast];
%     bar(hi,'stacked')
% 
%     % Add labels and legend
%     title("No. of Vertices = " + results{9}(i))
%     xlabel('Method');
%     set(gca,'xticklabel',{'No Budget','No Refinement','Ours', 'Fast'})
%     ylim([0 yLim])
%     if i == 1 %size(results{9},2)
%         ylabel('Nodes');
%         legend('Nodes generated', 'Nodes explored')
%     end
% end
% 
% %% Compare solution (path) length
% figure(3);
% x = 1:size(results{10},1); % x values
% % yLim = 1.1*max(abs(results{12}-results{1})./results{1},[],'all');
% for i = 1:size(results{9},2)
%     for j = 1:size(distanceRange,1)
%         subplot(size(distanceRange,1),size(results{9},2),i+(j-1)*size(results{9},2))
% 
%         pathFast = results{12}(i,j,:);
%         pathOurs = results{1}(i,j,:);
% 
%         % Plot the graph
%         diff = (pathFast-pathOurs)./pathOurs;
%         diff = sort(diff);
%         a = diff(floor(numel(diff)*0.8):end);
%         histogram(diff)
%         [mean(diff), mean(diff(diff>0)), mean(a)]
%         %     bar(x, [pathFast; pathOurs]');
%         %     ylim([0 yLim])
%         title("No. of Vertices = " + results{9}(i))
%         % Add labels and legend
%         %     xlabel('Distance Range');
%         %     set(gca,'xticklabel',{'0-300', '300-600', '600-1000'})
%         if i == size(results{9},2)
%             ignore = 1;
%         elseif i == 1
%             %         legend('Fast', 'Ours');
%             ylabel('Path Length');
%         end
%     end
% end


%%

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

% plot(pathTime, timeCai, '*')
% hold on
% plot(pathTime, timeOur, 'o')
% plot(pathTime, timeAllT, 's')
% plot(pathTime, timeNoRef, '.')
% xlabel('Duration of optimal path (sec)', 'FontSize', 18)
% ylabel('Computation time (sec)', 'FontSize', 18)
% legend('Cai 1998', 'Ours', 'All Times', 'No Refinement')
% title("No. of Nodes = " + nNodes)
% ax = gca;
% ax.FontSize = 15;
%
% % Fit a linear curve to the data for timeCai
% p = polyfit(pathTime, timeCai, 1);
%
% % Evaluate the curve at a fine grid of points
% t = linspace(min(pathTime), max(pathTime), 100);
% y = polyval(p, t);
%
% % Plot the curve
% plot(t, y, '--', 'LineWidth', 2);
%
% % Fit a linear curve to the data for timeOur
% p = polyfit(pathTime, timeOur, 1);
%
% % Evaluate the curve at a fine grid of points
% t = linspace(min(pathTime), max(pathTime), 100);
% y = polyval(p, t);
%
% % Plot the curve
% plot(t, y, '--', 'LineWidth', 2);
%
% %% Plot 2
% plot(pathTime, nodesAllT(1,:), '*')
% hold on
% % plot(pathTime, nodesAllT(2,:), '*')
% plot(pathTime, nodesNoRef(1,:), 'o')
% % plot(pathTime, nodesNoRef(2,:), 'o')
% plot(pathTime, nodesOur(1,:), 's')
% % plot(pathTime, nodesOur(2,:), 's')
% legend('All Times', 'No Refinement', 'Ours')
%
% % g = 1;
% % reshaped = reshape(nodesAllT, [2, 25, g]);
% % mean_matrix = mean(reshaped, 2);
% % m1 = reshape(mean_matrix, [2, g]);
% %
% % reshaped = reshape(nodesNoRef, [2, 25, g]);
% % mean_matrix = mean(reshaped, 2);
% % m2 = reshape(mean_matrix, [2, g]);
% %
% % reshaped = reshape(nodesOur, [2, 25, g]);
% % mean_matrix = mean(reshaped, 2);
% % m3 = reshape(mean_matrix, [2, g]);
%
% %%
% nAll = mean(nodesAllT, 2);
% nNoR = mean(nodesNoRef, 2);
% nOur = mean(nodesOur, 2);
% hi = [nAll'; nNoR'; nOur'];
% bar(hi,'stacked')
% set(gca,'xticklabel',{'All Time','No Refinement','Ours'},'FontSize', 18)
% legend('Nodes generated', 'Nodes explored')
%
% %%
% x = 1:g;
% bar(x, m1, 'stacked');
% hold on;
%
% % Plot the second bar in each group
% bar(x, m2, 'stacked');
%
% % Plot the third bar in each group
% bar(x, m3, 'stacked');
%
% % % Plot the number of nodes explored by each algorithm
% hold on
% errorbar(x, nodesAllT(1,:), nodesAllT(2,:), nodesAllT(1,:), 'b', 'LineWidth', 2, 'LineStyle', 'none');
% errorbar(x, nodesNoRef(1,:), nodesNoRef(2,:), nodesNoRef(1,:), 'r', 'LineWidth', 2, 'LineStyle', 'none');
% errorbar(x, nodesOur(1,:), nodesOur(2,:), nodesOur(1,:), 'k', 'LineWidth', 2, 'LineStyle', 'none');
% %
% % % Plot the number of nodes generated by each algorithm
% % stem(x, nodesAllT(2,:), '--b', 'LineWidth', 2);
% % stem(x, nodesOur(2,:), '--r', 'LineWidth', 2);
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
%     if tCurr > T
%         break;
%     end
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
%         currE = (B <= value) .* (value-B);
%         newE(:,:,t+1) = currE; % t+1 because we start with t=0
        newE(:,:,t+1) = B<=value;
    end
end
end
