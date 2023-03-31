%% This the main code of your simulation
clear; close all;

% textprogressbar('Progress: ');
% Define test parameters
numVertices = [49];%, 100, 144];
distanceRange = [00, 100000];% 500, 1000];
numMaps = 500;
numTests = 1;

% Initialize variables to store results
results = cell(6,1); % A cell to contain 8 results (times, nodes etc.) and 1 for test parameters
% cell(length(numVertices), length(distanceRange), 8); % 8 is the number of quantities we want to record
results{1} = zeros(length(numVertices), size(distanceRange, 1), numMaps*numTests); % Optimal Path length
results{2} = zeros(length(numVertices), size(distanceRange, 1)); % Time Ours
results{3} = zeros(length(numVertices), size(distanceRange, 1), 2); % Nodes Ours
results{4} = zeros(length(numVertices), size(distanceRange, 1), numMaps*numTests); % Path Length Fast
results{5} = zeros(length(numVertices), size(distanceRange, 1)); % Time Fast
results{6} = zeros(length(numVertices), size(distanceRange, 1), 2); % Nodes Fast method
results{7} = numVertices;
results{8} = distanceRange;

% Loop over test parameters
for i = 1:length(numVertices)
    for j = 1:size(distanceRange,1)
        disp([i,j])
        % Initialize result variables for current test case
        pathTime2 = zeros(1, numMaps*numTests); % Time of final path
        pathTimeFast2 = zeros(1, numMaps*numTests);
        timeOur2 = zeros(1, numMaps);
        timeFast2 = zeros(1, numMaps);
        nodesOur2 = zeros(2, numMaps);
        nodesFast2 = zeros(2, numMaps);

        % Loop over maps
        k = 1;
        while k <= numMaps
            nNodes = numVertices(i); %randi([30,50]); % Nodes in the graph are <1,2,...,nNodes>
            xMax = 1000;
            yMax = 1000;
            xMin = 0;
            yMin = 0;
            xCentre = 1220333.064;
            yCentre = 769592.061;
            xMax = xCentre + 5000; % Now, everything is in metres
            yMax = yCentre + 5000;
            xMin = xCentre - 5000; % Now, everything is in metres
            yMin = yCentre - 5000;
            [E, D, posX, posY] = getMap(nNodes, xMax, yMax, xMin, yMin, 'del');
%             E = triu(E);
            f = 1;
%             autoSpeed = zeros(nNodes);
%             teleSpeed = zeros(nNodes);
%             maxWaits  = zeros(nNodes, 1);
%             for n = 1:nNodes
%                 if n <= nNodes/3 % Big roads, similar speeds and medium waiting
%                     autoSpeed(n,:) = randi([10,20], 1, nNodes);
%                     teleSpeed(n,:) = autoSpeed(n,:) + randi([10,20], 1, nNodes);
%                     maxWaits(n) = f*randi([1, 2], [1, 1]);
%                 elseif n > nNodes/3 && n <= 2*nNodes/3 % Inner roads, medium speeds, large waiting
%                     autoSpeed(n,:) = randi([5,10], 1, nNodes);
%                     teleSpeed(n,:) = autoSpeed(n,:) + randi([0,0], 1, nNodes);
%                     maxWaits(n) = f*randi([15, 20], [1, 1]);
%                 else % Crowded areas, low speeds, low waiting
%                     autoSpeed(n,:) = randi([1,5], 1, nNodes);
%                     teleSpeed(n,:) = autoSpeed(n,:) + randi([10,20], 1, nNodes);
%                     maxWaits(n) = f*randi([0, 1], [1, 1]);
%                 end
%             end
            
%             autoSpeed = randi([1,5], nNodes, nNodes);
%             teleSpeed = autoSpeed + randi([1,20], nNodes, nNodes);
%             autoSpeed = 1.0 + 5*rand(nNodes, nNodes);
%             teleSpeed = autoSpeed + 0.0 + 10*rand(nNodes, nNodes);
            autoSpeed = 40*rand(nNodes, nNodes) + 1; % In metres per minute
            teleSpeed = autoSpeed + 40*rand(nNodes, nNodes) + 1;


            A = f*ceil(D ./ autoSpeed);
%             th1 = maxk(max(A), floor(sum(sum(E))*0.2)); % Means we are going to change 10% of the slowest edges
%             th = th1(end);
%             A = A.*(A<th) + 99999*E.*(A>=th);
            B = f*ceil(D ./ teleSpeed);
            [eX,eY] = find(E);
            edges = [eX eY];
            edgesCai = [eX eY; eX eY+nNodes];

            % Waiting limits
            maxWaits = f*randi([0, 15], [nNodes, 1]);

            % Operator availability
            meanB = sum(B, 'all')/sum(B>0, 'all');
            oprAvail = f*getOprAvail(floor(0.4*meanB), floor(1.5*meanB), floor(0.2*meanB), floor(1.5*meanB), 300);
            oprAvail = f*getOprAvail(30, 200, 10, 100, 2000);
            
            % Convert Operator availability to Edge availability
            tic
            edgeAvail = getEdgeBudgets(E, B, oprAvail(end), oprAvail); % We only go until oprAvail(end), because after that operator is unavailable
            edgeAvailTime = toc;

            % Initialize more result storing variables
            pathTimeOur = zeros(1, numTests); % Time of final path
            pathTimeFast = zeros(1, numTests); % Time of final path under fast method
            timeOur = zeros(1, numTests);
            timeFast = zeros(1, numTests);    

            nodesOur = zeros(2, numTests);
            nodesFast = zeros(2, numTests);

%             newHeuristic = Inf(nNodes,1);
%             for ver = 1:nNodes
%                 distToGoal = staticDijkstras([edges(:,2) edges(:,1)], ver, startVertex, B', 'all');
%                 [pathTemp, ~, ~]  = fastestTaskDijkstras(startVertex, ver, A, B, edgeAvail, maxWaits, distToGoal);
%                 newHeuristic(ver) = pathTemp(end,2);
%             end

                
            for itr = 1:numTests
                % Start and Goal vertices
                goodStart = 0;
                while goodStart == 0
                    startVertex = randi([1, floor(nNodes)]);
                    if A(startVertex) < 99990
                        goodStart = 1; % This ensures that we do not start at a vertex where teleoperation is definitely required.
                    end
                end
                goalVertex = 0;
                for g = randperm(nNodes)
                    dist = norm([posX(startVertex) posY(startVertex)] - [posX(g) posY(g)]);
                    if g ~= startVertex && dist >= distanceRange(j,1) && dist <= distanceRange(j,2)
                        goalVertex = g;
                        break;
                    end
                end
                startVertex = 9; %randi([1, floor(nNodes*0.1)]);
                goalVertex = 41; %randi([floor(nNodes*0.9), nNodes]);
                getNewMap = 0;
                if goalVertex > nNodes || goalVertex == 0
%                     drawmap(E, xMax, yMax, posX, posY, startVertex, goalVertex, distanceRange(j,1), distanceRange(j,2))
                    getNewMap = 1;
                    disp('Wrong Goal.')
                    break;
                end

                pathAuto = staticDijkstras(edges, startVertex, goalVertex, A);
                if isempty(pathAuto)
                    disp("No path exists!")
                    break;
                end
                maxTime = pathAuto(end,2);

                pathTele = staticDijkstras(edges, startVertex, goalVertex, B);
                if isempty(pathTele)
                    haha = 1;
                end
                % pathAuto = staticDijkstras(edges, startVertex, goalVertex, A);
                distToGoal = staticDijkstras([edges(:,2) edges(:,1)], goalVertex, startVertex, B', 'all');
                
                tic
                [pathFast, Q4, Qexp4]  = fastestTaskDijkstras(startVertex, goalVertex, A, B, edgeAvail, maxWaits, distToGoal, 0);
                timeFast(itr) = toc;% + edgeAvailTime;
                pathTimeFast(itr) = pathFast(end,2);
                nodesFast(:,itr) = [size(Q4,1); size(Qexp4,2)]; 
                
                tic
                [pathOur, Q3, Qexp3] = getRobotPath(E, startVertex, goalVertex, A, B, oprAvail, edgeAvail, maxWaits, pathAuto(end,2), distToGoal, 'our');
                timeOur(itr) = toc;% + edgeAvailTime;
                nodesOur(:,itr) = [size(Q3,1); size(Qexp3,1)];
                pathTimeOur(itr) = pathOur(end,2);

                % Visualize paths
                notUsed = 0;
                for ver = 1:size(pathOur,1)-1
                    if pathOur(ver,1) <= size(edgeAvail,3) && edgeAvail(pathOur(ver,1), pathOur(ver+1,1), pathOur(ver,2)+1) >= 0 && pathOur(ver,4) == 0
                        notUsed = 1;
                        break;
                    end
                end
                if any(pathOur(:,3) > 5) % > 10 && size(pathFast,1) > 10 % pathOur(end,2) > pathFast(end,2)
                    aa = 1;
                end
                if pathOur(end,2)/pathFast(end,2) < 0.9 && isequal(pathOur(:,1), pathFast(:,1)) && notUsed == 1 %&& length(pathOur(:,1)) > length(pathFast(:,1)) + 1 %
                    [pathOur(end,2), pathFast(end,2)]
                    figure(1);
                    subplot(1,2,1)
                    drawmap(E, xMax, yMax, xMin, yMin, posX, posY, startVertex, goalVertex, 0, 0, pathOur, "Proposed")
                    
                    subplot(1,2,2)
                    drawmap(E, xMax, yMax, xMin, yMin, posX, posY, startVertex, goalVertex, 0, 0, pathFast, "Greedy")
                    ax = gca;
                    ax.FontSize = 18;
%                     figure(2);
%                     subplot(2,2,[3 4])
                    figure(2);
                    visualizePath(oprAvail, pathFast(end,2), pathOur, pathFast)
                    plotted = 1;
                end
            end
            if getNewMap == 1
                continue;
            end
            pathTime2((k-1)*numTests+1:k*numTests) = pathTimeOur;
            pathTimeFast2((k-1)*numTests+1:k*numTests) = pathTimeFast;
            timeOur2(k) = mean(timeOur);
            timeFast2(k) = mean(timeFast);
            
            nodesOur2(:, k) = mean(nodesOur, 2);
            nodesFast2(:, k) = mean(nodesFast, 2);

            k = k + 1;
        end
        % Store results for the particular test condition
        results{1}(i,j,:) = pathTime2;
        results{2}(i,j)   = mean(timeOur2);
        results{3}(i,j,:) = mean(nodesOur2, 2);
        
        results{4}(i,j,:) = pathTimeFast2;
        results{5}(i,j)   = mean(timeFast2);
        results{6}(i,j,:) = mean(nodesFast2, 2);
               
    end
end
save("results_paths.mat", "results")

%% Plotting
close;
% Computation Time (nVertices = x)
figure(1);
x = 1:size(results{7},2); % results{7} is nVertices
yLim = 1.1*max([results{2} results{5}], [], 'all');
for i = 1:size(results{8},1) % results{8} is distance ranges
    subplot(1,size(results{8},1),i)

    timeOurs = results{2}(:,i);
    timeFast = results{5}(:,i);

    % Plot the bar graph
    bar(x, [timeOurs timeFast]);
    ylim([0 yLim])
%     title("Distance range: " + results{8}(i,1) + "-" + results{8}(i,2))
    % Add labels and legend
    xlabel('Number of vertices');
    set(gca,'xticklabel',results{7})%{'0-300', '300-600', '600-1000', '0-1000'})
    if i == 1
        legend('Proposed', 'Fast');
        ylabel('Computation Time');
    end
end


%% Nodes Explored
figure(2);
nodesOurs = mean(results{3}(:,:,:),1);
nodesFast = mean(results{6}(:,:,:),1);
hi = sum([nodesOurs; nodesFast], 3);
yLim = 1.1*max(hi, [],"all");
for i = 1:size(results{8},1) % Distance ranges
    subplot(1,size(results{8},1),i)

    nodesOurs = reshape(mean(results{3}(:,i,:),1), [1 2]);
    nodesFast = reshape(mean(results{6}(:,i,:),1), [1 2]);

    hi = [nodesOurs; nodesFast];
    bar(hi,'stacked')

    % Add labels and legend
%     title("Distance range: " + results{8}(i,1) + "-" + results{8}(i,2))
    xlabel('Method');
    set(gca,'xticklabel',{'Ours', 'Fast'})
    ylim([0 yLim])
    if i == 1 %size(results{9},2)
        ylabel('Total Nodes Generated');
        legend('Nodes not explored', 'Nodes explored')
    end
end

%% Compare solution (path) length
figure(3);
x = 1:size(results{7},2); % x values
% yLim = 1.1*max(abs(results{12}-results{1})./results{1},[],'all');
for i = 1:size(results{8},1)
%     for j = 1:size(distanceRange,1)
%         subplot(size(distanceRange,1),size(results{7},2),i+(j-1)*size(results{7},2))
        subplot(2,size(results{8},1),i)

        pathOurs = results{1}(:,i,:);
        pathOurs = reshape(pathOurs, [1, numel(pathOurs)]);
        pathFast = results{4}(:,i,:);
        pathFast = reshape(pathFast, [1, numel(pathFast)]);
    
        % Plot the graph
        diff = (pathFast-pathOurs)./pathOurs;
        diff = sort(diff);
        a = diff(floor(numel(diff)*0.8):end);
        histogram(diff)
        [mean(diff), mean(diff(diff>0)), mean(a)]
    %     bar(x, [pathFast; pathOurs]');
    %     ylim([0 yLim])
%         title("Distance range: " + results{8}(i,1) + "-" + results{8}(i,2))
    % Add labels and legend
        xlabel('Relative path time');
    %     set(gca,'xticklabel',{'0-300', '300-600', '600-1000'})
        if i == 1
    %         legend('Fast', 'Ours');
            ylabel('Number of instances');
        end
%     end
end
yLim = 0;
for i = 1:size(results{8},1)
        subplot(2,size(results{8},1),i+size(results{8},1))

        pathOurs = results{1}(:,i,:);
        pathOurs = reshape(pathOurs, [1, numel(pathOurs)]);
        pathFast = results{4}(:,i,:);
        pathFast = reshape(pathFast, [1, numel(pathFast)]);
        
        meanOurs = mean(pathOurs);
        meanFast = mean(pathFast);
        
        sortedOurs = sort(pathOurs, 'descend');
        sortedFast = sort(pathFast, 'descend');
        meanOurs20 = mean(sortedOurs(1:floor(numel(sortedOurs)*0.2)));
        meanFast20 = mean(sortedFast(1:floor(numel(sortedFast)*0.2)));
        
        diff = (pathFast-pathOurs);%./pathOurs;
        sortedDiff = sortrows([diff' pathOurs' pathFast'], 1, 'descend');
        meanOursDiff20 = mean(sortedDiff(1:floor(numel(sortedOurs)*0.2), 2));
        meanFastDiff20 = mean(sortedDiff(1:floor(numel(sortedFast)*0.2), 3));
        
%         diff = (pathFast-pathOurs)./pathOurs;
%         sortedDiff = sortrows([diff' pathOurs' pathFast'], 1, 'descend');
%         meanOursDiff30 = mean(sortedDiff(1:floor(numel(sortedOurs)*0.2), 2));
%         meanFastDiff30 = mean(sortedDiff(1:floor(numel(sortedFast)*0.2), 3));
        yLim = max(yLim, (meanFastDiff20-meanOursDiff20)/meanOursDiff20);
        % Plot the graph
%         x = 1:2;
        bar([(meanFast-meanOurs)/meanOurs; (meanFastDiff20-meanOursDiff20)/meanOursDiff20]);
        set(gca,'xticklabel',{'Mean', 'Worst 20%'})
%         xtickangle(20)
%         ax = gca;
%         ax.set_xticklabels(labels, rotation=45, ha='right', rotation_mode='anchor')
        title("Distance range: " + results{8}(i,1) + "-" + results{8}(i,2))
        if i == 1
            ylabel('Relative path length');
        end
%     end
end
if yLim > 0
    for i = 1:size(results{8},1)
        subplot(2,size(results{8},1),i+size(results{8},1))
        ylim([0 1.1*yLim])
    end
end
done = 1;


% %% Plotting
% close;
% % Computation Time (nVertices = x)
% figure(1);
% x = 1:size(results{8},1); % results{8} is distance ranges
% yLim = 1.1*max([results{2} results{5}], [], 'all');
% for i = 1:size(results{7},2) % results{7} is nVertices
%     subplot(1,size(results{7},2),i)
% 
%     timeOurs = results{2}(i,:);
%     timeFast = results{5}(i,:);
% 
%     % Plot the bar graph
%     bar(x, [timeOurs; timeFast]');
%     ylim([0 yLim])
%     title("No. of Vertices = " + results{7}(i))
%     % Add labels and legend
%     xlabel('Distance Range');
%     set(gca,'xticklabel',{'0-300', '300-600', '600-1000', '0-1000'})
%     if i == size(results{7},2)
%         ignore = 1;
%     elseif i == 1
%         legend('Ours', 'Fast');
%         ylabel('Computation Time');
%     end
% end
% 
% 
% % Nodes Explored
% figure(2);
% nodesOurs = mean(results{3}(:,:,:),2);
% nodesFast = mean(results{6}(:,:,:),2);
% hi = sum([nodesOurs; nodesFast], 3);
% yLim = 1.1*max(hi, [],"all");
% for i = 1:size(results{7},2) % NVertices
%     subplot(1,size(results{7},2),i)
% 
%     nodesOurs = reshape(mean(results{3}(i,:,:),2), [1 2]);
%     nodesFast = reshape(mean(results{6}(i,:,:),2), [1 2]);
% 
%     hi = [nodesOurs; nodesFast];
%     bar(hi,'stacked')
% 
%     % Add labels and legend
%     title("No. of Vertices = " + results{7}(i))
%     xlabel('Method');
%     set(gca,'xticklabel',{'Ours', 'Fast'})
%     ylim([0 yLim])
%     if i == 1 %size(results{9},2)
%         ylabel('Nodes');
%         legend('Not explored', 'Explored')
%     end
% end
% 
%% Compare solution (path) length
figure(4);
x = 1:size(results{7},1); % x values
% yLim = 1.1*max(abs(results{12}-results{1})./results{1},[],'all');
for i = 1:size(results{7},2)
%     for j = 1:size(distanceRange,1)
%         subplot(size(distanceRange,1),size(results{7},2),i+(j-1)*size(results{7},2))
        subplot(2,size(results{7},2),i)

        pathOurs = results{1}(i,:,:);
        pathOurs = reshape(pathOurs, [1, numel(pathOurs)]);
        pathFast = results{4}(i,:,:);
        pathFast = reshape(pathFast, [1, numel(pathFast)]);
    
        % Plot the graph
        diff = (pathFast-pathOurs)./pathOurs;
        diff = sort(diff);
        a = diff(floor(numel(diff)*0.8):end);
        histogram(diff)
        [mean(diff), mean(diff(diff>0)), mean(a)]
    %     bar(x, [pathFast; pathOurs]');
    %     ylim([0 yLim])
        title("No. of Vertices = " + results{7}(i))
        % Add labels and legend
    %     xlabel('Distance Range');
    %     set(gca,'xticklabel',{'0-300', '300-600', '600-1000'})
        if i == size(results{7},2)
            ignore = 1;
        elseif i == 1
    %         legend('Fast', 'Ours');
            ylabel('Path Length');
        end
%     end
end
yLim = 0;
for i = 1:size(results{7},2)
        subplot(2,size(results{7},2),i+size(results{7},2))

        pathOurs = results{1}(i,:,:);
        pathOurs = reshape(pathOurs, [1, numel(pathOurs)]);
        pathFast = results{4}(i,:,:);
        pathFast = reshape(pathFast, [1, numel(pathFast)]);
        
        meanOurs = mean(pathOurs);
        meanFast = mean(pathFast);
        
        sortedOurs = sort(pathOurs, 'descend');
        sortedFast = sort(pathFast, 'descend');
        meanOurs20 = mean(sortedOurs(1:floor(numel(sortedOurs)*0.2)));
        meanFast20 = mean(sortedFast(1:floor(numel(sortedFast)*0.2)));
        
        diff = (pathFast-pathOurs);%./pathOurs;
        sortedDiff = sortrows([diff' pathOurs' pathFast'], 1, 'descend');
        meanOursDiff20 = mean(sortedDiff(1:floor(numel(sortedOurs)*0.2), 2));
        meanFastDiff20 = mean(sortedDiff(1:floor(numel(sortedFast)*0.2), 3));
        
        diff = (pathFast-pathOurs)./pathOurs;
        sortedDiff = sortrows([diff' pathOurs' pathFast'], 1, 'descend');
        meanOursDiff30 = mean(sortedDiff(1:floor(numel(sortedOurs)*0.2), 2));
        meanFastDiff30 = mean(sortedDiff(1:floor(numel(sortedFast)*0.2), 3));

        % Plot the graph
%         x = 1:2;
        bar([(meanFast-meanOurs)/meanOurs; (meanFastDiff20-meanOursDiff20)/meanOursDiff20]);
        set(gca,'xticklabel',{'Mean', 'Worst 20%'})
        yLim = max(yLim, (meanFastDiff20-meanOursDiff20)/meanOursDiff20);
        title("No. of Vertices = " + results{7}(i))
        if i == size(results{7},2)
            ignore = 1;
        elseif i == 1
            ylabel('Relative path length');
        end
%     end
end
for i = 1:size(results{7},2)
    subplot(2,size(results{7},2),i+size(results{7},2))
    ylim([0 1.1*yLim])
end
done = 1;
% 
% 
% 
%% Grid Compare solution (path) length
figure(5);
x = 1:size(results{7},1); % x values
% yLim = 1.1*max(abs(results{12}-results{1})./results{1},[],'all');
% for i = 1:size(results{7},2)
%     for j = 1:size(results{8},1)
%         subplot(size(results{8},1),size(results{7},2),i+(j-1)*size(results{7},2))
% %         subplot(size(results{8},1),size(results{7},2),i)
% 
%         pathOurs = results{1}(j,i,:);
%         pathOurs = reshape(pathOurs, [1, numel(pathOurs)]);
%         pathFast = results{4}(j,i,:);
%         pathFast = reshape(pathFast, [1, numel(pathFast)]);
%     
%         % Plot the graph
%         diff = (pathFast-pathOurs)./pathOurs;
%         diff = sort(diff);
%         a = diff(floor(numel(diff)*0.8):end);
%         histogram(diff)
%         [mean(diff), mean(diff(diff>0)), mean(a)]
%     %     bar(x, [pathFast; pathOurs]');
%     %     ylim([0 yLim])
%         title("Vertices = " + results{7}(i) + ", " + "Distance range: " + results{8}(i,1) + "-" + results{8}(i,2))
%         % Add labels and legend
%     %     xlabel('Distance Range');
%     %     set(gca,'xticklabel',{'0-300', '300-600', '600-1000'})
%         if i == 1
%             ylabel('Relative path length');
%         end
%         if j == size(results{8},1)
%             set(gca,'xticklabel',{'Mean', 'Worst 20%'})
%         end
% 
%     end
% end
yLim = 0;
for i = 1:size(results{7},2) % nVertices
    for j = 1:size(results{8},1) % Distance range
        subplot(size(results{8},1),size(results{7},2),i+(j-1)*size(results{7},2))

        pathOurs = results{1}(i,j,:);
        pathOurs = reshape(pathOurs, [1, numel(pathOurs)]);
        pathFast = results{4}(i,j,:);
        pathFast = reshape(pathFast, [1, numel(pathFast)]);
        
        meanOurs = mean(pathOurs);
        meanFast = mean(pathFast);
        
        sortedOurs = sort(pathOurs, 'descend');
        sortedFast = sort(pathFast, 'descend');
        meanOurs20 = mean(sortedOurs(1:floor(numel(sortedOurs)*0.2)));
        meanFast20 = mean(sortedFast(1:floor(numel(sortedFast)*0.2)));
        
        diff = (pathFast-pathOurs);%./pathOurs;
        sortedDiff = sortrows([diff' pathOurs' pathFast'], 1, 'descend');
        meanOursDiff20 = mean(sortedDiff(1:floor(numel(sortedOurs)*0.2), 2));
        meanFastDiff20 = mean(sortedDiff(1:floor(numel(sortedFast)*0.2), 3));
        
        diff = (pathFast-pathOurs)./pathOurs;
        sortedDiff = sortrows([diff' pathOurs' pathFast'], 1, 'descend');
        meanOursDiff30 = mean(sortedDiff(1:floor(numel(sortedOurs)*0.3), 2));
        meanFastDiff30 = mean(sortedDiff(1:floor(numel(sortedFast)*0.3), 3));

        % Plot the graph
%         x = 1:2;
        bar([(meanFast-meanOurs)/meanOurs; (meanFastDiff20-meanOursDiff20)/meanOursDiff20]);
        set(gca,'xticklabel',{'Mean', 'Worst 20%'})
        yLim = max(yLim, (meanFastDiff20-meanOursDiff20)/meanOursDiff20);
        title("Vertices = " + results{7}(i) + ", " + "Distance: " + results{8}(j,1) + "-" + results{8}(j,2))
        if i == 1
            ylabel('Relative path length');
        end
        if j == size(results{8},1)
            set(gca,'xticklabel',{'Mean', 'Worst 20%'})
        end
    end
end
if yLim > 0
    for i = 1:size(results{7},2)
        for j = 1:size(results{8},1)
            subplot(size(results{8},1),size(results{7},2),i+(j-1)*size(results{7},2))
            ylim([0 1.1*yLim])
        end
    end
end
done = 1;
% 
% 
%
%% All Compare solution (path) length
figure(6);
% x = 1:size(results{7},1); % x values
% % yLim = 1.1*max(abs(results{12}-results{1})./results{1},[],'all');
% subplot(1,2,1)
% 
% pathOurs = results{1}(:,:,:);
% pathOurs = reshape(pathOurs, [1, numel(pathOurs)]);
% pathFast = results{4}(:,:,:);
% pathFast = reshape(pathFast, [1, numel(pathFast)]);
% 
% % Plot the graph
% diff = (pathFast-pathOurs)./pathOurs;
% diff = sort(diff);
% a = diff(floor(numel(diff)*0.8):end);
% histogram(diff)
% [mean(diff), mean(diff(diff>0)), mean(a)]
% %     bar(x, [pathFast; pathOurs]');
% %     ylim([0 yLim])
% title("Distribution of relative path lengths")
% if i == 1
%     ylabel('Path Length');
% end
% %
% yLim = 0;
% subplot(1,2,2)

pathOurs = results{1}(:,:,:);
pathOurs = reshape(pathOurs, [1, numel(pathOurs)]);
pathFast = results{4}(:,:,:);
pathFast = reshape(pathFast, [1, numel(pathFast)]);

meanOurs = mean(pathOurs);
meanFast = mean(pathFast);

sortedOurs = sort(pathOurs, 'descend');
sortedFast = sort(pathFast, 'descend');
meanOurs20 = mean(sortedOurs(1:floor(numel(sortedOurs)*0.2)));
meanFast20 = mean(sortedFast(1:floor(numel(sortedFast)*0.2)));

diff = (pathFast-pathOurs);%./pathOurs;
sortedDiff = sortrows([diff' pathOurs' pathFast'], 1, 'descend');
meanOursDiff20 = mean(sortedDiff(1:floor(numel(sortedOurs)*0.2), 2));
meanFastDiff20 = mean(sortedDiff(1:floor(numel(sortedFast)*0.2), 3));

diff = (pathFast-pathOurs)./pathOurs;
sortedDiff = sortrows([diff' pathOurs' pathFast'], 1, 'descend');
meanOursDiff30 = mean(sortedDiff(1:floor(numel(sortedOurs)*0.3), 2));
meanFastDiff30 = mean(sortedDiff(1:floor(numel(sortedFast)*0.3), 3));

% Plot the graph
bar([(meanFast-meanOurs)/meanOurs; (meanFastDiff20-meanOursDiff20)/meanOursDiff20]);
set(gca,'xticklabel',{'Mean', 'Worst 20%'})
yLim = max(yLim, (meanFastDiff20-meanOursDiff20)/meanOursDiff20);
title("Average and Worst 20% comparison")
if i == 1
    ylabel('Relative path length');
end
% 
% 
% 
%% Violin Compare solution (path) length
figure(7);

pathOurs = results{1}(:,:,:);
pathOurs = reshape(pathOurs, [1, numel(pathOurs)]);
pathFast = results{4}(:,:,:);
pathFast = reshape(pathFast, [1, numel(pathFast)]);

% Plot the graph
diff = (pathFast-pathOurs)./pathOurs;
violin(diff')
boxplot(diff')
% set(gca,'xticklabel',{'Mean', 'Worst 20%'})
% yLim = max(yLim, (meanFastDiff20-meanOursDiff20)/meanOursDiff20);
title("Performance comparison")
if i == 1
    ylabel('Relative path length');
end
done = 1;
% 
% 
% 
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

function newE = getEdgeBudgets(E, B, T, oprAvail)
    newE = -1*ones(size(B,1),size(B,2), T+1);
    for i = 1:ceil(size(oprAvail,2)/2)
        tCurr = oprAvail(2*i-1); % This only works if oprAvaila has even number of elements, i.e., in the end we get unavailable operator
        if tCurr > T
            break;
        end
        changeTime = oprAvail(2*i);
        for t = tCurr:changeTime-1
            % For all elements less than changeTime-t-1 add an edge
            value = changeTime-1-t; % -1 because the availabilty changes at changeTime, so we can only assist until 1 unit before
            currE = (B <= value) .* (value-B) .* E;
            newE(:,:,t+1) = currE - (B>value); % t+1 because we start with t=0
        end
    end 
end