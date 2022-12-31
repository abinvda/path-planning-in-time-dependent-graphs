function path = getRobotPath(edges, startVertex, endVertex, A, B, oprAvail, maxWaits, maxTime)
% 'method' argument specifies which algorithm to use for computing path
% TODO: Greedy dijkstras is the one where we try to teleoperate whenever possible. 
% If opr is not available, we check if it is faster to wait and then teleoperate.

Qexp = [];
QSize = 10000;
QEmpty = repmat([0 10000 0 0 0 0 0 0 0], QSize,1);
Q = [startVertex, 0, maxWaits(startVertex), 0, 0, 0, 0, 0, 1]; % Elements are [vertex, arrival time, budget, previous edge, its arrival time, and wait done there, mode of operation from there to current vertex, index of predecessor node, index of current node]
Q = [Q; QEmpty];
% For now, implementing the priority Q as a matrix that we'll sort or find minimim from.
% We can remove the previous vertex and its arrival time if we are using the node indices. Keeping them for testing and verification now.
maxIndex = 1;
while ~isempty(Q)
    % Pop the best element from the Q
    [~, bestIndex] = min(Q(:,2));
    bestNode = Q(bestIndex,:);
    if bestNode(1,1) == endVertex
        break;
    end
    if bestNode(1,2) >= maxTime
        disp("Time's up!!")
        break;
    end
    neighbours = edges(edges(:,1) == bestNode(1,1), 2); % In the edges matrix, find all where first element is the selected node
%   neighbours = cell2mat(edges(bestNode(1,1),1)); %(edges(:,1) == bestNode(1,1), 2); % In the edges matrix, find all where first element is the selected node
%     for i = 1:size(neighbours,1)
    for e = neighbours'
        arrivalTime = bestNode(1,2);
        budget = bestNode(1,3);
        autoTime = A(bestNode(1,1), e);
        teleopTime = B(bestNode(1,1), e);
        % for each neighbour, determine the times for which to expand, using function edgeAvailability
        [departureTimes, finishTimes, modes, newBudgets] = edgeExplorationTimes(arrivalTime, budget, autoTime, teleopTime, oprAvail); % finishTimes corresponding to those departureTimes and mode of operation
        % add these neighbours to priorityQ with respective travel times
        for j = 1:size(departureTimes,2)
            wait = departureTimes(j) - bestNode(1,2);
            newBudget = newBudgets(j) + maxWaits(e);
%             a = [e, finishTimes(j), newBudget, bestNode(1,1), bestNode(1,2), wait, modes(j)];
%             b = [e, finishTimes(j), newBudget];
%             [tf, ~] = ismember(b, Q(:,1:3),'rows');
%             if ~tf
                maxIndex = maxIndex+1;
                if maxIndex > size(Q,1)
                    Q = [Q; QEmpty];
                end
                Q(maxIndex, :) = [e, finishTimes(j), newBudget, bestNode(1,1), bestNode(1,2), wait, modes(j), bestNode(1,9), maxIndex];
%             end
        end
    end
%     Q(1,:) = []; % Remove the currently explored node
%     Q(bestIndex,:) = []; % Remove the currently explored node
    Q(bestIndex,2) = Inf; % Increasing the arrival time of the currently explored node to a high number
%     tic
%     Q = sortrows(Q,2);
%     toc
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Later on we can also think of getting an approximation algorithms, by solving static graph with both autonomous and teleoperated costs.
    % This will give is min and max cost from a vertex to the goal.
    % So, we can use these to eliminate vertices from the search for which things are not very good
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Add this explored node to a separate list, which we'll use later to find the path
    Qexp(end+1,:) = bestNode;
    % Do some cleaning, remove redundant nodes from the Q
end
% Do whatever to get the path
Qexp(end+1,:) = bestNode;
if bestNode(1,1) == endVertex
    path = getPath(Qexp, maxWaits);
else
    disp("Path not found.")
    path = [];
end
end


%% TODO: Function to compute opr availability for a given edge at a given time
function [departureTimes, finishTimes, modes, newBudgets] = edgeExplorationTimes(arrivalTime, budget, autoTime, teleopTime, oprAvail)
% Given an edge, current time, budget and operator availability, compute which are all the departure times that we should be exploring
% For autonomous travel, we don't need to wait at all, and all the waiting time can be stored in the budget. This way even if the final path requires autonomous travel with waiting, we can use the budget.    

    departureTimes = arrivalTime; % Means no waiting
    finishTimes = arrivalTime + autoTime; % One node to always expand is autonomous mode with no waiting
    modes = 0;
    newBudgets = budget;
    
    % Check is operator is available right away
%     [avail, bud] = edgeAvailability(teleopTime, arrivalTime, oprAvail, budget);
%     if avail
%         departureTimes(end+1) = arrivalTime;
%         finishTimes(end+1) = arrivalTime + teleopTime;
%         modes(end+1) = 1;
%         newBudgets(end+1) = bud; 
%     end
    rank = find(oprAvail<=arrivalTime,1, 'last'); %rank of arrivalTime in oprAvail
    if isempty(rank)
        rank = 0;
    end
    %     for i = 1:size(oprAvail,2) % For large oprAvail vectors, we can also do something like a binary search or other methods of finding rank in an ordered list
%         if oprAvail(i) <= arrivalTime
%             rank = rank + 1;
%         else
%             break;
%         end
%     end
    maxDepartureTime = arrivalTime + budget; % This is the max time we can depart at
    maxOprAvailIdx = find(oprAvail<=maxDepartureTime,1, "last");
    if mod(rank,2) == 1 % Means available right now
        times = [arrivalTime, oprAvail(rank+2:2:maxOprAvailIdx)];
    else % Means operator not available currently
        times = [oprAvail(rank+1:2:maxOprAvailIdx)];
    end
    for currentTime = times
        [avail, bud] = edgeAvailability(teleopTime, currentTime, oprAvail, maxDepartureTime-currentTime);
        if avail
            departureTimes(end+1) = currentTime;
            finishTimes(end+1) = currentTime + teleopTime;
            modes(end+1) = 1;
            newBudgets(end+1) = bud; 
        end
    end
%     oprAvailIdx = min(size(oprAvail, 2), rank + 1);
%     currentTime = oprAvail(oprAvailIdx);
%     while currentTime <= maxTime
%         if mod(oprAvailIdx, 2) == 1
%             [avail, bud] = edgeAvailability(teleopTime, currentTime, oprAvail, maxTime-currentTime);
%             if avail
%                 departureTimes(end+1) = currentTime;
%                 finishTimes(end+1) = currentTime + teleopTime;
%                 modes(end+1) = 1;
%                 newBudgets(end+1) = bud; 
%             end
%         end
%         oprAvailIdx = oprAvailIdx + 1;
%         if oprAvailIdx > size(oprAvail, 2)
%             break;
%         end
%         currentTime = oprAvail(oprAvailIdx);
%     end
end

function [result, budget] = edgeAvailability(teleopTime, startTime, oprAvail, oldBudget)
    result = 0;
    budget = 0;
%     rank = 0; %rank of startTime in oprAvail
%     for i = 1:size(oprAvail,2) % For large oprAvail vectors, we can also do something like a binary search or other methods of finding rank in an ordered list
%         if oprAvail(i) <= startTime
%             rank = rank + 1;
%         else
%             break;
%         end
%     end
    rank = find(oprAvail<=startTime,1, 'last'); %rank of arrivalTime in oprAvail
    if isempty(rank)
        rank = 0;
    end
    %rank2 = rank of startTime + teleopTime in oprAvail % We can also check if oprAvail(rank1) > startTime + teleopTime
    if mod(rank, 2) == 1 && (rank == size(oprAvail,2) || oprAvail(rank+1) > startTime + teleopTime) % Is operator is available throughout the duration of the edge
        result = 1;
        if rank == size(oprAvail,2)
            budget = oldBudget;
        else
            budget = min(oldBudget, oprAvail(rank+1) - (startTime + teleopTime)); % TODO: This will change if travel times are not constant.
        end
        % NOTE: The end time won't exceed the last element of oprAvail, which is selected to be a large number
    end
end

function path = getPath(Qexp, maxWaits)
% Path is stored as [vertex, arrival time, wait, mode]
    currentNode = Qexp(end,:);
    remainingWait = currentNode(6);
    path = [currentNode(1:2), 0, 0]; %Qexp(end,[1,2,6,7]); % Can't wait at the goal, and mode is 0
    % We need to match vertex and arrival time and pick the one with largest budget
    while currentNode(9) ~= 1 %~(currentNode(1) == 1 && currentNode(2) == 0) 
        predecessorNode = Qexp(Qexp(:,9) == currentNode(:,8), :);
%         path = [predecessorNode(end,[1,2,6,7]); path];
%         currentNode = predecessorNode;

        currentWait = min(remainingWait, maxWaits(predecessorNode(1)));
        realArrivalTime = predecessorNode(2)+remainingWait-currentWait;
        path = [predecessorNode(1), realArrivalTime, currentWait, currentNode(7); path];
        remainingWait = remainingWait-currentWait+predecessorNode(6);
        currentNode = predecessorNode;
    end
end

