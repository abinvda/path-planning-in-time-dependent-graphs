function [path, Q, Qexp] = getRobotPathAllTime(edges, startVertex, endVertex, A, B, oprAvail, maxWaits, maxTime, heuristic)

Qexp = [];
Q = cell(1, 1); % Initialize heap with starting vertex
Q{1} = [startVertex, 0, maxWaits(startVertex), 0, 0, 0, 0, 0, 1, false, heuristic(startVertex), heuristic(startVertex)]; % Elements are [vertex, arrival time, budget, previous edge, its arrival time, and wait done there, mode of operation from there to current vertex, index of predecessor node, index of current node, visited flag]

% We can remove the previous vertex and its arrival time if we are using the node indices. Keeping them for testing and verification now.
maxIndex = 1;
while ~isempty(Q)
    % Pop the best element from the Q
    [bestNode, Q] = heapPop(Q, heuristic);
    if bestNode(1) == endVertex
        break;
    end
    if bestNode(2) + heuristic(bestNode(1)) > maxTime
        disp("Time's up!!") 
        break;
    end
    if bestNode(10)
        continue; % Skip this node if it has already been visited
    end
    neighbours = edges(edges(:,1) == bestNode(1), 2); % In the edges matrix, find all where first element is the selected node
    
    arrivalTime = bestNode(2);
    budget = bestNode(3);
    autoTimes = A(bestNode(1), neighbours);
    teleopTimes = B(bestNode(1), neighbours);
        

    for i = 1:size(neighbours,1)
        e = neighbours(i);
        % for each neighbour, determine the times for which to expand, using function edgeAvailability
        [departureTimes, finishTimes, modes, newBudgets] = allExplorationTimes(arrivalTime, maxWaits(bestNode(1)), autoTimes(i), teleopTimes(i), oprAvail); % finishTimes corresponding to those departureTimes and mode of operation
        % add these neighbours to priorityQ with respective travel times
        for j = 1:size(departureTimes,1)
            wait = departureTimes(j) - bestNode(2);
            newBudget = newBudgets(j) + maxWaits(e);
            % We use the function below to check if the same node is present or if new one has higher budget
            [better, location, indices] = isBetterNodePresent(Q, [e, finishTimes(j), newBudget]);
            if better == 0
                for in = indices'
                    Q{in,1}(10) = true;
                end
                maxIndex = maxIndex+1;
                Q = heapPush(Q, [e, finishTimes(j), newBudget, bestNode(1), bestNode(2), wait, modes(j), bestNode(9), maxIndex, false, heuristic(e), finishTimes(j)+heuristic(e)], heuristic);
            end
        end
    end
    % Mark the current node as visited
    bestNode(10) = true;
Qexp(end+1,:) = bestNode;
    
end
Qexp(end+1,:) = bestNode;
if bestNode(1) == endVertex
    path = getPath(Qexp, maxWaits);
else
    disp("Path not found.")
    path = [];
end
end


%% TODO: Function to compute opr availability for a given edge at a given time
function [departureTimes, finishTimes, modes, newBudgets] = allExplorationTimes(arrivalTime, maxWait, autoTime, teleopTime, oprAvail)
% Given an edge, current time, budget and operator availability, compute which are all the departure times that we should be exploring
% For autonomous travel, we don't need to wait at all, and all the waiting time can be stored in the budget. This way even if the final path requires autonomous travel with waiting, we can use the budget.    

    departureTimes = zeros(maxWait+1,1); %arrivalTime;
    finishTimes = zeros(maxWait+1,1); %arrivalTime + autoTime; % One node to always expand is autonomous mode with no waiting
    modes = zeros(maxWait+1,1); %0;
    newBudgets = zeros(maxWait+1,1); %maxWait;
%     rank = find(oprAvail<=arrivalTime,1, 'last'); %rank of arrivalTime in oprAvail
%     if isempty(rank)
%         rank = 0;
%     end
    maxDepartureTime = arrivalTime + maxWait; % This is the max time we can depart at
%     maxOprAvailIdx = find(oprAvail<=maxDepartureTime,1, "last");
%     if mod(rank,2) == 1 % Means available right now
%         times = [arrivalTime, oprAvail(rank+2:2:maxOprAvailIdx)];
%     else % Means operator not available currently
%         times = [oprAvail(rank+1:2:maxOprAvailIdx)];
%     end
    times = arrivalTime:maxDepartureTime;
    for i = 1:maxWait+1
        currentTime = times(i);
        departureTimes(i) = currentTime;
        finishTimes(i) = currentTime + autoTime;
        modes(i) = 0;
        newBudgets(i) = maxDepartureTime-currentTime; 
    end
    for i = 1:maxWait+1
        currentTime = times(i);
        [avail, bud] = edgeAvailability(teleopTime, currentTime, oprAvail, maxDepartureTime-currentTime);
        if avail
            departureTimes(end+1) = currentTime;
            finishTimes(end+1) = currentTime + teleopTime;
            modes(end+1) = 1;
            newBudgets(end+1) = bud; 
        end
    end
end

function [result, budget] = edgeAvailability(teleopTime, startTime, oprAvail, oldBudget)
    result = 0;
    budget = 0;
    rank = find(oprAvail<=startTime,1, 'last'); %rank of arrivalTime in oprAvail
    if isempty(rank)
        rank = 0;
    end
    if mod(rank, 2) == 1 && (rank == size(oprAvail,2) || oprAvail(rank+1) > startTime + teleopTime) % Is operator is available throughout the duration of the edge
        result = 1;
        if rank == size(oprAvail,2)
            budget = oldBudget;
        else
            budget = min(oldBudget, oprAvail(rank+1) - 1 - (startTime + teleopTime)); % TODO: This will change if travel times are not constant.
            % There's a -1 in the previous line because the availability changes at that time. So, the operator is available onlt until the time [oprAvail(rank+1) - 1]
        end
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

function heap = heapPush(heap, node, heuristic)
% Push a node onto the heap
% The heap is sorted with increasing arrival time and if that's same then decreasing budget.
if isempty(heap)
    heap{1,1} = node;
else
    heap{end+1,1} = node; % Add the node to the end of the heap
end
idx = numel(heap); % Get the index of the new node
while idx > 1 % While the node is not the root
    parentIdx = floor(idx/2); % Get the index of the parent
   if heap{parentIdx}(12) > heap{idx}(12) || (heap{parentIdx}(12) == heap{idx}(12) && heap{parentIdx}(3) < heap{idx}(3)) %|| (isequal(heap{parentIdx}(2:3), heap{idx}(2:3)) && heap{parentIdx}(1) < heap{idx}(1))
     % Swap the parent and the node
        temp = heap{parentIdx};
        heap{parentIdx} = heap{idx};
        heap{idx} = temp;
        idx = parentIdx; % Update the index of the node
    else
        break; % Heap property is restored, so exit loop
    end
end
end

function [node, heap] = heapPop(heap, heuristic)
% Pop the root node from the heap
node = heap{1}; % Extract the root node
heap{1} = heap{end}; % Replace the root with the last element
heap(end) = []; % Remove the last element from the heap
idx = 1; % Set the index of the node to the root
while 2*idx <= numel(heap) % While the node has at least one child
    childIdx = 2*idx; % Set the index of the child to the left child
    if childIdx+1 <= numel(heap) % If the node has a right child
        % Set the index of the child to the right child if it has a smaller arrival time
        childIdx = childIdx + (heap{childIdx+1}(12) < heap{childIdx}(12));
    end
    g1 =  heap{childIdx}(12); % + heuristic(heap{childIdx}(1));
    g2 = heap{idx}(12); % + heuristic(heap{idx}(1));
    if g2 > g1 || (g2 == g1 && heap{idx}(3) < heap{childIdx}(3)) %|| (isequal(heap{idx}(2:3), heap{childIdx}(2:3)) && heap{idx}(1) < heap{childIdx}(1))
    % Swap the child and the node
        temp = heap{childIdx};
        heap{childIdx} = heap{idx};
        heap{idx} = temp;
        idx = childIdx; % Update the index of the node
    else
        break; % Heap property is restored, so exit loop
    end
end
end

function [better, location, locations] = isBetterNodePresent(heap, node)
% Check if the node with the same first three elements is present in the heap
better = false; % Initialize the return value to false
location = 0;
locations = [];
for i = 1:numel(heap) % Iterate through all elements in the heap
%     if isequal(heap{i}(1:2), node(1:2)) % If the first two elements of the current element match the given pair
%         better = true;
%         location = i;
%         break;
%     end
    if isequal(heap{i}(1:2), node(1:2)) % If the first two elements of the current element match the given pair
        if heap{i}(3) >= node(3)
            better = true;
            location = i;
            break;
        else % Same arrival is present with worse budget
            better = false;
            locations = [locations; i];
        end
%         break; % Exit the loop
    elseif heap{i}(1) == node(1) && heap{i}(2) <= node(2) && heap{i}(2) + heap{i}(3) >= node(2) + node(3)
        better = true;
        location = i;
        break;
    elseif heap{i}(1) == node(1) && heap{i}(2) >= node(2) && heap{i}(2) + heap{i}(3) <= node(2) + node(3)
        better = false;
        locations = [locations; i];
    end
end
end
