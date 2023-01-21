function [path, Q, Qexp] = getRobotPath(E, startVertex, endVertex, A, B, oprAvail, maxWaits, maxTime, heuristic, method)
% 'method' argument specifies which algorithm to use for computing path
% TODO: Greedy dijkstras is the one where we try to teleoperate whenever possible. 
% If opr is not available, we check if it is faster to wait and then teleoperate.

%% Convert oprAvail to edgeAvail
edgeAvail = getEdgeBudgets(E, B, maxTime, oprAvail);

%% Initialize Queue
Qexp = [];
Q = cell(1, 1); % Initialize heap with starting vertex
Q{1} = [startVertex, 0, maxWaits(startVertex), 0, 0, 0, 0, 0, 1, false, heuristic(startVertex), heuristic(startVertex)]; % Elements are [vertex, arrival time, budget, previous edge, its arrival time, and wait done there, mode of operation from there to current vertex, index of predecessor node, index of current node, visited flag]

% Store all arrival time intervals for each vertex in a variable. This will be used to speed up refinement process.
vertexArrivals = cell(size(A,1),1);
vertexArrivals{startVertex} = [0, maxWaits(startVertex)]; % Stores arrival time and budget pairs.

% For now, implementing the priority Q as a matrix that we'll sort or find minimim from.
% We can remove the previous vertex and its arrival time if we are using the node indices. Keeping them for testing and verification now.
maxIndex = 1;
while ~isempty(Q)
    % Pop the best element from the Q
    [bestNode, Q] = heapPop(Q, heuristic);
    if bestNode(1) == endVertex
        break;
    end
    if bestNode(2) + heuristic(bestNode(1)) > maxTime
        disp("Time's up!! Ours")
        break;
    end
    if method == "our" && ~any(ismember(vertexArrivals{bestNode(1)}, [bestNode(2) bestNode(3)], 'rows'))
        bestNode(10) = 1;
        continue;
    end
    if bestNode(10)
        continue; % Skip this node if it has already been visited
    end
%     neighbours = edges(edges(:,1) == bestNode(1), 2); % In the edges matrix, find all where first element is the selected node
    neighbours = find(E(bestNode(1),:))';

    arrivalTime = bestNode(2);
    budget = bestNode(3);
    autoTimes = A(bestNode(1), neighbours);
    teleopTimes = B(bestNode(1), neighbours);
        

    for i = 1:size(neighbours,1)
        e = neighbours(i);
        if method == "noref" && size(unique(vertexArrivals{e},'rows'),1) ~= size(vertexArrivals{e},1)
            aa = 1;
        end
        % for each neighbour, determine the times for which to expand, using function edgeAvailability
%         [departureTimes, finishTimes, modes, newBudgets] = edgeExplorationTimes(arrivalTime, budget, autoTimes(i), teleopTimes(i), oprAvail); % finishTimes corresponding to those departureTimes and mode of operation
        if method == "allT"
            [departureTimes, finishTimes, modes, newBudgets] = allExplorationTimes(bestNode(1), e, arrivalTime, budget, autoTimes(i), teleopTimes(i), oprAvail, edgeAvail);
            newBudgets = newBudgets + maxWaits(e);
            if ~isempty(vertexArrivals{e})
                [departureTimes, finishTimes, modes, newBudgets] = nodeRefinementAllT(vertexArrivals{e}, departureTimes, finishTimes, modes, newBudgets);
            end
        else
            [departureTimes, finishTimes, modes, newBudgets] = criticalExplorationTimes(bestNode(1), e, arrivalTime, budget, autoTimes(i), teleopTimes(i), oprAvail, edgeAvail); % finishTimes corresponding to those departureTimes and mode of operation
            newBudgets = newBudgets + maxWaits(e);
            if method == "noref" && ~isempty(vertexArrivals{e})
                [departureTimes, finishTimes, modes, newBudgets] = nodeRefinementNoRef(vertexArrivals{e}, departureTimes, finishTimes, modes, newBudgets);
            end
        end
        % add these neighbours to priorityQ with respective travel times
        for j = 1:size(departureTimes,2)
            wait = departureTimes(j) - bestNode(2);
            newBudget = newBudgets(j);
            if method == "noref"
                maxIndex = maxIndex+1;
                vertexArrivals{e} = [vertexArrivals{e}; [finishTimes(j), newBudget]];
                Q = heapPush(Q, [e, finishTimes(j), newBudget, bestNode(1), bestNode(2), wait, modes(j), bestNode(9), maxIndex, false, heuristic(e), finishTimes(j)+heuristic(e)], heuristic);
            elseif method == "allT"
                maxIndex = maxIndex+1;
                vertexArrivals{e} = [vertexArrivals{e}; [finishTimes(j), newBudget]];
                Q = heapPush(Q, [e, finishTimes(j), newBudget, bestNode(1), bestNode(2), wait, modes(j), bestNode(9), maxIndex, false, heuristic(e), finishTimes(j)+heuristic(e)], heuristic);
            else
                % We use the function below to check if the same node is present or if new one has higher budget
                [better, newRanges] = nodeRefinement(vertexArrivals{e}, finishTimes(j), newBudget);
    %             [better, location, indices] = isBetterNodePresent(Q, [e, finishTimes(j), newBudget]);
                if better == 0
                    vertexArrivals{e} = newRanges;
                    maxIndex = maxIndex+1;
                    Q = heapPush(Q, [e, finishTimes(j), newBudget, bestNode(1), bestNode(2), wait, modes(j), bestNode(9), maxIndex, false, heuristic(e), finishTimes(j)+heuristic(e)], heuristic);
                end
            end
        end
    end
    % Mark the current node as visited
    bestNode(10) = true;
%     Q = heapPush(Q, bestNode); % This way if this node is being added again, we know not to explore it. Need to check if this is really needed though.
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Later on we can also think of getting an approximation algorithms, by solving static graph with both autonomous and teleoperated costs.
    % This will give is min and max cost from a vertex to the goal.
    % So, we can use these to eliminate vertices from the search for which things are not within range
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Add this explored node to a separate list, which we'll use later to find the path
%     Qexp = heapPush(Qexp, bestNode);
Qexp(end+1,:) = bestNode;
    
end
% Do whatever to get the path
% Qexp = heapPush(Qexp, bestNode);
Qexp(end+1,:) = bestNode;  
if bestNode(1) == endVertex
    path = getPath(Qexp, maxWaits);
else
    disp("Path not found. Ours.")
    path = [0 0 0 0];
end
end


%% TODO: Function to compute opr availability for a given edge at a given time
function [departureTimes, finishTimes, modes, newBudgets] = criticalExplorationTimes(x,y,arrivalTime, budget, autoTime, teleopTime, oprAvail, edgeAvail)
% Given an edge, current time, budget and operator availability, compute which are all the departure times that we should be exploring
% For autonomous travel, we don't need to wait at all, and all the waiting time can be stored in the budget. This way even if the final path requires autonomous travel with waiting, we can use the budget.    

    departureTimes = arrivalTime; % Means no waiting
    finishTimes = arrivalTime + autoTime; % One node to always expand is autonomous mode with no waiting
    modes = 0;
    newBudgets = budget;
    
    % Check if operator is available right away
    rank = find(oprAvail<=arrivalTime,1, 'last'); %rank of arrivalTime in oprAvail
    if isempty(rank)
        rank = 0;
    end
    maxDepartureTime = min(size(edgeAvail,3)-1, arrivalTime + min(autoTime-teleopTime, budget)); % This is the max time we can depart at
    maxOprAvailIdx = find(oprAvail<=maxDepartureTime,1, "last");
    if mod(rank,2) == 1 % Means available right now
        times = [arrivalTime, oprAvail(rank+2:2:maxOprAvailIdx)];
    else % Means operator not available currently
        times = [oprAvail(rank+1:2:maxOprAvailIdx)];
    end
%     possibleTimes = edgeAvail(x,y,arrivalTime+1:maxDepartureTime+1);
%     possibleTimes = reshape(possibleTimes, 1, size(possibleTimes,3));
%     traversableTimes = find(possibleTimes > -1);
    for currentTime = times
%         [avail, bud] = edgeAvailability(currentTime, currentTime+teleopTime, oprAvail, maxDepartureTime-currentTime);
%         [avail, bud] = getEdgeBudget(x,y,startTime,A,B,edgeAvail);
        bud = edgeAvail(x,y,currentTime+1);
        % Can avoid this for loop by using buds = edgeAvail(edgeAvail(x,y,times+1)> -1);
        if bud >= 0
            departureTimes(end+1) = currentTime;
            finishTimes(end+1) = currentTime + teleopTime;
            modes(end+1) = 1;
            wait = currentTime - arrivalTime; 
            newBudgets(end+1) = min(bud, budget - wait); 
        end
    end
    [~,uniqueID,~] = unique([finishTimes; newBudgets]','rows');
    if numel(uniqueID) ~= numel(finishTimes)
        departureTimes = departureTimes(uniqueID);
        finishTimes = finishTimes(uniqueID);
        modes = modes(uniqueID);
        newBudgets = newBudgets(uniqueID);
    end
end

function [departureTimes, finishTimes, modes, newBudgets] = allExplorationTimes(x,y,arrivalTime, maxWait, autoTime, teleopTime, oprAvail, edgeAvail)
% This explores all times within arrival time and wait time
    maxDepartureTime = min(size(edgeAvail,3)-1, arrivalTime + maxWait);
    times = arrivalTime:maxDepartureTime;
    
    departureTimes = Inf(1, size(times,2));
    finishTimes = Inf(1, size(times,2)); 
    modes = Inf(1, size(times,2));
    newBudgets = Inf(1, size(times,2));
        
    for i = 1:size(times,2)
        currentTime = times(i);
        departureTimes(i) = currentTime;
        finishTimes(i) = currentTime + autoTime;
        modes(i) = 0;
        newBudgets(i) = 0; 
    end

    possibleTimes = edgeAvail(x,y,arrivalTime+1:maxDepartureTime+1);
    possibleTimes = reshape(possibleTimes, 1, size(possibleTimes,3));
    traversableTimes = arrivalTime + find(possibleTimes > -1) - 1;
    
    if ~isempty(traversableTimes)
        departureTimes = [departureTimes, Inf(1, size(traversableTimes,2))];
        finishTimes = [finishTimes, Inf(1, size(traversableTimes,2))];
        modes = [modes, Inf(1, size(traversableTimes,2))];
        newBudgets = [newBudgets, Inf(1, size(traversableTimes,2))];

        for i = 1:size(traversableTimes,2)
            currentTime = traversableTimes(i);
            departureTimes(i+size(times,2)) = currentTime;
            finishTimes(i+size(times,2)) = currentTime + teleopTime;
            modes(i+size(times,2)) = 1;
            newBudgets(i+size(times,2)) = 0;
        end
    end
end

% function [result, budget] = edgeAvailability(startTime, endTime, oprAvail, oldBudget)
%     result = 0;
%     budget = 0;
%     rank = find(oprAvail<=startTime,1, 'last'); %rank of arrivalTime in oprAvail
%     if isempty(rank)
%         rank = 0;
%     end
%     %rank2 = rank of startTime + teleopTime in oprAvail % We can also check if oprAvail(rank1) > startTime + teleopTime
%     if mod(rank, 2) == 1 && (rank == size(oprAvail,2) || oprAvail(rank+1) > endTime) % Is operator is available throughout the duration of the edge
%         result = 1;
%         if rank == size(oprAvail,2)
%             budget = oldBudget;
%         else
%             budget = min(oldBudget, oprAvail(rank+1) - 1 - endTime); % TODO: This will change if travel times are not constant.
%             % There's a -1 in the previous line because the availability changes at that time. So, the operator is available onlt until the time [oprAvail(rank+1) - 1]
%         end
%         % NOTE: The end time won't exceed the last element of oprAvail, which is selected to be a large number
%     end
% end


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
%     if heap{parentIdx}(2) > heap{idx}(2) % If the parent has a larger arrival time than the node
%     g1 =  heap{parentIdx}(2) + heuristic(heap{parentIdx}(1));
%     g2 = heap{idx}(2) + heuristic(heap{idx}(1));
%     if g1 > g2 || (g1 == g2 && heap{parentIdx}(3) < heap{idx}(3)) %|| (isequal(heap{parentIdx}(2:3), heap{idx}(2:3)) && heap{parentIdx}(1) < heap{idx}(1))
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
%     if heap{idx}(2) > heap{childIdx}(2) % If the child has a smaller arrival time than the node
%     if heap{idx}(2) > heap{childIdx}(2) || (heap{idx}(2) == heap{childIdx}(2) && heap{idx}(3) < heap{childIdx}(3)) || (isequal(heap{idx}(2:3), heap{childIdx}(2:3)) && heap{idx}(1) < heap{childIdx}(1))
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
% TODO: Also add other conditions to ignore nodes
% better = 1, means better already present in the heap
% location ~= 0 means, same node present, but this new one has better bedget, so we replace
% The value of location gives the index of that worse node in the heap to be replaced with this new one

% location = 0;
% low = 1;
% high = numel(heap);
% while low <= high
%     mid = floor((low + high) / 2);
%     temp = heap{mid};
%     if isequal(heap{mid}(1:2), node(1:2))
%         if heap{mid}(3) >= node(3)
%             better = true;
%         else % Same is present
%             better = false;
%             location = mid;
%         end
%         return;
%     elseif heap{mid}(2) < node(2)
%         low = mid + 1;
%     else
%         high = mid - 1;
%     end
% end
% better = false;

better = false; % Initialize the return value to false
location = 0;
locations = [];
for i = 1:numel(heap) % Iterate through all elements in the heap
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

function [better, allRanges] = nodeRefinement(allRanges, t, b)
% returns 0, 1: 1 for better in Q, 0 for non-enclosing or worse node in Q
% indices are the indices of nodes in llRanges that are worse than the current one
    better = 0;
    indices = [];
    for i = 1:size(allRanges,1)
        tb = allRanges(i,:);
        if tb(1) <= t && tb(1) + tb(2) >= t + b
            better = 1;
            break;
        elseif tb(1) >= t && tb(1) + tb(2) <= t + b
            indices = [indices; i];
%         else
%             better = 0;
        end
    end
    if better == 0
        allRanges(indices,:) = [];
        allRanges(end+1,:) = [t b];
    end
end

function [departureTimes, finishTimes, modes, newBudgets] = nodeRefinementAllT(existingTimes, departureTimes, finishTimes, modes, newBudgets)
    id = ~ismember(finishTimes, existingTimes(:,1));
    finishTimes = finishTimes(id);
    departureTimes = departureTimes(id);
    modes = modes(id);
    newBudgets = newBudgets(id);
end

function [departureTimes2, finishTimes2, modes2, newBudgets2] = nodeRefinementNoRef(existingTimes, departureTimes, finishTimes, modes, newBudgets)
    id = ~ismember([finishTimes; newBudgets]', existingTimes, 'rows');    
    departureTimes2 = departureTimes(id);
    modes2 = modes(id);
    newBudgets2 = newBudgets(id);
    finishTimes2 = finishTimes(id);
end

function entries = heap_to_matrix(Q)
%HEAP_TO_MATRIX Convert a binary heap into a matrix
%   heap: cell array representing the heap
%   entries: matrix containing the entries from the heap

% Initialize the matrix
entries = zeros(numel(Q), numel(Q{1}));

% Copy the entries from the heap into the matrix
for i = 1:numel(Q)
    entries(i, :) = Q{i};
end
entries = sortrows(entries, [2,1,3,12]);
end

function indices = findWorseNodes(Q, node)
% Find all indices in the heap where the first three elements of a node match a given node
%   heap: cell array representing the heap
%   node: node to be searched for in the heap
%   indices: array of indices where the first three elements of a node match the given node

indices = []; % Initialize the array of indices

i = 1; % Initialize the index of the current node
while i <= numel(Q) % While the current node is within the bounds of the heap
    if isequal(Q{i}(1:2), node(1:2)) &&  Q{i}(3) <= node(3) % If the first three elements of the node match the given node
        indices = [indices, i]; % Add the index to the array of indices
%     elseif Q{i}(2) > node(2) || (Q{i}(2) == node(2) && Q{i}(3) < node(3))
%         % The current node has a larger arrival time or lesser budget than the given node, so it cannot be present in the heap
%         break;
    end
    i = i + 1; % Move to the next node
end

end
