function path = getRobotPath(edges, startVertex, endVertex, A, B, oprAvail, maxWaits, maxTime, heuristic)
% 'method' argument specifies which algorithm to use for computing path
% TODO: Greedy dijkstras is the one where we try to teleoperate whenever possible. 
% If opr is not available, we check if it is faster to wait and then teleoperate.

% Qexp = cell(1,1);
% Qexp{1} = [startVertex, 0, maxWaits(startVertex), 0, 0, 0, 0, 0, 1, false]; % TODO: Remove this later.
Qexp = [];
% QSize = 10000;
% QEmpty = repmat([0 10000 0 0 0 0 0 0 0], QSize,1);
% Q = [startVertex, 0, maxWaits(startVertex), 0, 0, 0, 0, 0, 1]; % Elements are [vertex, arrival time, budget, previous edge, its arrival time, and wait done there, mode of operation from there to current vertex, index of predecessor node, index of current node]
% Q = [Q; QEmpty];

Q = cell(1, 1); % Initialize heap with starting vertex
Q{1} = [startVertex, 0, maxWaits(startVertex), 0, 0, 0, 0, 0, 1, false, heuristic(startVertex), heuristic(startVertex)]; % Elements are [vertex, arrival time, budget, previous edge, its arrival time, and wait done there, mode of operation from there to current vertex, index of predecessor node, index of current node, visited flag]

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
        [departureTimes, finishTimes, modes, newBudgets] = edgeExplorationTimes(arrivalTime, budget, autoTimes(i), teleopTimes(i), oprAvail); % finishTimes corresponding to those departureTimes and mode of operation
        % add these neighbours to priorityQ with respective travel times
        for j = 1:size(departureTimes,2)
            wait = departureTimes(j) - bestNode(2);
            newBudget = newBudgets(j) + maxWaits(e);
            % We use the function below to check if the same node is present or if new one has higher budget
            [better, location, indices] = isBetterNodePresent(Q, [e, finishTimes(j), newBudget]);
            if better == 0
%                 indices = findWorseNodes(Q, [e, finishTimes(j), newBudget]);
%                 tt = 0;
%                 for in = indices'
%                     if Q{in,1}(10) == 0
%                         tt = tt + 1;
%                     end
% %                     Q{in,1}(10) = true;
%                 end
%                 if tt > 1
%                     for in = indices'
%                         Q{in,1}
%                     end
%                     aa = 1
%                 end
                for in = indices'
                    Q{in,1}(10) = true;
                end
%                 if location ~= 0 % location is the index of worse node
%                     Q{location}(10) = true; % set it to visited so we don't explore it.
%                 end
                maxIndex = maxIndex+1;
                Q = heapPush(Q, [e, finishTimes(j), newBudget, bestNode(1), bestNode(2), wait, modes(j), bestNode(9), maxIndex, false, heuristic(e), finishTimes(j)+heuristic(e)], heuristic);
            end
        end
    end
    % Mark the current node as visited
    bestNode(10) = true;
%     Q = heapPush(Q, bestNode); % This way if this node is being added again, we know not to explore it. Need to check if this is really needed though.
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Later on we can also think of getting an approximation algorithms, by solving static graph with both autonomous and teleoperated costs.
    % This will give is min and max cost from a vertex to the goal.
    % So, we can use these to eliminate vertices from the search for which things are not very good
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
            budget = min(oldBudget, oprAvail(rank+1) - 1 - (startTime + teleopTime)); % TODO: This will change if travel times are not constant.
            % There's a -1 in the previous line because the availability changes at that time. So, the operator is available onlt until the time [oprAvail(rank+1) - 1]
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

% function heap = heapDelete(heap, delete_idx)
% %DELETE Delete an entry from a binary heap at a given location
% %   heap: cell array representing the heap
% %   delete_idx: index of the entry to be deleted
% 
% % Delete the entry at the given index
% heap{delete_idx} = heap{end};
% heap(end) = [];
% 
% % Maintain the heap property by repeatedly swapping the entry at the delete index with its smaller child
% while true
%     left_child_idx = 2 * delete_idx;
%     right_child_idx = 2 * delete_idx + 1;
% 
%     if left_child_idx > numel(heap)
%         % There are no more children, so we can exit the loop
%         break;
%     elseif right_child_idx > numel(heap)
%         % There is only a left child, so compare the entry to the left child and we need only one swap
%         if heap{delete_idx}(2) > heap{left_child_idx}(2) || (heap{delete_idx}(2) == heap{left_child_idx}(2) && heap{delete_idx}(3) < heap{left_child_idx}(3))
%             temp = heap{delete_idx};
%             heap{delete_idx} = heap{left_child_idx};
%             heap{left_child_idx} = temp;
%         end
%         break;
%     else
%         % There are both left and right children, so compare the entry to the smaller of the two
%         if heap{left_child_idx}(2) < heap{right_child_idx}(2) || (heap{left_child_idx}(2) == heap{right_child_idx}(2) && heap{left_child_idx}(3) > heap{right_child_idx}(3))
%             if heap{delete_idx}(2) > heap{left_child_idx}(2)
%                 heap = swap(heap, delete_idx, left_child_idx);
%                 delete_idx = left_child_idx;
%             else
%                 break;
%             end
%         else
%             if heap{delete_idx}(2) > heap{right_child_idx}(2)
%                 heap = swap(heap, delete_idx, right_child_idx);
%                 delete_idx = right_child_idx;
%             else
%                 break;
%             end
%         end
%     end
% end
% end

% function heap = delete(heap, delete_idx)
% %DELETE Delete an entry from a binary heap at a given location
% %   heap: cell array representing the heap
% %   delete_idx: index of the entry to be deleted
% 
% % Delete the entry at the given index
% heap{delete_idx} = heap{end};
% heap(end) = [];
% 
% % Maintain the heap property by repeatedly swapping the entry at the delete index with its parent
% % if the parent has a larger arrival time or budget than the entry, or with its child if the entry has a smaller arrival time or budget
% while true
%     % Calculate the index of the parent node
%     parent_idx = floor(delete_idx / 2);
%     
%     if delete_idx > 1 && (heap{parent_idx}(2) > heap{delete_idx}(2) || (heap{parent_idx}(2) == heap{delete_idx}(2) && heap{parent_idx}(3) < heap{delete_idx}(3)))
%         % The entry has a smaller arrival time or equal arrival time and a larger budget than the parent, so swap them
%         heap = swap(heap, parent_idx, delete_idx);
%         delete_idx = parent_idx; % Update the index of the entry
%     else
%         % The heap property has been restored, so we can exit the loop
%         break;
%     end
% end
% while true
%     left_child_idx = 2 * delete_idx;
%     right_child_idx = 2 * delete_idx + 1;
% 
%     if left_child_idx > numel(heap)
%         % There are no more children, so we can exit the loop
%         break;
%     elseif right_child_idx > numel(heap)
%         % There is only a left child, so compare the entry to the left child and we need only one swap
%         if heap{delete_idx}(2) > heap{left_child_idx}(2) || (heap{delete_idx}(2) == heap{left_child_idx}(2) && heap{delete_idx}(3) < heap{left_child_idx}(3))
%             temp = heap{delete_idx};
%             heap{delete_idx} = heap{left_child_idx};
%             heap{left_child_idx} = temp;
%         end
%         break;
%     else
%         % There are both left and right children, so compare the entry to the smaller of the two
%         if heap{left_child_idx}(2) < heap{right_child_idx}(2) || (heap{left_child_idx}(2) == heap{right_child_idx}(2) && heap{left_child_idx}(3) > heap{right_child_idx}(3))
%             if heap{delete_idx}(2) > heap{left_child_idx}(2)
%                 heap = swap(heap, delete_idx, left_child_idx);
%                 delete_idx = left_child_idx;
%             else
%                 break;
%             end
%         else
%             if heap{delete_idx}(2) > heap{right_child_idx}(2)
%                 heap = swap(heap, delete_idx, right_child_idx);
%                 delete_idx = right_child_idx;
%             else
%                 break;
%             end
%         end
%     end
% end
% end

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
entries = sortrows(entries, [2,1,3,10]);
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
