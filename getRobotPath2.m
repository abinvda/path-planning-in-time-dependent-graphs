function path = getRobotPath2(edges, startVertex, endVertex, A, B, oprAvail, maxWaits, maxTime)
% 'method' argument specifies which algorithm to use for computing path
% TODO: Greedy dijkstras is the one where we try to teleoperate whenever possible. 
% If opr is not available, we check if it is faster to wait and then teleoperate.

Q = cell(1, 1); % Initialize heap with starting vertex
Q{1} = [startVertex, 0, maxWaits(startVertex), 0, 0, 0, 0, 0, 1, false]; % Elements are [vertex, arrival time, budget, previous edge, its arrival time, and wait done there, mode of operation from there to current vertex, index of predecessor node, index of current node, visited flag]

while ~isempty(Q)
    % Pop the best element from the heap
    [bestNode, Q] = heapPop(Q);
    if bestNode(1) == endVertex
        break;
    end
    if bestNode(2) >= maxTime
        disp("Time's up!!")
        break;
    end
    if bestNode(10)
        continue; % Skip this node if it has already been visited
    end
    neighbours = edges(edges(:,1) == bestNode(1), 2); % In the edges matrix, find all where first element is the selected node
    arrivalTimes = bestNode(2) + A(bestNode(1), neighbours); % Compute arrival times for each neighbor
    budgets = bestNode(3) - A(bestNode(1), neighbours) + maxWaits(neighbours); % Compute budgets for each neighbor
    modes = zeros(size(neighbours)); % Initialize array for modes of operation
    modes(budgets >= B(bestNode(1), neighbours)) = 2; % Use teleoperation if budget allows it
    modes(budgets < B(bestNode(1), neighbours) & oprAvail(neighbours, modes+1)) = 1; % Use autonomy if teleoperation not possible but available
    waitTimes = modes .* (B(bestNode(1), neighbours) - budgets); % Compute wait times for each neighbor
    departureTimes = bestNode(2) + waitTimes; % Compute departure times for each neighbor
    finishTimes = departureTimes + A(bestNode(1), neighbours) + modes .* B(bestNode(1), neighbours); % Compute finish times for each neighbor
    
    % Add these neighbors to the heap with respective travel times
    for i = 1:numel(neighbours)
        e = neighbours(i);
        newBudget = budgets(i);
        % Add the neighbor to the heap if it has not already been visited
        if ~bestNode(10) && ~isNodePresent(Q, [e, finishTimes(i), newBudget])
            Q = heapPush(Q, [e, finishTimes(i), newBudget, bestNode(1), bestNode(2), waitTimes(i), modes(i), bestNode(9), maxIndex, false]);
        end
    end
    % Mark the current node as visited
    bestNode(10) = true;
    Q = heapPush(Q, bestNode);
end

% Extract the path from the heap
path = zeros(1, maxIndex);
node = bestNode;
while node(9) ~= 1
    path(node(9)) = node(1);
    node = heapPop(Q);
end
path(1) = node(1);
path = flip(path);

end

function heap = heapPush(heap, node)
% Push a node onto the heap
heap{end+1} = node; % Add the node to the end of the heap
idx = numel(heap); % Get the index of the new node
while idx > 1 % While the node is not the root
    parentIdx = idx/2; % Get the index of the parent
    if heap{parentIdx}(2) > heap{idx}(2) % If the parent has a larger arrival time than the node
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

function [node, heap] = heapPop(heap)
% Pop the root node from the heap
node = heap{1}; % Extract the root node
heap{1} = heap{end}; % Replace the root with the last element
heap(end) = []; % Remove the last element from the heap
idx = 1; % Set the index of the node to the root
while 2*idx <= numel(heap) % While the node has at least one child
    childIdx = 2*idx; % Set the index of the child to the left child
    if childIdx+1 <= numel(heap) % If the node has a right child
        % Set the index of the child to the right child if it has a smaller arrival time
        childIdx = childIdx + (heap{childIdx+1}(2) < heap{childIdx}(2));
    end
    if heap{idx}(2) > heap{childIdx}(2) % If the child has a smaller arrival time than the node
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

function present = isNodePresent(heap, node)
% Check if the node with the same first three elements is present in the heap
low = 1;
high = numel(heap);
while low <= high
    mid = floor((low + high) / 2);
    if isequal(heap{mid}(1:3), node(1:3))
        present = true;
        return;
    elseif heap{mid}(2) < node(2)
        low = mid + 1;
    else
        high = mid - 1;
    end
end
present = false;
end


