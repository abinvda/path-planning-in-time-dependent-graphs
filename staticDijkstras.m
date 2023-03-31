function path = staticDijkstras(edges, startVertex, goalVertex, A, ~)
% Solve shortest path problem for static graph using Dijktra's

if nargin == 5
    all = 1;
else
    all = 0;
end
nNodes = size(A,1);
arrivalTimes = Inf(nNodes, 1);
predecessors = zeros(nNodes,1);
done = zeros(nNodes, 1);
arrivalTimes(startVertex) = 0; % Starting from vertex 1 at t = 1

Q = cell(1, 1); % Initialize heap with starting vertex
Q{1} = [startVertex, arrivalTimes(startVertex)];
while ~isempty(Q)
    % Pop the best element from the Q
    [bestNode, Q] = heapPop(Q);
    if done(bestNode(1)) == 1
        continue;
    end
    done(bestNode(1)) = 1;
    if ~all
        if bestNode(1) == goalVertex
            break;
        end
    end
    neighbours = find(A(bestNode(1),:) > 0); % In the edges matrix, find all where first element is the selected node
%   neighbours = edges(edges(:,1) == bestNode(1,1), 2); % In the edges matrix, find all where first element is the selected node
%   neighbours = cell2mat(edges(bestNode(1,1),1)); %(edges(:,1) == bestNode(1,1), 2); % In the edges matrix, find all where first element is the selected node
    for i = 1:size(neighbours,2)
        e = neighbours(i);
        if ~done(e)
            arrivalTime = bestNode(2) + A(bestNode(1), e);
            if arrivalTime < arrivalTimes(e)
                arrivalTimes(e) = arrivalTime;
                predecessors(e) = bestNode(1);
                Q = heapPush(Q, [e, arrivalTime]);
            end
        end
    end
    % Do some cleaning, remove redundant nodes from the Q
end 
% Get the path
if ~all
    if bestNode(1) == goalVertex
        path = getPathStatic(predecessors, arrivalTimes, startVertex, goalVertex);
    else
        disp("Static Path not found.")
        path = [];
    end
else
    path = arrivalTimes;
end
end


%% TODO: Function to compute opr availability for a given edge at a given time
function path = getPathStatic(predecessors, arrivalTimes, startVertex, goalVertex)
    currentNode = goalVertex;
    path = [goalVertex, arrivalTimes(goalVertex)];
    % We need to match vertex and arrival time and pick the one with largest budget
    while currentNode ~= startVertex %~(currentNode(1) == 1 && currentNode(2) == 0) 
        path = [predecessors(currentNode) arrivalTimes(predecessors(currentNode)); path];
        currentNode = predecessors(currentNode);
    end
end

function heap = heapPush(heap, node)
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
     if heap{parentIdx}(2) > heap{idx}(2)
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
    g1 =  heap{childIdx}(2);
    g2 = heap{idx}(2);
    if g2 > g1
        temp = heap{childIdx};
        heap{childIdx} = heap{idx};
        heap{idx} = temp;
        idx = childIdx; % Update the index of the node
    else
        break; % Heap property is restored, so exit loop
    end
end
end

