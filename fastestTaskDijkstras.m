function [path, Q, Qexp] = fastestTaskDijkstras(startVertex, goalVertex, A, B, edgeAvail, maxWaits, distToGoal, startTime)
% Solve shortest path problem using fastest task version of Dijktra's

%% Convert oprAvail to edgeAvail
% edgeAvail = getEdgeBudgets(E, B, maxTime, oprAvail);

nNodes = size(A,1);
arrivalTimes = Inf(nNodes, 1);  
predecessors = zeros(nNodes,4);
done = zeros(nNodes, 1);
arrivalTimes(startVertex) = startTime; % Starting from vertex 1 at t = StartTime

Q = cell(1, 1); % Initialize heap with starting vertex
Q{1} = [startVertex, arrivalTimes(startVertex)+distToGoal(startVertex)];
Qexp = [];
while ~isempty(Q)
    % Pop the best element from the Q
    [bestNode, Q] = heapPop(Q);
    done(bestNode(1)) = 1;
    Qexp(end+1) = bestNode(1);
    if bestNode(1) == goalVertex
        break;
    end
    neighbours = find(A(bestNode(1),:) > 0); % In the edges matrix, find all where first element is the selected node
    %   neighbours = edges(edges(:,1) == bestNode(1,1), 2); % In the edges matrix, find all where first element is the selected node
    %   neighbours = cell2mat(edges(bestNode(1,1),1)); %(edges(:,1) == bestNode(1,1), 2); % In the edges matrix, find all where first element is the selected node
    for i = 1:size(neighbours,2)
        e = neighbours(i);
        if ~done(e)
            edgeStart = arrivalTimes(bestNode(1));
            dt = timeToOprAvail(bestNode(1), e, edgeStart, edgeStart+maxWaits(bestNode(1)), edgeAvail);
            fastestTime = min(A(bestNode(1), e), dt + B(bestNode(1), e));
            arrivalTime = arrivalTimes(bestNode(1)) + fastestTime;
            if arrivalTime < arrivalTimes(e)
                arrivalTimes(e) = arrivalTime;
                if A(bestNode(1), e) <= dt + B(bestNode(1), e)
                    wait = 0;
                    mode = 0;
                else
                    wait = dt;
                    mode = 1;
                end
                predecessors(e, :) = [bestNode(1), arrivalTimes(bestNode(1)), wait, mode];
                Q = heapPush(Q, [e, arrivalTime+distToGoal(e)]);
            end
        end
    end
    % Do some cleaning, remove redundant nodes from the Q
end
% Get the path
if bestNode(1) == goalVertex
    path = getPath(predecessors, startVertex, goalVertex, bestNode(2)-distToGoal(goalVertex)); % distToGoal(goalVertex) = 0, but still keeping it to avoid any future errors
else
    disp("Static Path not found.")
    path = [];
end
end


%% TODO: Function to compute opr availability for a given edge at a given time
function path = getPath(predecessors, startVertex, goalVertex, goalArrivalTime)
    currentNode = goalVertex;
    path = [goalVertex, goalArrivalTime, 0, 0]; % [vertex, arrivalTime, wait, mode]
    % We need to match vertex and arrival time and pick the one with largest budget
    while currentNode(1) ~= startVertex %~(currentNode(1) == 1 && currentNode(2) == 0)
        pred = predecessors(currentNode(1),:);
        path = [pred; path];
        currentNode = pred;
    end
end

function dt = timeToOprAvail(x, y, arrivalTime, maxDepartureTime, edgeAvail)  
    if maxDepartureTime+1 > size(edgeAvail, 3)
        dt = Inf;
    else
        possibleTimes = edgeAvail(x,y,arrivalTime+1:maxDepartureTime+1);
        possibleTimes = reshape(possibleTimes, 1, size(possibleTimes,3));
        traversableTimes = arrivalTime + find(possibleTimes > -1) - 1;
    
        if isempty(traversableTimes) % Means not available at any of the possible times
            dt = Inf;
        else % Means operator can be made available
    
            dt = traversableTimes(1) - arrivalTime;
        end
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
