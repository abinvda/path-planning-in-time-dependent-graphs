function path = staticDijkstras(edges, startVertex, goalVertex, A, all)
% Solve shortest path problem for static graph using Dijktra's

if nargin == 5
    all = 1;
else
    all = 0;
end
Qexp = [];
QSize = 10000;
nNodes = size(A,1);
arrivalTimes = Inf(nNodes, 1);
predecessors = zeros(nNodes,1);
done = zeros(nNodes, 1);
arrivalTimes(startVertex) = 0; % Starting from vertex 1 at t = 1

Q = [startVertex, arrivalTimes(startVertex)];
while ~isempty(Q)
    % Pop the best element from the Q
    [~, bestIndex] = min(Q(:,2));
    bestNode = Q(bestIndex,:);
    done(bestNode(1,1)) = 1;
    if bestNode(1,1) == goalVertex
        break;
    end
%     if bestNode(1,2) >= 100000
%         disp("Time's up!!")
%         break;
%     end
    neighbours = edges(edges(:,1) == bestNode(1,1), 2); % In the edges matrix, find all where first element is the selected node
%   neighbours = cell2mat(edges(bestNode(1,1),1)); %(edges(:,1) == bestNode(1,1), 2); % In the edges matrix, find all where first element is the selected node
    for i = 1:size(neighbours,1)
        e = neighbours(i);
        if ~done(e)
            arrivalTime = bestNode(1,2) + A(bestNode(1,1), e);
            if arrivalTime < arrivalTimes(e)
                arrivalTimes(e) = arrivalTime;
                predecessors(e) = bestNode(1,1);
                Q(end+1,:) = [e, arrivalTime];
            end
        end
    end
%     Q(1,:) = []; % Remove the currently explored node
    Q(bestIndex,:) = []; % Remove the currently explored node
    % Do some cleaning, remove redundant nodes from the Q
end 
% Get the path
if bestNode(1,1) == goalVertex
    if all
        path = arrivalTimes;
    else
        path = getPathStatic(predecessors, arrivalTimes, startVertex, goalVertex);
    end
else
    disp("Path not found.")
    path = [];
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