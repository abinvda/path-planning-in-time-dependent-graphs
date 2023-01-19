%% Edge availability pre-computation
B = magic(4);
T = 25;
oprAvail = [0,8,12,20];
e1 = edgeAvailability(B, T, oprAvail);
% e2 = edgeAvailability2(B, T, oprAvail);

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
            currE = (B <= value) .* (value-B);
            newE(:,:,t+1) = currE; % t+1 because we start with t=0
            newE(:,:,t+1) = B<=value;
        end
    end 
end

function newE = edgeAvailability2(B, T, oprAvail)
    newE = zeros(size(B,1),size(B,2), T+1);
    for i = 1:ceil(size(oprAvail,2)/2)
        tCurr = oprAvail(2*i-1);
        changeTime = oprAvail(2*i);
        value = changeTime-1-tCurr:changeTime-2;
        update_index = B <= value;
        newE(:,:,tCurr:changeTime-1) = update_index .* (value-B);
    end 
end





% function path = timeExpandedDijkstras(edges, startVertex, goalVertex, A, T, heuristic)
% % Solve shortest path problem for time-expanded graph using A-star
% 
% nVertices = size(A,1);
% nNodes = nVertices;
% arrivalTimes = Inf(nNodes, T+1); % For t = [0,T]
% predecessors = zeros(2, nNodes,T+1); % Stores (vertex, time) for all vertex time pairs.
% done = zeros(nNodes, T+1);
% arrivalTimes(startVertex, 1) = 0;
% 
% Q = cell(1, 1); % Initialize heap with starting vertex
% Q{1} = [startVertex, 0, arrivalTimes(startVertex, 1)]; % In our search, a node is (vertex, time, arrivalTime), arrivalTime = Inf if the vetex-time pair is unachievable, else = time
% while ~isempty(Q)
%     % Pop the best element from the Q
%     [bestNode, Q] = heapPop(Q);
%     done(bestNode(1), bestNode(2)) = 1;
%     if bestNode(1) == goalVertex && bestNode(2) <= T
%         break;
%     end
%     neighbours = find(A(bestNode(1),:) > 0); % In the edges matrix, find all where first element is the selected node
%     for i = 1:size(neighbours,2)
%         e = neighbours(i);
%         aTime = 
%         if ~done(e)
%             arrivalTime = bestNode(2) + A(bestNode(1), e);
%             if arrivalTime < arrivalTimes(e)
%                 arrivalTimes(e) = arrivalTime;
%                 predecessors(e) = bestNode(1);
%                 Q = heapPush(Q, [e, arrivalTime]);
%             end
%         end
%     end
%     % Do some cleaning, remove redundant nodes from the Q
% end 
% % Get the path
% if bestNode(1) == goalVertex
%     if all
%         path = arrivalTimes;
%     else
%         path = getPathStatic(predecessors, arrivalTimes, startVertex, goalVertex);
%     end
% else
%     disp("Path not found.")
%     path = [];
% end
% end
% 
% 
% %% TODO: Function to compute opr availability for a given edge at a given time
% function path = getPathStatic(predecessors, arrivalTimes, startVertex, goalVertex)
%     currentNode = goalVertex;
%     path = [goalVertex, arrivalTimes(goalVertex)];
%     % We need to match vertex and arrival time and pick the one with largest budget
%     while currentNode ~= startVertex %~(currentNode(1) == 1 && currentNode(2) == 0) 
%         path = [predecessors(currentNode) arrivalTimes(predecessors(currentNode)); path];
%         currentNode = predecessors(currentNode);
%     end
% end
% 
% function heap = heapPush(heap, node)
% % Push a node onto the heap
% % The heap is sorted with increasing arrival time and if that's same then decreasing budget.
% if isempty(heap)
%     heap{1,1} = node;
% else
%     heap{end+1,1} = node; % Add the node to the end of the heap
% end
% idx = numel(heap); % Get the index of the new node
% while idx > 1 % While the node is not the root
%     parentIdx = floor(idx/2); % Get the index of the parent
%      if heap{parentIdx}(2) > heap{idx}(2)
%      % Swap the parent and the node
%         temp = heap{parentIdx};
%         heap{parentIdx} = heap{idx};
%         heap{idx} = temp;
%         idx = parentIdx; % Update the index of the node
%     else
%         break; % Heap property is restored, so exit loop
%     end
% end
% end
% 
% function [node, heap] = heapPop(heap)
% % Pop the root node from the heap
% node = heap{1}; % Extract the root node
% heap{1} = heap{end}; % Replace the root with the last element
% heap(end) = []; % Remove the last element from the heap
% idx = 1; % Set the index of the node to the root
% while 2*idx <= numel(heap) % While the node has at least one child
%     childIdx = 2*idx; % Set the index of the child to the left child
%     if childIdx+1 <= numel(heap) % If the node has a right child
%         % Set the index of the child to the right child if it has a smaller arrival time
%         childIdx = childIdx + (heap{childIdx+1}(2) < heap{childIdx}(2));
%     end
%     g1 =  heap{childIdx}(2);
%     g2 = heap{idx}(2);
%     if g2 > g1
%         temp = heap{childIdx};
%         heap{childIdx} = heap{idx};
%         heap{idx} = temp;
%         idx = childIdx; % Update the index of the node
%     else
%         break; % Heap property is restored, so exit loop
%     end
% end
% end
% 
