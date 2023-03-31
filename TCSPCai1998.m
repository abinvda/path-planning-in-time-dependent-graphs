function path = TCSPCai1998(edges, startVertex, goalVertex, A, B, oprAvail, edgeAvail, maxWaits, T)

nNodes = size(A,1);
%% Convert oprAvail to edgeAvail
% edgeAvail = edgeAvailability(B, T, oprAvail);

%% Initialize
dC = Inf(nNodes, T+1); % Stores cost to a node (row) of at max time t (column), t = [0,T]
dC(startVertex,1) = 0; % Corresponds to d(s,0)
d_opt = Inf(nNodes,1);
heaps = cell(nNodes,1);
dm = Inf(nNodes,T+1);
for x = 1:nNodes
%     heaps{x} = cell(1,1); 
%     heaps{x}{1} = [0, dC(x,1)];
    heaps{x} = [0, dC(x,1)]; %[t, d(x,t)] % This is for t = 0
    dm(x,1) = dC(x,1);
end
arrivalTimes = zeros(T*size(edges,1), 4); % [edgeStart, edgeEnd, arrivalAtStart, arrivalAtEnd]
for i = 1:size(edges,1)
    x = edges(i,1);
    y = edges(i,2);
    for u = 0:T
        edgeTime = getEdgeTime(x,y,u,A,B,edgeAvail);
        arrivalTimes((i-1)*T + u+1,:) = [x,y,u, u+edgeTime]; % For this time is starting from t=1 in the paper.
    end
end
arrivalTimes = sortrows(arrivalTimes, 4);

%% Store edge number for quick access later when updating gamma
edgeNum = zeros(nNodes, nNodes*2);
for i = 1:size(edges,1)
    x = edges(i,1);
    y = edges(i,2);
    edgeNum(x,y) = i;
end

%% Main loop

for t = 1:T
    % Initialize Gamma the arrival time at a vertex y through an edge e exactly at time t
    gamma = zeros(size(edges,1), 3); % [x,y,gamma(x,y)]
    for i = 1:size(edges,1)
        x = edges(i,1);
        y = edges(i,2);
        gamma(i,:) = [x,y,Inf];
    end
    all_uD = find(arrivalTimes(:,4) == t);
    for i = all_uD'
        x = arrivalTimes(i,1);
        y = arrivalTimes(i,2);
        e = edgeNum(x,y); %find(ismember(edges, [x,y], 'rows'));
        
        uD = arrivalTimes(i,3);
        gamma(e,3) = min(gamma(e,3), dm(x,uD+1) + getEdgeTime(x,y,uD,A,B,edgeAvail));
%         gamma(e,3) = min(gamma(e,3), max(uD, dm(x,uD+1)) + getEdgeTime(x,y,uD,A,B,oprAvail));
%         if gamma(e,3) < Inf
%             aa = 1;
%         end
    end
    for y = 1:nNodes
        gammaY1 = gamma(gamma(:,2)==y,3); % Gamma for autonomous edges
        gammaY2 = gamma(gamma(:,2)==y+nNodes,3); % For teleoperated edges
        gammaY = [gammaY1; gammaY2];
        if ~isempty(gammaY)
            dC(y,t+1) = min(gammaY);
        end
    end
    for y = 1:nNodes
%         heaps{y} = heapPush(heaps{y}, [t, dC(y,t+1)]);
        heaps{y} = sortrows([heaps{y}; [t, dC(y,t+1)]], 2);
        if t > maxWaits(y)
            heaps{y}(heaps{y}(:,1) == t-maxWaits(y)-1, :) = []; % Delete from the heap, all entries with time = t-maxW-1
%             heaps{y} = heapDelete(heaps{y}, t-maxWaits(y)-1);
        end
        heaps{y} = sortrows(heaps{y},2);
    end
    for y = 1:nNodes
%         minNode = heapPop(heaps{y}); 
%         uA = minNode(1); %
        uA = heaps{y}(1,1); % Time for minimum dC(y,t)
        if dC(y,uA+1) < Inf
            dm(y,t+1) = max(t,dC(y,uA+1)); % This max operation (along with this if condition) stores the time taken instead of cost of travel.
%         dm(y,t+1) = d(y,uA+1); % This gives cost of travel and is same as the paper
        end
    end
    if dm(goalVertex, t+1) < Inf
        break;
    end
end
for y = 1:nNodes
    d_opt(y) = min(dC(y,:));
end
path = getPath(startVertex, goalVertex, d_opt(goalVertex), dC, arrivalTimes, maxWaits, nNodes);
    
end

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

function time = getEdgeTime(x,y,startTime,A,B,edgeAvail)
    if y <= size(A,1) % Means it's the autonomous edge
        time = A(x,y);
    else % Means it's the teleoperated edge
        y = y - size(A,1);
        if edgeAvail(x,y,startTime+1) == 1
            time = B(x,y);
        else
            time = Inf;
        end
%         rank = find(oprAvail<=startTime,1, 'last'); %rank of arrivalTime in oprAvail
%         if isempty(rank)
%             rank = 0;
%         end
%         if mod(rank, 2) == 1 && (rank == size(oprAvail,2) || oprAvail(rank+1) > startTime + B(x,y)) % Is operator is available throughout the duration of the edge
%             time = B(x,y);
%         else
%             time = Inf;
%         end
    end
end

function path = getPath(startVertex, goalVertex, goalArrivalTime, d, arrivalTimes, maxWaits, nNodes)
    % Path is stored as [vertex, arrival time, wait, mode]
%     path = [goalVertex, goalArrivalTime, 0, 0];
% %     currentNode = find(ismember(arrivalTimes(:,[2,4]), [goalVertex, goalArrivalTime; goalVertex+nNodes, goalArrivalTime], 'rows'), 1);
%     currentNode = find(ismember(arrivalTimes(:,[2,4]), [goalVertex, goalArrivalTime; goalVertex+nNodes, goalArrivalTime], 'rows'), 1);
%     currentNode = arrivalTimes(currentNode,:);
    path = [];
    currentNode = [goalVertex, goalVertex, goalArrivalTime, goalArrivalTime];
    b = true;
    while b %currentNode(1) ~= 1 %~(currentNode(1) == 1 && currentNode(2) == 0)
        predDeparture = currentNode(3);
        if currentNode(1) == startVertex && predDeparture <= maxWaits(currentNode(1))
            b = false;
            selectedPred = [startVertex startVertex 0 0];
        else
        
            possibleArrivals = predDeparture:-1:predDeparture-maxWaits(currentNode(1));
            possibleVertices = [currentNode(1), currentNode(1)+nNodes];
            [p,q] = meshgrid(possibleVertices, possibleArrivals);
            possibleNodes = [p(:) q(:)];
            
            predecessorNodes = ismember(arrivalTimes(:,[2,4]), possibleNodes, 'rows');
            predecessorNodes = arrivalTimes(predecessorNodes,:);
            for i = 1:size(predecessorNodes,1)
                p = predecessorNodes(i,:);
                id = (p(3):-1:p(3)-maxWaits(p(1)))+1; % p(3) is the departure time, so this line gives us all possible arrival times at this predecessor node
                id = id(id>0);
                if min(d(p(1), id)) < Inf
                    selectedPred = p;
                    break
                end
            end
        end
        currentWait = predDeparture - selectedPred(4);
        if currentNode(2) <= nNodes
            mode = 0;
        else
            mode = 1;
        end
        path = [currentNode(1), selectedPred(4), currentWait, mode; path];
        currentNode = selectedPred;
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
    g1 =  heap{childIdx}(2); % + heuristic(heap{childIdx}(1));
    g2 = heap{idx}(2); % + heuristic(heap{idx}(1));
    if g2 > g1
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

function heap = heapDelete(heap, x)
    % Find all nodes that match the condition
    deleteIndices = cellfun(@(node) node(1) == x, heap);
    % Remove all matching nodes from the heap
    heap(deleteIndices) = [];
    % Rebuild the heap
    heap = buildHeap(heap);
end

function heap = buildHeap(heap)
    % Iterate through all nodes in the heap
    for i = floor(numel(heap)/2):-1:1
        % Call the heapify function to maintain the heap property
        heap = heapify(heap, i);
    end
end

function heap = heapify(heap, i)
    % Get the indices of the left and right children
    left = 2*i;
    right = 2*i + 1;
    % Find the index of the smallest value among the node and its children
    if left <= numel(heap) && heap{left}(2) < heap{i}(2)
        smallest = left;
    else
        smallest = i;
    end
    if right <= numel(heap) && heap{right}(2) < heap{smallest}(2)
        smallest = right;
    end
    % If the node is not the smallest, swap it with its smallest child and recursively heapify the subtree
    if smallest ~= i
        temp = heap{i};
        heap{i} = heap{smallest};
        heap{smallest} = temp;
        heap = heapify(heap, smallest);
    end
end

%% With heap implementation. Has bugs
% function path = TCSPCai1998(edges, goalVertex, A, B, oprAvail, maxWaits, T)
% 
% nNodes = size(A,1);
% %% Initialize
% dC = Inf(nNodes, T+1); % Stores cost to a node (row) of at max time t (column), t = [0,T]
% dC(1,1) = 0; % Corresponds to d(s,0)
% d_opt = Inf(nNodes,1);
% heaps = cell(nNodes,1);
% dm = Inf(nNodes,T+1);
% for x = 1:nNodes
%     heaps{x} = MinHeap();
%     heaps{x}.push([0, dC(x,1)]); % [t, dC(x,t)] % This is for t = 0
%     dm(x,1) = dC(x,1);
% end
% arrivalTimes = zeros(T*size(edges,1), 4); % [edgeStart, edgeEnd, arrivalAtStart, arrivalAtEnd]
% for i = 1:size(edges,1)
%     x = edges(i,1);
%     y = edges(i,2);
%     for u = 0:T
%         edgeTime = getEdgeTime(x,y,u,A,B,oprAvail);
%         arrivalTimes((i-1)*T + u+1,:) = [x,y,u, u+edgeTime]; % For this time is starting from t=1 in the paper.
%     end
% end
% arrivalTimes = sortrows(arrivalTimes, 4);
% 
% %% Store edge number for quick access later when updating gamma
% edgeNum = zeros(nNodes, nNodes*2);
% for i = 1:size(edges,1)
%     x = edges(i,1);
%     y = edges(i,2);
%     edgeNum(x,y) = i;
% end
% 
% %% Main loop
% 
% for t = 1:T
%     % Initialize Gamma the arrival time at a vertex y through an edge e exactly at time t
%     gamma = zeros(size(edges,1), 3); % [x,y,gamma(x,y)]
%     for i = 1:size(edges,1)
%         x = edges(i,1);
%         y = edges(i,2);
%         gamma(i,:) = [x,y,Inf];
%     end
%     all_uD = find(arrivalTimes(:,4) == t);
%     for i = all_uD'
%         x = arrivalTimes(i,1);
%         y = arrivalTimes(i,2);
%         e = edgeNum(x,y); %find(ismember(edges, [x,y], 'rows'));
%         
%         uD = arrivalTimes(i,3);
%         gamma(e,3) = min(gamma(e,3), dm(x,uD+1) + getEdgeTime(x,y,uD,A,B,oprAvail));
% %         gamma(e,3) = min(gamma(e,3), max(uD, dm(x,uD+1)) + getEdgeTime(x,y,uD,A,B,oprAvail));
% %         if gamma(e,3) < Inf
% %             aa = 1;
% %         end
%     end
%     for y = 1:nNodes
%         gammaY1 = gamma(gamma(:,2)==y,3); % Gamma for autonomous edges
%         gammaY2 = gamma(gamma(:,2)==y+nNodes,3); % For teleoperated edges
%         gammaY = [gammaY1; gammaY2];
%         if ~isempty(gammaY)
%             dC(y,t+1) = min(gammaY);
%         end
%     end
%     for y = 1:nNodes
%         heaps{y}.push([t, dC(y,t+1)]);
%         if t > maxWaits(y)
%             heaps{y}(heaps{y}(:,1) == t-maxWaits(y)-1, :) = []; % Delete from the heap, all entries with time = t-maxW-1
%         end
%         heaps{y} = sortrows(heaps{y},2);
%     end
%     for y = 1:nNodes
%         uA = heaps{y}(1,1); % Time for minimum dC(y,t)
%         if dC(y,uA+1) < Inf
%             dm(y,t+1) = max(t,dC(y,uA+1)); % This max operation stores the time taken instead of cost of travel.
% %         dm(y,t+1) = d(y,uA+1); % This gives cost of travel and is same as the paper
%         end
%     end
%     if dm(goalVertex, t+1) < Inf
%         break;
%     end
% end
% for y = 1:nNodes
%     d_opt(y) = min(dC(y,:));
% end
% path = getPath(goalVertex, d_opt(goalVertex), dC, arrivalTimes, maxWaits, nNodes);
%     
% end
% 
% function time = getEdgeTime(x,y,startTime,A,B,oprAvail)
%     if y <= size(A,1) % Means it's the autonomous edge
%         time = A(x,y);
%     else % Means it's the teleoperated edge
%         y = y - size(A,1);
%         rank = find(oprAvail<=startTime,1, 'last'); %rank of arrivalTime in oprAvail
%         if isempty(rank)
%             rank = 0;
%         end
%         if mod(rank, 2) == 1 && (rank == size(oprAvail,2) || oprAvail(rank+1) > startTime + B(x,y)) % Is operator is available throughout the duration of the edge
%             time = B(x,y);
%         else
%             time = Inf;
%         end
%     end
% end
% 
% function path = getPath(goalVertex, goalArrivalTime, d, arrivalTimes, maxWaits, nNodes)
%     % Path is stored as [vertex, arrival time, wait, mode]
% %     path = [goalVertex, goalArrivalTime, 0, 0];
% % %     currentNode = find(ismember(arrivalTimes(:,[2,4]), [goalVertex, goalArrivalTime; goalVertex+nNodes, goalArrivalTime], 'rows'), 1);
% %     currentNode = find(ismember(arrivalTimes(:,[2,4]), [goalVertex, goalArrivalTime; goalVertex+nNodes, goalArrivalTime], 'rows'), 1);
% %     currentNode = arrivalTimes(currentNode,:);
%     path = [];
%     currentNode = [goalVertex, goalVertex, goalArrivalTime, goalArrivalTime];
%     b = true;
%     while b %currentNode(1) ~= 1 %~(currentNode(1) == 1 && currentNode(2) == 0)
%         predDeparture = currentNode(3);
%         if currentNode(1) == 1 && predDeparture <= maxWaits(currentNode(1))
%             b = false;
%             selectedPred = [1 1 0 0];
%         else
%         
%             possibleArrivals = predDeparture:-1:predDeparture-maxWaits(currentNode(1));
%             possibleVertices = [currentNode(1), currentNode(1)+nNodes];
%             [p,q] = meshgrid(possibleVertices, possibleArrivals);
%             possibleNodes = [p(:) q(:)];
%             
%             predecessorNodes = ismember(arrivalTimes(:,[2,4]), possibleNodes, 'rows');
%             predecessorNodes = arrivalTimes(predecessorNodes,:);
%             for i = 1:size(predecessorNodes,1)
%                 p = predecessorNodes(i,:);
%                 id = (p(3):-1:p(3)-maxWaits(p(1)))+1; % p(3) is the departure time, so this line gives us all possible arrival times at this predecessor node
%                 id = id(id>0);
%                 if min(d(p(1), id)) < Inf
%                     selectedPred = p;
%                     break
%                 end
%             end
%         end
%         currentWait = predDeparture - selectedPred(4);
%         if currentNode(2) <= nNodes
%             mode = 0;
%         else
%             mode = 1;
%         end
%         path = [currentNode(1), selectedPred(4), currentWait, mode; path];
%         currentNode = selectedPred;
%     end
% end
% 
% function heap = heap_push(heap, element)
%     heap = [heap; element];
%     heap = heap_heapify_up(heap, size(heap, 1));
% end
% 
% function heap = heap_pop(heap)
%     heap(1,:) = heap(end,:);
%     heap = heap(1:end-1,:);
%     heap = heap_heapify_down(heap, 1);
% end
% 
% function heap = heap_heapify_up(heap, i)
%     while i > 1 && heap(i,2) < heap(floor(i/2),2)
%         heap([i, floor(i/2)],:) = heap([floor(i/2), i],:);
%         i = floor(i/2);
%     end
% end
% 
% function heap = heap_heapify_down(heap, i)
%     while 2*i <= size(heap, 1)
%         j = 2*i;
%         if j < size(heap, 1) && heap(j,2) > heap(j+1,2)
%             j = j + 1;
%         end
%         if heap(i,2) <= heap(j,2)
%             break;
%         end
%         heap([i, j],:) = heap([j, i],:);
%         i = j;
%     end
% end