function path = TCSPCai1998(edges, goalVertex, A, B, oprAvail, maxWaits, T)

nNodes = size(A,1);
%% Initialize
d = Inf(nNodes, T+1); % Stores cost to a node (row) of at max time t (column), t = [0,T]
d(1,1) = 0; % Corresponds to d(s,0)
d_opt = Inf(nNodes,1);
heaps = cell(nNodes,1);
dm = zeros(nNodes,T+1);
for x = 1:nNodes
    heaps{x} = [0, d(x,1)]; % [t, d(x,t)] % This is for t = 0
    dm(x,1) = d(x,1);
end
arrivalTimes = zeros(T*size(edges,1), 4); % [edgeStart, edgeEnd, arrivalAtStart, arrivalAtEnd]
for i = 1:size(edges,1)
    x = edges(i,1);
    y = edges(i,2);
    for u = 0:T
        edgeTime = getEdgeTime(x,y,u,A,B,oprAvail);
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

for t = 0:T
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
        gamma(e,3) = min(gamma(e,3), dm(x,uD+1) + getEdgeTime(x,y,uD,A,B,oprAvail));
%         gamma(e,3) = min(gamma(e,3), max(uD, dm(x,uD+1)) + getEdgeTime(x,y,uD,A,B,oprAvail));
        if gamma(e,3) < Inf
            aa = 1;
        end
    end
    for y = 1:nNodes
        gammaY1 = gamma(gamma(:,2)==y,3);
        gammaY2 = gamma(gamma(:,2)==y+nNodes,3);
        gammaY = [gammaY1; gammaY2];
        if ~isempty(gammaY)
            d(y,t+1) = min(gammaY);
        end
    end
    for y = 1:nNodes
        heaps{y} = sortrows([heaps{y}; [t, d(y,t+1)]], 2);
        if t > maxWaits(y)
            heaps{y}(heaps{y}(:,1) == t-maxWaits(y)-1, :) = []; % Delete from the heap, all entries with time = t-maxW-1
        end
        heaps{y} = sortrows(heaps{y},2);
    end
    for y = 1:nNodes
        uA = heaps{y}(1,1); % Time for minimum d(y,t)
        dm(y,t+1) = max(t,d(y,uA+1)); % This max operation stores the time taken instead of cost of travel.
%         dm(y,t+1) = d(y,uA+1);
    end
    if dm(goalVertex, t+1) < Inf
        break;
    end
end
for y = 1:nNodes
    d_opt(y) = min(d(y,:));
end
path = getPath(goalVertex, d_opt(goalVertex), d, arrivalTimes, maxWaits, nNodes);
    
end

function time = getEdgeTime(x,y,startTime,A,B,oprAvail)
    if y <= size(A,1) % Means it's the autonomous edge
        time = A(x,y);
    else % Means it's the teleoperated edge
        y = y - size(A,1);
        rank = find(oprAvail<=startTime,1, 'last'); %rank of arrivalTime in oprAvail
        if isempty(rank)
            rank = 0;
        end
        if mod(rank, 2) == 1 && (rank == size(oprAvail,2) || oprAvail(rank+1) > startTime + B(x,y)) % Is operator is available throughout the duration of the edge
            time = B(x,y);
        else
            time = Inf;
        end
    end
end

function path = getPath(goalVertex, goalArrivalTime, d, arrivalTimes, maxWaits, nNodes)
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
        if currentNode(1) == 1 && predDeparture <= maxWaits(currentNode(1))
            b = false;
            selectedPred = [1 1 0 0];
        else
        
            possibleArrivals = predDeparture:-1:predDeparture-maxWaits(currentNode(1));
            possibleVertices = [currentNode(1), currentNode(1)+nNodes];
            [p,q] = meshgrid(possibleVertices, possibleArrivals);
            possibleNodes = [p(:) q(:)];
            
            predecessorNodes = ismember(arrivalTimes(:,[2,4]), possibleNodes, 'rows');
            predecessorNodes = arrivalTimes(predecessorNodes,:);
            for i = 1:size(predecessorNodes,1)
                p = predecessorNodes(i,:);
                id = (p(3):-1:p(3)-maxWaits(p(1)))+1;
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