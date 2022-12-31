%% This the main code of your simulation
% 1) Define the Graph: Edges that robot can travel on
% 2) Specify travel time under autonomous and assisted operation
% 3) Specify max allowed waiting for each edge (I think we should use matrices for all these)
% 4) Operator availability function using a list of times when availability changes 
clear;


nRobots = 1;
% A = magic(4);
% A = A>10; % Matrix with binary values. 0 = No edge, 1 = Edge present
% A = A - diag(diag(A)); % Set diagonal to 0, meaning no self loop

nNodes = 10; % Nodes in the graph are <1,2,...,nNodes>
E = rand(nNodes)>0.2; % Will be used to create A and B
E = E - diag(diag(E)); % Diagonal elements are 0 because self-loops are not allowed.
for j = 1:nNodes
    for i = 1:j
        if abs(i-j) > 4
            E(i,j) = 0;
        else
            E(i,j) = E(j,i);
        end
    end
end
E = triu(E); % Trying upper diagonal matrix to prevent cycles.
% E(:,end) = 0;
B = 1*randi([2,20], nNodes, nNodes) .* E;
A = B + 1*(randi([10,30], nNodes, nNodes) .* E);
Diff = A - B;

% E = [0 1 1 0 0 0;
%      0 0 0 1 0 0;
%      0 1 0 1 1 0;
%      0 0 0 0 1 1;
%      0 0 0 1 0 0;
%      0 0 0 0 0 0];
% B = 1*[0 4 9 0 0 0;
%      0 0 0 6 0 0;
%      0 99 0 2 3 0;
%      0 0 0 0 2 3;
%      0 0 0 2 0 0;
%      0 0 0 0 0 0];
% A = 1*[0 8 14 0 0 0;
%      0 0 0 12 0 0;
%      0 99 0 3 9 0;
%      0 0 0 0 3 99;
%      0 0 0 3 0 0;
%      0 0 0 0 0 0];


[x,y] = find(E);
edges = [x y];
edgesCai = [x y; x y+nNodes];
% edges = cell(nNodes,1);%[x y];
% for i = 1:nNodes
%     y = find(E(i,:));
%     edges(i,1) = {y};
% end
% edges = [1,2; 1,3; 2,4; 3,2; 3,4; 3,5; 4,5; 4,6; 5,4];
% teleTimes = [4, 2, 8, 2, 12, 2, 2, 3, 2]; %randi(10, [size(edges,1), 1]);
% autoTimes = [5, 5, 10, 3, 13, 3, 3, 7, 3]; %teleTimes + randi(5, [size(edges,1), 1]);
% 
% %% Preparing travel times matrices. A = autonomous times, B = teleop times
% x = edges(:,1);
% y = edges(:,2);
% idx = sub2ind(size(E), x, y);
% A = 1000*ones(nNodes);
% B = A;
% 
% A(idx) = autoTimes;
% B(idx) = teleTimes;

%% max waiting times at each node
Dif = max(Diff');
maxWaits = 2*ones(nNodes,1); %randi(50, [nNodes, 1]); %11*ones(nNodes,1);
maxTime = 1000; % To limit the search while testing. Can be removed once I fix the issue with oprAvail

%% Start and Goal vertices
startVertex = 1;
goalVertex = nNodes;

%% Operator availability
% oprAvail = [2, 10, 15, 30, 35, 1000]; % Times when opr availability changes. Default start is not available, if first element is 0 mean we start with available.
oprAvail = [0 5 29 1000];%sort(randi(600, 1, 30));

%% Robot path
% A robot's path is a kx4 matrix. Specified as a list of nodes to traverse, arrival times, wait times at those nodes and mode of operation. 

%% TODO: A function to compute path
tic
path = staticDijkstras(edges, startVertex, goalVertex, B)
pathAuto = staticDijkstras(edges, startVertex, goalVertex, A)
toc
tic
path = TCSPCai1998(edgesCai, goalVertex, A, B, oprAvail, maxWaits, pathAuto(end,2))
toc
tic
path = getRobotPath(edges, startVertex, goalVertex, A, B, oprAvail, maxWaits, pathAuto(end,2))
toc
tic
path = getRobotPath2(edges, startVertex, goalVertex, A, B, oprAvail, maxWaits, pathAuto(end,2))
toc
%% Simulate the system and get finish times of all robots for all tasks
% completionTimes = getCompletionTimes(nRobots, A, B, startState, path, oprAvail);
