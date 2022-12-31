function robotFinishTime = getCompletionTimes(nRobots, A, B, robotState, path, oprAvail)

% Since this is a determinstic system, we can compute completion times for all robots easily
robotFinishTime = zeros(nRobots,1);
for r = 1:nRobots
    for i = 1:size(path,1)-1
        startNode = path(i,1);
        endNode = path(i+1,1);
        waitTime = path(i,2);
        oprMode = path(i,3);
        if oprMode == 0
            edgeTime = A(startNode, endNode);
        else
            edgeTime = B(startNode, endNode);
        end
        robotFinishTime(r) = robotFinishTime(r) + waitTime + edgeTime;
    end
end