function visualizePath(oprAvail, maxTime, pathOur, pathFast)

% Assume that time_instances is a vector of time instances where the availability switches
% and availability is a vector of 0's and 1's representing the availability at each time instance
hold off

% Initialize availability with 0 at t=0
tMax = maxTime; %max(maxTime, oprAvail(end));
t = 0;
ind = 1;
availability = Inf(1, tMax+1);
currAvail = 0;

% Iterate through switch_times and update availability
while t < tMax
    if ind > numel(oprAvail)
        availability(t+1:end) = 0;
        break;
    else
        if currAvail == 0
            availability(t+1:oprAvail(ind)) = 0;
            t = oprAvail(ind);
            ind = ind + 1;
            currAvail = 1;
        else
            availability(t+1:oprAvail(ind)) = 1;
            t = oprAvail(ind);
            ind = ind + 1;
            currAvail = 0;
        end
    end
end

% Plot availability
% stairs(0:numel(availability)-1, 0.3*availability, 'LineWidth', 3);
startT = 0;
for i = 1:numel(oprAvail)
    endT = oprAvail(i);
    if mod(i,2) == 1
        col = [0.9 0.8 0.8];
    else 
        col = [0.8 0.9 0.8];
    end
    rectangle('Position', [startT, 0, endT-startT, 0.4], 'FaceColor', col, 'LineStyle','none')
    startT = endT;    
end
hold on
if maxTime > oprAvail(end)
    rectangle('Position', [startT, 0, 1.1*maxTime-startT, 0.4], 'FaceColor', [0.9 0.8 0.8], 'LineStyle','none')
end
% ylim([-0.41, 0.4])
xlabel('Time (min)');
xlim([0, maxTime*1.1])
% ylabel('Availability');

plotPath(pathOur, 0.25)
plotPath(pathFast, 0.05)
% pos = get(gcf, 'Position');
% set(gcf, 'Position',pos+[0 00 0 -200])
set(gca,'YTickLabel',[]);
yticks([0.1 0.3])
yticklabels({'Greedy','Proposed'})
ax = gca;
ax.YAxis.FontSize = 14;
ax.XAxis.FontSize = 12;
end
function plotPath(path, pos)
    % Iterate through the path
    for i = 1:size(path,1)-1
        % Get the current vertex, arrival time, wait and mode
        arrivalTime = path(i,2);
        wait = path(i,3);
        mode = path(i,4);
        if mode == 0
            col = [0.2 0.2 0.2]; %[40,40,40]/255;
        else
            col = [0.0 0.4 1.0];
        end
        colE = [0.9 0.9 0.9];
        
        recHeight = 0.1;
        % Plot the waiting segments
        if wait > 0
            rectangle('Position', [arrivalTime, pos, wait, recHeight], 'FaceColor', [0.6,0.6,0.6], 'EdgeColor',colE, 'LineWidth',0.25)% 'LineStyle','none')
        end
    
        % Plot the edge segments
        departureTime = arrivalTime+wait;
        rectangle('Position', [departureTime, pos, path(i+1,2)-departureTime, recHeight], 'FaceColor', col, 'EdgeColor',colE, 'LineWidth',0.25)% 'LineStyle','none')% 'EdgeColor',colE)
    end
end
