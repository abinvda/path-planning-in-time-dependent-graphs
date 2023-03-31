function drawmap(E, xMax, yMax, xMin, yMin, posX, posY, s, g, minD, maxD, path, titl)
% Check if plotting the path
if nargin < 10
    path = [];
end
%% Plot the raph
hold off
plot(posX(s), posY(s), '.', 'Color', 'b', 'MarkerSize', 20);
axis equal
% DT = delaunay(x, y);

r1 = maxD;
pos = [posX(s)-r1 posY(s)-r1 2*r1 2*r1];
rectangle('Position',pos,'Curvature',[1 1], 'FaceColor',[0.92 .92 1.0],'EdgeColor','b', 'LineWidth',1)
hold on

r1 = minD;
pos = [posX(s)-r1 posY(s)-r1 2*r1 2*r1];
rectangle('Position',pos,'Curvature',[1 1], 'FaceColor',[1.0 1.0 1.0],'EdgeColor','b', 'LineWidth',1)
% alpha(.95)

% Plot the path
hold on;
for i = 1:size(path,1)-1
    currV = path(i,1);
    nextV = path(i+1,1);
    currX = posX(currV);
    currY = posY(currV);
    % Plot waiting circles
    if path(i,3) > 0
        scatter(currX, currY, 40*path(i,3), 'MarkerEdgeColor',[.6 .6 .6], 'MarkerFaceColor',[.8 .8 .8]); % plot waiting as a grey circle
    end 
end
for i = 1:size(path,1)-1
    currV = path(i,1);
    nextV = path(i+1,1);
    currX = posX(currV);
    currY = posY(currV);
    nextX = posX(nextV);
    nextY = posY(nextV);
    if path(i,4) == 0
        modeColor = [40,40,40]/255; % red color for mode 0
        sty = '-';
    elseif path(i,4) == 1
        modeColor = [0.0 0.4 1.0]; % blue color for mode 1
        sty = ':';
    else
        disp("Unknown mode: " + path(i,4))
    end

%     % Plot waiting circles
%     if path(i,3) > 0
%         scatter(currX, currY, 100*path(i,3), 'MarkerEdgeColor',[.6 .6 .6], 'MarkerFaceColor',[.8 .8 .8]); % plot waiting as a grey circle
%     end

    % Plot line from previous vertex to current vertex
    plot([currX nextX], [currY nextY], 'Color', modeColor, 'LineWidth', 2.5, 'LineStyle', sty); 

    plot(currX, currY, '.', 'Color', 'r', 'MarkerSize', 15);
    
    % Plot arrows in the direction of the path
% quiver(prevX, prevY, currX-prevX, currY-prevY, 'color', modeColor, 'AutoScale','on','MaxHeadSize',0.5, 'LineWidth', 2);
%     annotation('arrow', [(prevX+currX)/2 ((prevX+currX))/2], [(prevY+currY)/2 (prevY+currY)/2], 'HeadLength',5, 'HeadWidth',5);
    
end

% triplot(DT, x, y, 'Color', 'k', 'LineWidth', 0.5);
gplot(E, [posX, posY], ':.k')

% Plot the points
plot(posX, posY, '.', 'Color', 'r', 'MarkerSize', 10);
plot(posX(s), posY(s), '.', 'Color', 'b', 'MarkerSize', 20);
plot(posX(g), posY(g), '.', 'Color', 'g', 'MarkerSize', 20);

% Set the axis limits
xlim([min(xMin, min(posX)) max(xMax, max(posX))]);
ylim([min(yMin, min(posY)) max(yMax, max(posY))]);

% Add a title and labels
title(titl);
% xlabel('X');
% ylabel('Y');
ax = gca;
ax.FontSize = 18;
set(gca,'YTickLabel',[]);
set(gca,'XTickLabel',[]);

hold off
end
