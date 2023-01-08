function [E, x, y] = getMap(nPoints, xMax, yMax)
% Generate the x and y coordinates of the vertices
x = sort(xMax*rand(nPoints, 1));
y = yMax*rand(nPoints, 1);

% % Compute the Voronoi diagram
% [V, C] = voronoi(x, y);

% Compute the Delaunay triangulation
DT = delaunay(x, y);

% Create an empty adjacency matrix
E = zeros(nPoints);

% Iterate over the rows of the Delaunay triangulation
for i = 1:size(DT, 1)
    % Get the indices of the vertices of the current triangle
    v1 = DT(i, 1);
    v2 = DT(i, 2);
    v3 = DT(i, 3);
    
    % Set the corresponding elements in the adjacency matrix to 1
    E(v1, v2) = 1;
    E(v2, v3) = 1;
    E(v3, v1) = 1;
end

% % Plot the Delaunay triangulation
% figure;
% triplot(DT, x, y, 'Color', 'k', 'LineWidth', 0.5);
% hold on;
% 
% % Plot the points
% plot(x, y, '.', 'Color', 'r', 'MarkerSize', 10);
% 
% % Set the axis limits
% xlim([0 xMax]);
% ylim([0 yMax]);
% 
% % Add a title and labels
% title('City Road Network');
% xlabel('X');
% ylabel('Y');
end

% function E = getMap(nPoints, xRange, yRange)
% 
% E = zeros(nPoints);
% xMin = xRange(1);
% xMax = xRange(2);
% yMin = yRange(1);
% yMax = yRange(2);
% 
% % Section 3.3 of https://people.eecs.berkeley.edu/~pabbeel/cs287-fa19/optreadings/rrtstar.pdf
% d = 2; % Dimension of our space
% mu = (xMax - xMin) * (yMax - yMin); % Volume of the obstacle-free space. Lebesgue measure
% psi = pi; % Volume of unit ball. In out case, of unit circle.
% gamma = 2*(1 + 1/d)^(1/d) * (mu/psi)^(1/d);
% factor = 0.7;
% 
% r = factor*gamma*(log(nPoints)/nPoints)^(1/d);
% 
% xSamples = xMin + (xMax-xMin)*rand(nPoints,1);
% ySamples = yMin + (yMax-yMin)*rand(nPoints,1);
% 
% samples = [xSamples, ySamples];
% % samples = sortrows(samples, [1,2]); % Sort according to x then y coordinate
% 
% edges = [];
% 
% for i = 1:size(samples,1)
%     for j = 1:size(samples,1)
%         if norm(samples(i,:) - samples(j,:)) < r
%             E(i,j) = 1;
%             edges(end+1, :) = [i,j];
%         end
%     end
% end
% 
% %% Plotting
% plot(xSamples, ySamples, 'o', 'MarkerFaceColor', 'b');
% for i = 1:size(edges, 1)
%     % Retrieve the indices of the starting and ending points
%     p1 = edges(i, 1);
%     p2 = edges(i, 2);
%     
%     % Retrieve the coordinates of the points
%     x1 = xSamples(p1);
%     y1 = ySamples(p1);
%     x2 = xSamples(p2);
%     y2 = ySamples(p2);
%     
%     % Draw the edge between the points
%     line([x1 x2], [y1 y2], 'Color', 'k')
% end
% 
% %%
% % Generate the x and y coordinates of the vertices
% x = 10*rand(1, 30);
% y = 10*rand(1, 30);
% 
% % Compute the Voronoi diagram
% [V, C] = voronoi(x, y);
% 
% % Compute the Delaunay triangulation
% DT = delaunay(x, y);
% 
% % Plot the Delaunay triangulation
% figure;
% triplot(DT, x, y, 'Color', 'k', 'LineWidth', 0.5);
% hold on;
% 
% % Plot the points
% plot(x, y, '.', 'Color', 'r', 'MarkerSize', 10);
% 
% % Set the axis limits
% xlim([0 10]);
% ylim([0 10]);
% 
% % Add a title and labels
% title('City Road Network');
% xlabel('X');
% ylabel('Y');
% 
% end