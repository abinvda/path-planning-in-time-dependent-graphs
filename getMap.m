function [E, D, x, y] = getMap(nPoints, xMax, yMax, xMin, yMin, method, x, y)
if nargin < 8 % Means we need to generate x and y coordinates now, otherwise it's given
%     % Generate the x and y coordinates of the vertices
%     x = (xMax-xMin)*rand(nPoints, 1) + xMin;
%     y = (yMax-yMin)*rand(nPoints, 1) + yMin;
    
    % Square grid
    x = linspace(xMin, xMax, floor(sqrt(nPoints)));
    y = linspace(yMin, yMax, floor(sqrt(nPoints))); % + rand(1, nPoints);
    
    [x, y] = meshgrid(x,y);
    x = reshape(x, [numel(x), 1]) + 600*rand(numel(x), 1) - 300; % Shifting it randomly between -1km and + 1km
    y = reshape(y, [numel(y), 1]) + 600*rand(numel(y), 1) - 300;
%     x = reshape(x, [numel(x), 1]) + 40*rand(numel(x), 1) - 20;
%     y = reshape(y, [numel(y), 1]) + 40*rand(numel(y), 1) - 20;
end

% Initialize an empty adjacency matrix
E = zeros(nPoints);
D = zeros(nPoints);

if method == "del"
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

        % Save the corresponding distances
        D(v1, v2) = norm([x(v1) y(v1)] - [x(v2) y(v2)]);
        D(v2, v3) = norm([x(v2) y(v2)] - [x(v3) y(v3)]);
        D(v3, v1) = norm([x(v3) y(v3)] - [x(v1) y(v1)]);
    end
elseif method == "gab"
    for i=1:nPoints
        for j=i+1:nPoints
            % Compute the distance between points i and j
            d = sqrt((x(i)-x(j))^2 + (y(i)-y(j))^2);

            % Check if there are no other points within the circle with
            % radius d and center at the midpoint of i and j
            midpoint = [(x(i)+x(j))/2, (y(i)+y(j))/2];
            otherPoints = [x(1:nPoints ~= i & nPoints ~= j), y(1:nPoints ~= i & nPoints ~= j)];
            distances = sqrt((otherPoints(:,1) - midpoint(1)).^2 + (otherPoints(:,2) - midpoint(2)).^2);
            if all(distances > d/2)
                E(i,j) = 1;
                E(j,i) = 1;
            end
        end
    end
elseif method == "k"
    % Find the k-nearest neighbors for each point
    for i = 1:nPoints
        k = 2; % Connecting to k nearest neighbours
        % Get the distance between point i and all other points
        d = sqrt((x(i) - x).^2 + (y(i) - y).^2);

        % Sort the distances and get the indices of the k-nearest neighbors
        [~, idx] = sort(d);
        neighbors = idx(2:(k+1)); % The first neighbor is the point itself

        % Connect point i to its k-nearest neighbors
        E(i, neighbors) = 1;
        E(neighbors, i) = 1;
    end

    E = primMST(E, x,y);
end

    
end

function MST = primMST(E, x, y)
    % Initialize the MST with a starting vertex
    startVertex = 1;
    nVertices = size(E, 1);
    MST = E;
    visited = false(1, nVertices);
    visited(startVertex) = true;
    
    while any(~visited)
        % Find the minimum weight edge from a visited vertex to an unvisited vertex
        for i = 1:nVertices
            if visited(i)
                for j = 1:nVertices
                    if E(i,j) > 0
                        % Add the edge to the MST and mark the vertex as visited
                        MST(i, j) = 1;
                        MST(j, i) = 1;
                        visited(j) = true;
                    end
                end
            end
        end
        v = 1:nVertices;
        v = v(randperm(length(v)));
        for i = 1:nVertices
            if ~visited(i)
                for j = v
                    if visited(j)
                        % Add the edge to the MST and mark the vertex as visited
                        MST(i, j) = 1;
                        MST(j, i) = 1;
                        visited(i) = true;
                        break;
                    end
                end
                if visited(i)
                    break;
                end
            end
        end
    end
end

function MST = Kruskal(G)
        % G is the input graph represented as an adjacency matrix
        % MST is the output Minimum Spanning Tree represented as an adjacency matrix
        [m, n] = size(G);
        if m ~= n
            error("Input graph must be square matrix");
        end
        numVertices = n;
        % Create an empty MST with no edges
        MST = zeros(numVertices);
        % Find the edges of the graph and their weights
        edges = [];
        for i = 1:numVertices
            for j = i+1:numVertices
                if G(i,j) ~= 0
                    edges = [edges; i j G(i,j)];
                end
            end
        end
        % Sort edges by ascending weight
        edges = sortrows(edges, 3);
        % Create a vector to keep track of the connected components
        connected = 1:numVertices;
        for i = 1:size(edges,1)
            % Get the current edge and its endpoints
            edge = edges(i,:);
            u = edge(1);
            v = edge(2);
            % Get the connected components of the endpoints
            component1 = connected(u);
            component2 = connected(v);
            % If the endpoints belong to different connected components, add the edge to the MST
            if component1 ~= component2
                MST(u,v) = edge(3);
                MST(v,u) = edge(3);
                % Update the connected components vector
                connected(connected == component2) = component1;
            end
        end
end

%%
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