function visualizeExploration(Q,Qexp, x,y, col, s)
% Extract data from the queue
entries = zeros(numel(Q), numel(Q{1}));
for i = 1:numel(Q)
    entries(i, :) = Q{i};
end
Qnew = [entries; Qexp];

nodeNumbers = Qnew(:,1);

% Extract the x and y coordinates of the vertices using the node numbers
x = x(nodeNumbers);
y = y(nodeNumbers);
z = Qnew(:,2);

% Create the scatter plot
scatter3(x, y, z, 'LineWidth', s, 'MarkerEdgeColor', col);

% Add labels and a title to the axes
xlabel('X coordinate');
ylabel('Y coordinate');
zlabel('Time');
title('Vertices explored by algorithm');

% Adjust the view angle and aspect ratio of the plot
view(45, 30);
axis tight;


end