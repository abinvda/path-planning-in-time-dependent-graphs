%% Functions related to binary heap implementation 

%% For testing
% heap = {[1 2 3 4 5 6 7], [2 3 4 5 6 7 8]};
% tic
% for i = 1:10
%     heap = binary_heap3(heap, randi(200, [1,7]));
% end
% toc
% heap;
% 
% tic
% min_element = get_min(heap);
% heap = pop_min(heap);
% toc
% % Calculate the x and y coordinates of each node
% [x, y] = plot_heap(heap);

% % Plot the tree
% plot(x, y, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
% 
% % Label each node with its value
% for i = 1:numel(heap)
%     text(x(i), y(i), num2str(heap{i}(2)), 'HorizontalAlignment', 'center');
% end
% 
% axis equal
% axis off

function heap = binary_heap(heap, entry)
%BINARY_HEAP Add an entry to a binary heap and maintain the heap property
%   heap: cell array of vectors
%   entry: vector to be added to the heap

% Add the entry to the end of the heap
heap{end+1} = entry;

% Maintain the heap property by repeatedly swapping the new entry with its parent
% if the parent is greater than the new entry
idx = numel(heap); % Index of the new entry
while idx > 1
    % Calculate the index of the parent node
    parent_idx = floor(idx / 2);
    
    % If the parent is greater than the new entry, swap them
    if heap{parent_idx}(2) > heap{idx}(2) || (heap{parent_idx}(2) == heap{idx}(2) && heap{parent_idx}(3) < heap{idx}(3))
        heap = swap(heap, parent_idx, idx);
        idx = parent_idx; % Update the index of the new entry
    else
        % The heap property has been restored, so we can exit the loop
        break;
    end
end

end


% 
% function heap = binary_heap2(heap, entry)
% %BINARY_HEAP Add an entry to a binary heap and maintain the heap property
% %   heap: cell array of vectors
% %   entry: vector to be added to the heap
% 
% % Add the entry to the end of the heap
% heap{end+1} = entry;
% 
% % Maintain the heap property by repeatedly swapping the new entry with its parent
% % if the parent is greater than the new entry
% idx = numel(heap); % Index of the new entry
% while idx > 1
%     % Calculate the index of the parent node
%     parent_idx = floor(idx / 2);
%     
%     % If the parent is greater than the new entry, swap them
%     if heap{parent_idx}(2) > heap{idx}(2)
%         heap = swap(heap, parent_idx, idx);
%         idx = parent_idx; % Update the index of the new entry
%     else
%         % The heap property has been restored, so we can exit the loop
%         break;
%     end
% end
% 
% end

function heap = swap(heap, i, j)
%SWAP Swap two elements in a heap
%   heap: cell array representing the heap
%   i, j: indices of the elements to be swapped

% Swap the elements in the heap
temp = heap{i};
heap{i} = heap{j};
heap{j} = temp;

end

function heap = pop_min(heap)
%POP_MIN Pop the element with the minimum second element from the heap
%   heap: cell array representing the heap

if isempty(heap)
    error('Heap is empty');
end

% Replace the root of the heap with the last element in the heap
heap{1} = heap{end};
heap(end) = [];

% Maintain the heap property by repeatedly swapping the root with the smaller of its children
idx = 1;
while true
    left_child_idx = 2 * idx;
    right_child_idx = 2 * idx + 1;
    
    if left_child_idx > numel(heap)
        % There are no more children, so we can exit the loop
        break;
    elseif right_child_idx > numel(heap)
        % There is only a left child, so compare the root to the left child
        if heap{idx}(2) > heap{left_child_idx}(2)
            heap = swap(heap, idx, left_child_idx);
        end
        break;
    else
        % There are both left and right children, so compare the root to the smaller of the two
        if heap{left_child_idx}(2) < heap{right_child_idx}(2)
            if heap{idx}(2) > heap{left_child_idx}(2)
                heap = swap(heap, idx, left_child_idx);
                idx = left_child_idx;
            else
                break;
            end
        else
            if heap{idx}(2) > heap{right_child_idx}(2)
                heap = swap(heap, idx, right_child_idx);
                idx = right_child_idx;
            else
                break;
            end
        end
    end
end

end

function min_element = get_min(heap)
%GET_MIN Get the element with the minimum second element from the heap
%   heap: cell array representing the heap

if isempty(heap)
    error('Heap is empty');
end

min_element = heap{1};

end

function [x, y] = plot_heap(heap)
%PLOT_HEAP Calculate the x and y coordinates of each node in the heap
%   heap: cell array representing the heap
%   x, y: x and y coordinates of the nodes

% Initialize the coordinates
x = nan(1, numel(heap));
y = nan(1, numel(heap));

% Calculate the coordinates of the root
x(1) = 0;
y(1) = 0;

% Calculate the coordinates of the rest of the nodes
for i = 2:numel(heap)
    parent_idx = floor(i / 2);
    x(i) = x(parent_idx) + (-1)^(i+1);
    y(i) = y(parent_idx) - 1;
end

end
