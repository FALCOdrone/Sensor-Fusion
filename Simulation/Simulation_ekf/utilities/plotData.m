% First import .mat file

% Plot a graph for each variable in the struct
% Get the field names of the struct
fieldNames = fieldnames(data);

% Iterate over each field
for i = 1:numel(fieldNames)
    % Get the matrix for the current field
    matrix = data.(fieldNames{i});
    
    % Get the number of parameters
    numParams = size(matrix, 1) - 1;
    
    % Get the time vector
    time = matrix(end, :);
    % Compute the cumulative sum of the spacing vector to get the time vector
    time = cumsum(time/1000);
    
    % Plot a graph for each parameter
    figure;
    hold on;
    for j = 1:numParams
        plot(time, matrix(j, :));
    end
    hold off;
    
    % Set the title and labels
    title(fieldNames{i});
    xlabel('Time');
    ylabel('Value');
    
    % Add a legend
    legend('X', 'Y', 'Z'); % Add more labels as needed
end