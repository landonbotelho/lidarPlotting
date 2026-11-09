% KITTI LiDAR Obstacle Detection
% Reads real Velodyne LiDAR data from the KITTI dataset.
% Filters the point cloud, removes approximate ground points,
% detects potential obstacles, and displays the results.

clear;
clc;
close all;

%% Step 1: Load a KITTI LiDAR scan

% Ask the user to select a KITTI .bin file
% Name of our uploaded KITTI LiDAR file
fileName = '0000000000.bin';

% Location of the file
folderPath = pwd;

% Open the selected file
filePath = fullfile(folderPath, fileName);
fileID = fopen(filePath, 'rb');

if fileID == -1
    error('Could not open the selected file.');
end

% KITTI stores each point as four 32-bit floating-point values:
% x, y, z, and reflectance
rawData = fread(fileID, 'single=>single');
fclose(fileID);

% Check that the file contains complete LiDAR points
if mod(numel(rawData), 4) ~= 0
    error('Invalid KITTI LiDAR file.');
end

% Convert the data into rows of [x, y, z, reflectance]
points = reshape(rawData, 4, [])';

x = points(:, 1);
y = points(:, 2);
z = points(:, 3);
reflectance = points(:, 4);

fprintf('Loaded %d LiDAR points.\n', size(points, 1));

%% Step 2: Display the original 3D point cloud

figure;

scatter3(x, y, z, 2, reflectance, 'filled');

xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Z (meters)');

title('Original KITTI LiDAR Point Cloud');

axis equal;
grid on;
colorbar;
view(3);

%% Step 3: Filter points outside our detection area

% Define the region we want to examine
minX = 0;
maxX = 50;

minY = -25;
maxY = 25;

minZ = -3;
maxZ = 2;

% Keep points inside the region
validPoints = ...
    x >= minX & x <= maxX & ...
    y >= minY & y <= maxY & ...
    z >= minZ & z <= maxZ;

filteredPoints = points(validPoints, :);

fprintf('Points after filtering: %d\n', ...
    size(filteredPoints, 1));

%% Step 4: Remove approximate ground points

% Simple height threshold for removing ground
% This value may need adjustment for different KITTI scans
groundThreshold = -1.5;

% Keep points above the threshold
obstaclePoints = filteredPoints( ...
    filteredPoints(:, 3) > groundThreshold, :);

fprintf('Points after ground removal: %d\n', ...
    size(obstaclePoints, 1));

%% Step 5: Create a 2D occupancy grid

% Each grid cell represents 0.25 meters
cellSize = 0.25;

numRows = ceil((maxY - minY) / cellSize);
numCols = ceil((maxX - minX) / cellSize);

% Initialize grid with zeros
occupancyGrid = zeros(numRows, numCols);

% Convert LiDAR coordinates into grid coordinates
for i = 1:size(obstaclePoints, 1)

    pointX = obstaclePoints(i, 1);
    pointY = obstaclePoints(i, 2);

    col = floor((pointX - minX) / cellSize) + 1;
    row = floor((pointY - minY) / cellSize) + 1;

    % Check that the coordinates are valid
    if row >= 1 && row <= numRows && ...
       col >= 1 && col <= numCols

        occupancyGrid(row, col) = 1;
    end
end

%% Step 6: Find connected obstacle regions

% Track which cells have already been visited
visited = false(numRows, numCols);

% Store detected obstacle clusters
clusters = {};

% Eight possible neighboring directions
directions = [
    -1 -1;
    -1  0;
    -1  1;
     0 -1;
     0  1;
     1 -1;
     1  0;
     1  1
];

% Minimum occupied cells required for a cluster
minClusterSize = 3;

for row = 1:numRows
    for col = 1:numCols

        % Skip empty or previously visited cells
        if occupancyGrid(row, col) == 0 || visited(row, col)
            continue;
        end

        % Start a new cluster using a queue
        queue = [row, col];
        visited(row, col) = true;

        clusterCells = [];

        % Breadth-first search
        while ~isempty(queue)

            current = queue(1, :);
            queue(1, :) = [];

            clusterCells(end + 1, :) = current;

            % Examine all eight neighboring cells
            for d = 1:size(directions, 1)

                nextRow = current(1) + directions(d, 1);
                nextCol = current(2) + directions(d, 2);

                % Check grid boundaries
                if nextRow < 1 || nextRow > numRows || ...
                   nextCol < 1 || nextCol > numCols
                    continue;
                end

                % Add occupied, unvisited neighbors
                if occupancyGrid(nextRow, nextCol) == 1 && ...
                   ~visited(nextRow, nextCol)

                    visited(nextRow, nextCol) = true;

                    queue(end + 1, :) = [nextRow, nextCol];
                end
            end
        end

        % Keep clusters containing enough cells
        if size(clusterCells, 1) >= minClusterSize
            clusters{end + 1} = clusterCells;
        end
    end
end

fprintf('Detected %d potential obstacle clusters.\n', ...
    length(clusters));

%% Step 7: Display detected obstacles

figure;
hold on;

% Display obstacle points in gray
scatter(obstaclePoints(:, 1), obstaclePoints(:, 2), ...
    3, [0.7 0.7 0.7], 'filled');

% Display each cluster with a different color
colors = lines(max(length(clusters), 1));

for i = 1:length(clusters)

    clusterCells = clusters{i};

    % Convert grid coordinates back to meters
    clusterX = minX + ...
        (clusterCells(:, 2) - 0.5) * cellSize;

    clusterY = minY + ...
        (clusterCells(:, 1) - 0.5) * cellSize;

    % Calculate approximate center
    centerX = mean(clusterX);
    centerY = mean(clusterY);

    % Draw cluster cells
    scatter(clusterX, clusterY, 15, ...
        colors(i, :), 'filled');

    % Mark the center
    plot(centerX, centerY, 'kx', ...
        'MarkerSize', 10, 'LineWidth', 2);

    % Label the cluster
    text(centerX, centerY, sprintf(' %d', i));
end

xlabel('X (meters)');
ylabel('Y (meters)');

title('KITTI LiDAR Potential Obstacle Detection');

axis equal;
xlim([minX maxX]);
ylim([minY maxY]);

grid on;
hold off;

%% Step 8: Display the occupancy grid

figure;

imagesc([minX maxX], [minY maxY], occupancyGrid);

set(gca, 'YDir', 'normal');

xlabel('X (meters)');
ylabel('Y (meters)');

title('LiDAR Occupancy Grid');

colormap(gray);
colorbar;
axis equal tight;
