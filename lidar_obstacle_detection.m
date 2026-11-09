%% MATLAB LiDAR Obstacle Detection and Mapping
% Self-contained demonstration using simulated 2D LiDAR measurements.
% Requires base MATLAB only (no additional toolboxes).
%
% Run this script in MATLAB. Local functions are defined at the end.
% Tested design target: MATLAB R2016b or newer (scripts with local functions).

clear;
clc;
close all;

rng(7); % Make the simulated scan repeatable.

%% 1. Define the environment
% The LiDAR is located at the origin (0,0), facing the positive X direction.
room = [-6, 6, -5, 5]; % [xmin, xmax, ymin, ymax]

% Each obstacle is a circle: [centerX, centerY, radius].
obstacles = [
    -2.0,  1.4, 0.75;
     1.8,  2.0, 0.90;
     2.6, -1.8, 0.65;
    -2.8, -2.0, 0.85
];

sensorPosition = [0, 0];
maxRange = 10.0;

%% 2. Simulate LiDAR measurements
% A 0.5-degree angular resolution creates 720 measurements per scan.
anglesDeg = 0:0.5:359.5;
anglesRad = deg2rad(anglesDeg);
measuredRanges = zeros(size(anglesRad));

for i = 1:length(anglesRad)
    direction = [cos(anglesRad(i)), sin(anglesRad(i))];

    % Find the closest intersection with a room wall or circular obstacle.
    measuredRanges(i) = simulateRay( ...
        sensorPosition, direction, room, obstacles, maxRange);
end

% Add realistic measurement noise.
noiseStdDev = 0.025; % Standard deviation in meters.
noisyRanges = measuredRanges + noiseStdDev .* randn(size(measuredRanges));

% Simulate occasional invalid readings.
invalidIndices = rand(size(noisyRanges)) < 0.025;
noisyRanges(invalidIndices) = NaN;

% Values outside the sensor's usable range are also invalid.
noisyRanges(noisyRanges <= 0 | noisyRanges > maxRange) = NaN;

%% 3. Filter the measurements
% Keep only finite, positive measurements within the sensor range.
validMask = isfinite(noisyRanges) & ...
            noisyRanges > 0 & noisyRanges <= maxRange;

filteredRanges = noisyRanges;
filteredRanges(~validMask) = NaN;

%% 4. Convert polar measurements into Cartesian coordinates
% x = range * cos(angle)
% y = range * sin(angle)
xPoints = filteredRanges .* cos(anglesRad);
yPoints = filteredRanges .* sin(anglesRad);

validPoints = [xPoints(validMask)', yPoints(validMask)'];

%% 5. Group nearby scan points into obstacle clusters
% The room boundary is farther away than the simulated obstacles.
% Excluding long-range returns prevents the room walls from being labeled
% as obstacles in this simple demonstration.
obstacleMaxRange = 4.8;
obstacleMask = validMask & filteredRanges < obstacleMaxRange;

% This simple distance-based method avoids requiring a clustering toolbox.
% A new cluster starts when:
%   - a measurement is invalid or beyond obstacleMaxRange, or
%   - two consecutive points are farther apart than clusterGap.

clusterGap = 0.35;       % Maximum point-to-point gap, in meters.
minimumClusterSize = 4;  % Ignore clusters with fewer points than this.

clusters = {};
currentCluster = [];

for i = 1:length(anglesRad)
    if ~obstacleMask(i)
        % Invalid or long-range reading: finish the current cluster, if one exists.
        if size(currentCluster, 1) >= minimumClusterSize
            clusters{end + 1} = currentCluster; %#ok<SAGROW>
        end
        currentCluster = [];
        continue;
    end

    point = [xPoints(i), yPoints(i)];

    if isempty(currentCluster)
        currentCluster = point;
    else
        previousPoint = currentCluster(end, :);
        pointGap = norm(point - previousPoint);

        if pointGap <= clusterGap
            currentCluster = [currentCluster; point]; %#ok<AGROW>
        else
            if size(currentCluster, 1) >= minimumClusterSize
                clusters{end + 1} = currentCluster; %#ok<SAGROW>
            end
            currentCluster = point;
        end
    end
end

% Save the final cluster after the loop.
if size(currentCluster, 1) >= minimumClusterSize
    clusters{end + 1} = currentCluster;
end

%% 6. Calculate a center and size estimate for each detected cluster
clusterCenters = zeros(length(clusters), 2);
clusterSizes = zeros(length(clusters), 1);

for i = 1:length(clusters)
    points = clusters{i};

    % The mean position is a simple estimate of the cluster center.
    clusterCenters(i, :) = mean(points, 1);

    % Estimate cluster size using the maximum distance from its center.
    distancesFromCenter = sqrt(sum( ...
        (points - clusterCenters(i, :)).^2, 2));
    clusterSizes(i) = 2 * max(distancesFromCenter);
end

%% 7. Display results
fprintf('MATLAB LiDAR Obstacle Detection Results\n');
fprintf('---------------------------------------\n');
fprintf('Total LiDAR measurements: %d\n', length(anglesRad));
fprintf('Valid measurements: %d\n', sum(validMask));
fprintf('Invalid measurements: %d\n', sum(~validMask));
fprintf('Detected point clusters: %d\n\n', length(clusters));

for i = 1:length(clusters)
    fprintf('Cluster %d: center = (%.2f, %.2f) m, approximate width = %.2f m\n', ...
        i, clusterCenters(i, 1), clusterCenters(i, 2), clusterSizes(i));
end

%% 8. Plot the LiDAR scan and detected clusters
figure('Name', 'LiDAR Obstacle Detection', 'Color', 'w');

hold on;
axis equal;
grid on;

% Draw room boundaries.
plot([room(1), room(2), room(2), room(1), room(1)], ...
     [room(3), room(3), room(4), room(4), room(3)], ...
     'k-', 'LineWidth', 1.5);

% Draw all valid LiDAR points.
scatter(xPoints(validMask), yPoints(validMask), ...
    8, [0.35, 0.55, 0.85], 'filled');

% Draw each detected cluster and its estimated center.
for i = 1:length(clusters)
    points = clusters{i};

    scatter(points(:, 1), points(:, 2), 18, 'g', 'filled');

    plot(clusterCenters(i, 1), clusterCenters(i, 2), ...
        'rx', 'MarkerSize', 12, 'LineWidth', 2);

    text(clusterCenters(i, 1) + 0.12, ...
         clusterCenters(i, 2) + 0.12, ...
         sprintf('Obstacle %d', i), 'Color', 'r', 'FontWeight', 'bold');
end

% Mark the LiDAR sensor.
plot(sensorPosition(1), sensorPosition(2), ...
    'kp', 'MarkerSize', 13, 'MarkerFaceColor', 'y');

xlabel('X position (meters)');
ylabel('Y position (meters)');
title('2D LiDAR Scan with Detected Point Clusters');
legend('Room boundary', 'Valid LiDAR points', ...
       'Detected cluster points', 'Estimated obstacle center', ...
       'LiDAR sensor', 'Location', 'bestoutside');

xlim([room(1) - 0.5, room(2) + 0.5]);
ylim([room(3) - 0.5, room(4) + 0.5]);
hold off;

%% 9. Plot measured range versus angle
figure('Name', 'LiDAR Range Measurements', 'Color', 'w');

plot(anglesDeg, noisyRanges, '.', 'Color', [0.7, 0.7, 0.7]);
hold on;
plot(anglesDeg, filteredRanges, 'b-', 'LineWidth', 1);
grid on;

xlabel('Scan angle (degrees)');
ylabel('Measured distance (meters)');
title('LiDAR Range Measurements Before and After Filtering');
legend('Raw noisy measurements', 'Filtered measurements');
xlim([0, 360]);
hold off;

%% Local functions

function range = simulateRay(origin, direction, room, obstacles, maxRange)
    % Return the nearest positive ray intersection with the room or an
    % obstacle. If no intersection is found, return maxRange.

    range = maxRange;

    % Intersections with the four room walls.
    if direction(1) > 0
        t = (room(2) - origin(1)) / direction(1);
        if t > 0
            range = min(range, t);
        end
    elseif direction(1) < 0
        t = (room(1) - origin(1)) / direction(1);
        if t > 0
            range = min(range, t);
        end
    end

    if direction(2) > 0
        t = (room(4) - origin(2)) / direction(2);
        if t > 0
            range = min(range, t);
        end
    elseif direction(2) < 0
        t = (room(3) - origin(2)) / direction(2);
        if t > 0
            range = min(range, t);
        end
    end

    % Intersections with circular obstacles.
    for j = 1:size(obstacles, 1)
        center = obstacles(j, 1:2);
        radius = obstacles(j, 3);

        offset = origin - center;
        b = dot(direction, offset);
        c = dot(offset, offset) - radius^2;
        discriminant = b^2 - c;

        if discriminant >= 0
            tNear = -b - sqrt(discriminant);
            tFar = -b + sqrt(discriminant);

            if tNear > 0
                range = min(range, tNear);
            elseif tFar > 0
                range = min(range, tFar);
            end
        end
    end
end
