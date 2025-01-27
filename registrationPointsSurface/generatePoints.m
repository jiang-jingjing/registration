% Function to generate and project 3D points onto a head surface
function [projectedPoints, originalPoints] = generateAndProjectPoints(headModel, numPointsPerCircle, radius1, radius2)

    % Generate points on two circles as in Pioneer position sensors
    theta = linspace(0, 2*pi, numPointsPerCircle+1);
    theta(end) = []; % Remove duplicate point at 2*pi
    z0= 160;
    x0 = 80;
    y0 = 80;
    centerC = [x0 y0 z0];
    circle1 = [radius1*cos(theta) + x0; radius1*sin(theta) + y0; zeros(1, numPointsPerCircle)+z0];
    circle2 = [radius2*cos(theta) + x0; radius2*sin(theta) + y0; zeros(1, numPointsPerCircle)+z0];

    % Shift the second circle up
%     circle2(3,:) = 5;

    originalPoints = [circle1'; circle2'] ;

    % Project points onto the head surface
%     projectedPoints = zeros(size(originalPoints));
% 
%     distances = vecnorm(headModel - centerC,2,2);
%     [~, minIndex] = min(distances);
%     closestPoint = headModel(minIndex,:);

 projectedPoints = zeros(size(originalPoints));
    for i = 1:size(originalPoints, 1)
        point = originalPoints(i,:);
        % Find closest point on the head surface for EACH point
        distances = vecnorm(headModel - point,2,2);
        [~, minIndex] = min(distances);
        projectedPoints(i,:) = headModel(minIndex,:);
    end
end

 