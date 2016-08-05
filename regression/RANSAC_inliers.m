function inliers = RANSAC_inliers(points, d, n)
% RANSAC_inliers Returns a list of inliers in the data assuming linear model
% points is a matrix of the form [X y] where X is n*m, y is n*1
% d is the distance threshold
% n is the number of iterations

npts = size(points, 1);    % total number of points
k    = size(points, 2);    % number of points to sample to fit linear model
inliers = [];

for i = 1:n
    % select k points randomly (ensuring that they are distinct)
    idx = randsample(npts,k);   % select k distinct numbers in [1:npts]
    pt = points(idx,:);         % pt(1) = [x1, x2, ..., xn, y]
    xx = pt; xx(:,end) = 1;     % xx(1) = [x1, x2, ..., xn, 1]
    yy = pt(:,end);             % yy(1) = [y]
    dir = xx\yy;                % find hyper-plane through these points

    % find the set of inliers for this line
    dist = arrayfun(@(n) distance(dir, points(n,:)), 1:npts);    
    in = points(dist < d,:);
    if (size(in, 1) > size(inliers, 1))
        inliers = in;
    end
end
end

% -------------------------------------------------------------------------

function d = distance(hyp, point)
% distance Computes perpendicular distance between a hyperplane and a point
x = point; x(end) = 1;           % x = [x1, x2, ..., xn, 1]
y = point(end);                  % y = [y]
v = hyp; v(end) = 1;             % v = [w1, w2, ..., wn, 1]
d = abs(y - hyp' * x')/norm(v);
end

