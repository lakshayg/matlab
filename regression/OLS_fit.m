function [f, w] = OLS_fit(x, y)
% OLS_fit Returns a linear fit for data using ordinary least squares
% Returns a fit function f : X -> Y
X = [x, ones(size(y))];
w = pinv(X) * y;
f = @(x) w' * [x; ones(1,size(x,2))];
end
