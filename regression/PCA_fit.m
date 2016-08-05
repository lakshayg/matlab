function [f, w] = PCA_fit(x, y)
% function R = PCA_fit(data)
%OLS_fit Returns a linear fit for data using the PCA decomposition
% last columns is taken as the dependent variable and all the other
% columns represent independent variables
X = [x y];
mu = mean(X);
x0 = mu(1:end-1); y0 = mu(end);

[vec, ~] = eig(cov(X)); % eigenvectors of covariance matrix
basis = vec(:,(end-size(x,2)+1):end); % basis of the hyperplane

A = [x0 1; basis(1:end-1,:)' zeros(size(basis,2),1)];
b = [y0 basis(end,:)]';
w = pinv(A)*b;
f = @(x) w' * [x; ones(1,size(x,2))];
end
