classdef ExtendedKalman
    properties
        % g = @(mu, u) ...   // process model
        % h = @(mu_bar) ...  // measurement model
        g, h
        R, Q  % noise in process and measurement models
        G, H  % linearized process and measurement models
              % these must be manually set by the user 
              % after every iteration of predict and correct
    end
    methods
        function obj = ExtendedKalman(g, h, R, Q)
        % This method creates a ExtendedKalmanFilter object
        % x(t) = g(x(t-1), u(t)) + eps       [cov(eps) = R]
        % z(t) = h(x(t)) + del               [cov(del) = Q]
        obj.g = g; obj.h = h;
        obj.R = R; obj.Q = Q;
        end
        
        function [mu_pred, cov_pred] = predict(obj, mu, cov, u)
        % Applies the prediction step of ExtendedKalman filter
        mu_pred  = obj.g(mu, u);
        cov_pred = obj.G * cov * obj.G' + obj.R;
        end
        
        function [mu_corr, cov_corr] = correct(obj, mu_pred, cov_pred, z)
        % Applies the correction/measurement update step
        K = cov_pred * obj.H' * pinv( obj.H * cov_pred * obj.H' + obj.Q);
        mu_corr = mu_pred + K*(z - obj.h(mu_pred));
        cov_corr = (eye(size(mu_pred,1)) - K*obj.H)*cov_pred;
        end
        
        function [mu_corr, cov_corr] = filter(obj, mu_old, cov_old, u, z)
        % Convenience function, applies prediction
        % and correction steps one after another
        [mu_pred, cov_pred] = obj.predict(mu_old, cov_old, u);
        [mu_corr, cov_corr] = obj.correct(mu_pred, cov_pred, z);
        end
    end
end
