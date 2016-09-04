classdef Kalman
    properties
        A, B, C, R, Q
    end
    methods
        function obj = Kalman(A, B, C, R, Q)
        % This method creates a KalmanFilter object
        % x(t) = A*x(t-1) + B*u(t) + eps    [Cov(eps) = R]
        % z(t) = C*x(t) + del               [Cov(del) = Q]
        obj.A = A; obj.B = B; obj.C = C;
        obj.R = R; obj.Q = Q;
        end
        
        function [mu_pred, cov_pred] = predict(obj, mu, cov, u)
        % Applies the prediction step of Kalman filter
        mu_pred  = obj.A*mu + obj.B*u;
        cov_pred = obj.A*cov*obj.A' + obj.R;
        end
        
        function [mu_corr, cov_corr] = correct(obj, mu_pred, cov_pred, z)
        % Applies the correction/measurement update step
        K = cov_pred * obj.C' * pinv( obj.C * cov_pred * obj.C' + obj.Q);
        mu_corr = mu_pred + K*(z - obj.C*mu_pred);
        cov_corr = (eye(size(obj.A,2)) - K*obj.C)*cov_pred;
        end
        
        function [mu_corr, cov_corr] = filter(obj, mu_old, cov_old, u, z)
        % Convenience function, applies prediction
        % and correction steps one after another
        [mu_pred, cov_pred] = obj.predict(mu_old, cov_old, u);
        [mu_corr, cov_corr] = obj.correct(mu_pred, cov_pred, z);
        end
    end
end
