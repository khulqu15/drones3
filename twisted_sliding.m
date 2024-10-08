classdef twisted_sliding
    properties
        lambda
        eta
        k
        u_max
    end

    methods
        function obj = twisted_sliding(lambda, eta, k, u_max)
            obj.lambda = lambda;
            obj.eta = eta;
            obj.k = k;
            obj.u_max = u_max;
        end

        function [u, s, ds] = control(obj, x, x_ref, dx, dx_ref)
            s = (x - x_ref) + obj.lambda * (dx - dx_ref);
            ds = (dx - dx_ref) + obj.lambda * (x - x_ref);

            u_eq = -obj.lambda * dx;
            u_tsmc = obj.k * sign(ds) - obj.eta * sign(s);
            u = u_eq + u_tsmc;
            u = min(max(u, -obj.u_max), obj.u_max);
        end
    end
end