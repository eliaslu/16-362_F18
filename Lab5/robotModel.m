classdef robotModel < handle
    properties(Constant,GetAccess='public')
        W=0.087;
        W2=0.0435;
    end
    methods(Static = true)
        function [V,w] = vlvrToVw(vl, vr)
            % Converts wheel speeds to body linear and angular velocity.
            V = (vr+vl)/2;
            w = (vr-vl)/robotModel.W;
        end
        
        function [vl,vr] = VwTovlvr(V, w)
            if V > 0
            % Converts body linear and angular velocity to wheel speeds.
            vr = V + robotModel.W2*w;
            vl = V - robotModel.W2*w;
            else
                vl = V + robotModel.W2*w;
                vr = V - robotModel.W2*w;
            end
        end
    end
end

        
        