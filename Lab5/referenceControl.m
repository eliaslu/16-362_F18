classdef referenceControl

properties
    vlSoFar = zeros(1);
    vrSoFar = zeros(1);
    timeSoFar = zeros(1);
    firstEnter = true;
    tTotal = 0.0;
    tStep = 0.001;
end

    methods
        function [V,w] = computeControl(obj,timeNow)
            % Return the linear and angular velocity that the robot 
            % should be executing at time timeNow. Any zero velocity
            % pauses specified in the constructor are implemented here
            % too.
            [ ~, index ] = min( abs( obj.timeSoFar-timeNow ) );
%             index = floor(timeNow/obj.tStep);
            len = length(obj.vlSoFar);
            if index > len || index < 1
                V = 0;
                w = 0;
            else

                vl = obj.vlSoFar(index);
                vr = obj.vrSoFar(index);
                [V,w]= robotModel.vlvrToVw(vl, vr);
            end


        end
        function duration = getTrajectoryDuration(obj)
            % Return the total time required for motion and for the
            % initial and terminal pauses.
            duration = obj.tTotal;
        end
    end

    



end
