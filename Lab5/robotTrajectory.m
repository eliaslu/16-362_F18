classdef robotTrajectory < handle
    properties
        dt = 0.0;
        
        numSamples= 0;
        duration = 0.0;
        x  = [];
        y  = [];
        th = [];
        V = [];
        w = [];
        T = [];
    end
    
    methods(Access = public)
       
        function obj = robotTrajectory(referenceControl,dt)
            obj.x(end+1) = 0;  
            obj.y(end+1) = 0; 
            obj.th(end+1) = 0;
            obj.V(end+1) = 0;
            obj.w(end+1) = 0;
            obj.dt = dt;
            obj.T(end+1) = 0.0;
            
            obj.duration  = referenceControl.getTrajectoryDuration();
            obj.numSamples = floor( obj.duration/obj.dt);
            
            for i = 1:obj.numSamples
                
                obj.T(end+1)= i*obj.dt;
                [V,w] = referenceControl.computeControl(obj.T(end));
                tmpV = obj.V(end);
                tmpw = obj.w(end);
                obj.V(end+1) = V;
                obj.w(end+1) = w;
                obj.th(end+1) = obj.th(end) + tmpw*obj.dt;
                obj.x(end+1)  = obj.x(end)  + tmpV*cos(obj.th(end))*obj.dt;
                obj.y(end+1)  = obj.y(end)  + tmpV*sin(obj.th(end))*obj.dt;
            end
            figure
            plot(obj.x,obj.y, 'b-');
            title('State Estimate');
            xlabel('y-direction (m)');
            ylabel('x-direction (m)');

        end
        
         function dt = getDt(obj)
             dt = obj.dt;
         end
         
         function duration = getDuration(obj)
             duration = obj.duration;
         end
         function V = getV(obj,t)
            
            if t > obj.duration
                V = 0;
            elseif t < 0
                V = 0;
            else
             [~,index] = min(abs(obj.T - t));
             V = obj.V(index);
            end
         end
         function W = getW(obj,t)
             
            if t > obj.duration
                W = 0;
            elseif t < 0
                W = 0;
            else
              [~,index] = min(abs(obj.T - t));
             W = obj.w(index);
            end
         end
         
         
        function pose = getPoseAtTime(obj,t)
            realT = t;
            if t >= obj.duration
                realT = obj.duration-0.2;
            elseif t < 0
                realT = 0;
            end
            curX = interp1(obj.T,obj.x,realT);
            curY = interp1(obj.T,obj.y,realT);
            curTh = interp1(obj.T,obj.th,realT);
            
                
            pose = Pose(curX,curY,curTh);
            
            
        end
    end
end
        
        