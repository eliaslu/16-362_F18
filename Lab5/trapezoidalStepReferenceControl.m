classdef trapezoidalStepReferenceControl< referenceControl
    properties(Access = private)
        amax = 0.75;
        vmax = 0.25;
        sgn = 1;
        
        tDelay = 0.0;
    end
    
    methods(Access = public)
        function obj = trapezoidalStepReferenceControl(goal,tDelay,tPause)
            obj.tTotal=4.33;
            obj.tDelay = tDelay;
            obj.tTotal = obj.tTotal+tDelay+2*tPause;
            
            t = 0.0;
             while (t < obj.tTotal)
                 
%                  t = t+obj.tStep;
                
               pause(obj.tStep);
               %init the tic toc in the first loop
               if(obj.firstEnter == true)
                    startTic = tic();
                    t = toc(startTic);
                    obj.firstEnter = false;
               else
                   t = toc(startTic);
               end
               obj.timeSoFar(end+1) = t;
               if t<tPause
                   v=0;
               
               elseif t > obj.tTotal-tPause
                  v=0;
               else
                 
                  v = obj.trapezoidalVelocityProfile(t - tPause,obj.amax,obj.vmax,goal,obj.sgn);
               end
               
               obj.vlSoFar(end+1) = v;
               obj.vrSoFar(end+1) = v;
               
             end
             
        
        end
    end
    methods(Access = private)
        function uref = trapezoidalVelocityProfile(~,t,amax,vmax,dist,sgn)
            tf = dist/vmax + vmax/amax;
            tramp = vmax/(amax);
            if t < 0
                uref = 0;
            elseif t < tramp
                uref = amax*t;
            elseif t < tf-tramp
                uref = vmax;
            elseif t < tf
                uref = amax*(tf-t);
            else
                uref = 0;
            end

            uref = uref * sgn;

        end
    end
    
        


end
