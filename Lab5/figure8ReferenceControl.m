classdef figure8ReferenceControl <  referenceControl
    properties(Access = private)
        v = 0.2;
        sf = 1;
        tPause = 0.0;
        tf = 0.0;
        kt = 0.0;
        kk = 15.1084;
        Tf = 0.0;
    end
    
    methods(Access = public)
        function obj = figure8ReferenceControl(Ks,Kv,tPause)
            % Construct a figure 8 trajectory. It will not start until
            % tPause has elapsed and it will stay at zero for tPause
            % afterwards. Kv scales velocity up when > 1 and Ks scales
            % the size of the curve itself up.
            obj.tf = obj.sf/obj.v;
            obj.kt = 2*pi/obj.sf;
            obj.tPause = tPause;
            kv = Kv;
            ks = Ks;
            obj.tf = (ks/kv)*obj.tf;
            
            t = 0.0;
            obj.tTotal = obj.tf+2*obj.tPause;
            T = 0.0;
           while (T < obj.tTotal)

               pause(obj.tStep);
               %init the tic toc in the first loop
               if(obj.firstEnter == true)
                    startTic = tic();
                    T = toc(startTic);
                    obj.firstEnter = false;
               else
                   T = toc(startTic);
               end
%                
                obj.timeSoFar(end+1) = T;
               if T<obj.tPause
                   vr = 0;
                   vl = 0;
               
               elseif T > obj.tTotal-obj.tPause
                   vr = 0;
                   vl = 0;
                   
               else
               
                   t = (T-obj.tPause)*kv/ks;
                   s = obj.v *t;
                   curvature = obj.kk/ks * sin(obj.kt*s);
                   omega = curvature * obj.v;
                   vr = obj.v + robotModel.W * omega/2;
                   vl = obj.v - robotModel.W * omega/2;
               end
               
               obj.vlSoFar(end+1) = vl;
               obj.vrSoFar(end+1) = vr;
           end
           
           
            
            

        end
        
        
        
        
    end
    
    
    
end
