classdef controller < handle
    properties(Constant)       
        Tt =3;%21000;%to be changed
    end
    properties(Access=public)
       
    end
    
        
    methods(Static = true)
        function [uv, uw,error] = errorControl(poseRef,poseReal,V)
            kx = 1/controller.Tt;
            if abs(V)<0.001
                ky = 0;
            else
                ky = 2/((controller.Tt^2)*abs(V));
            end
             error = [0,0,0];
            kth = 2/controller.Tt;
            errorWorld = poseRef.getPose() - poseReal.getPose();
            errorRobot = poseReal.aToB()*errorWorld;
            eTh = errorRobot(3);
            eTh =  atan2(sin(eTh),cos(eTh));
            error(1) = errorRobot(1);
             error(2) = errorRobot(2);
              error(3) = eTh;
            
            uv = errorRobot(1)*kx;
            uw = errorRobot(2)*ky+eTh*kth;
            
        end
        
        
    end

end
