

classdef trajectoryFollower <handle
    properties
        
        
        
        
    end
    
    methods(Access = public)
        function obj = trajectoryFollower(robotTrajectory)
            PID_ON =true;
            xSoFarRob = zeros(1);
            ySoFarRob = zeros(1);
            thSoFarRob = zeros(1);
            vlSoFarRob = zeros(1);
            vrSoFarRob = zeros(1);
            vSoFarRob = zeros(1);
             wSoFarRob = zeros(1);
            timeSoFarRob = zeros(1);
            leftSoFarRob = zeros(1);
            rightSoFarRob = zeros(1);
            curPoseX = zeros(1);
            curPoseY = zeros(1);
            curPoseTh = zeros(1);
            refPoseX = zeros(1);
            refPoseY = zeros(1);
            refPoseTh = zeros(1);
            errorX=zeros(1);
            errorY=zeros(1);
            errorTh=zeros(1);
            robot = raspbot('raspbot-6');
%             robot = raspbot('sim');
            pause(1);
%             robot.encoders.NewMessageFcn=@encoderEventListener;
%             % initialize encoder values
            left_initial = robot.encoders.LatestMessage.Vector.X;
            right_initial = robot.encoders.LatestMessage.Vector.Y;
            dt = robotTrajectory.getDt();
            duration = robotTrajectory.getDuration();
            t = 0.0;
            firstEnter = true;
            
            
             



            
            while t<duration+0.5
                pause(dt);
                if firstEnter
                    startTime = tic();
                    t = toc(startTime);
%                     l = length(xSoFarRob);
                    firstEnter = false;
                else
                    t = toc(startTime);
%                     l = length(xSoFarRob);
                end
                
                dt = t - timeSoFarRob(end);
                timeSoFarRob(end+1) = t;
                
                left_cur = ((robot.encoders.LatestMessage.Vector.X-left_initial) - leftSoFarRob(end))./dt;
                right_cur = ((robot.encoders.LatestMessage.Vector.Y-right_initial)- rightSoFarRob(end))./dt;
                vlSoFarRob(end+1) = left_cur;
                vrSoFarRob(end+1) = right_cur;
                leftSoFarRob(end+1) = robot.encoders.LatestMessage.Vector.X - left_initial;
                rightSoFarRob(end+1) = robot.encoders.LatestMessage.Vector.Y - right_initial;
                v_cur = (left_cur+right_cur)/2;
                w_cur = (right_cur-left_cur)/robotModel.W;
                thSoFarRob(end+1) = thSoFarRob(end) + w_cur*dt;
                vSoFarRob(end+1) = v_cur;
                wSoFarRob(end+1) = w_cur;
                xSoFarRob(end+1)  = xSoFarRob(end)  + vSoFarRob(end-1)*cos(thSoFarRob(end-1))*dt;
                ySoFarRob(end+1)  = ySoFarRob(end)  + vSoFarRob(end-1)*sin(thSoFarRob(end-1))*dt;
   

                curRealPose = Pose( xSoFarRob(end), ySoFarRob(end), thSoFarRob(end));
                curRefPose = robotTrajectory.getPoseAtTime(t);
%                 curRealPose.x()
%                  curRealPose.y()
%                   curRealPose.th()
%                   curRefPose.x()
%                   curRefPose.y()
%                   curRefPose.th()
%                 
                [uv,uw,error] = controller.errorControl(curRefPose,curRealPose,v_cur);
                
                if PID_ON
                    V = robotTrajectory.getV(t)+uv;
                    w = robotTrajectory.getW(t)+uw;
                else
                    V = robotTrajectory.getV(t);
                    w = robotTrajectory.getW(t);
                end
                
                [vl,vr] = robotModel.VwTovlvr(V,w);
                robot.sendVelocity(vl,vr);
                
                %graph
                curPoseX(end+1) =curRealPose.x();
                curPoseY(end+1) = curRealPose.y();
                curPoseTh(end+1) = curRealPose.th();
                refPoseX(end+1) = curRefPose.x();
                refPoseY(end+1) = curRefPose.y();
                refPoseTh(end+1) = curRefPose.th();
                 errorX(end+1)=error(1);
                 errorY(end+1)=error(2);
                errorTh(end+1)=error(3);
                
%                 set(f1, 'XData', timeSoFarRob);
%                 set(f1, 'YData', curPoseX);
%                 set(f2, 'XData', timeSoFarRob);
%                 set(f2, 'YData', curPoseY);
%                 set(f3,'YData', curPoseTh);
%                 set(f3,'XData', timeSoFarRob);
%                 set(f4, 'XData', timeSoFarRob);
%                 set(f4, 'YData', refPoseX);
%                 set(f5, 'XData', timeSoFarRob);
%                 set(f5, 'YData', refPoseY);
%                 set(f6,'YData', refPoseTh);
%                 set(f6,'XData', timeSoFarRob);
%                 
%                 set(f7, 'XData', curPoseX);
%                 set(f7, 'YData', curPoseY);
%                 set(f8,'YData', refPoseY);
%                 set(f8,'XData', refPoseX);
%                 
%                 set(f9, 'XData', timeSoFarRob);
%                 set(f9, 'YData', errorX);
%                 set(f10, 'XData', timeSoFarRob);
%                 set(f10, 'YData', errorY);
%                 set(f11,'YData', errorTh);
%                 set(f11,'XData', timeSoFarRob);
                
                
                
            
            end
            robot.sendVelocity(0,0);
            figure;
            subplot(1,3,1);
            f1 = plot(curPoseX,timeSoFarRob);
            hold on 
            f2 = plot(curPoseY,timeSoFarRob);
            f3 = plot(curPoseTh,timeSoFarRob);
            f4 = plot(refPoseX,timeSoFarRob);
            f5 = plot(refPoseY,timeSoFarRob);
            f6 = plot(refPoseTh,timeSoFarRob);
            hold off
            legend('curPoseX', 'curPoseY','curPoseTh','refPoseX','refPoseY','refPoseTh');
            xlabel('time(s)');
            ylabel('distance(m)');
            title('ref and real pose over time');
             subplot(1,3,2);
             f7 = plot(curPoseX(1:end-1),curPoseY(1:end-1));
             hold on
             f8 = plot(refPoseX(1:end-1),refPoseY(1:end-1));
             hold off
             legend('curPose', 'refPose');
            xlabel('x(m)');
            ylabel('y(m)');
            title('ref and real location over time');
            subplot(1,3,3);
            f9 = plot(errorX,timeSoFarRob);
             hold on
             f10 = plot(errorY,timeSoFarRob);
             f11 = plot(errorTh,timeSoFarRob);
             hold off
             legend('xError', 'yError','thError');
             xlabel('time(s)');
            ylabel('error(m)');
            title('error over time');
            
            robot.shutdown();
            
        end
    end
    
        
end
