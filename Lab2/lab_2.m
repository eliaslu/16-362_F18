%%
robot = raspbot("sim");
pause(1);
robot.startLaser();
pause(3);
tmpx=0;
tmpy=0;
tmpth=0;
while true
%     a = robot.laser.LatestMessage.Intensities;
    x=[];
    y=[];
    th=[];
    a = robot.laser.LatestMessage.Ranges;
    for index = 1:size(a)
        if a(index)<=0.06
            a(index)=0;
        end
        if a(index)<=1
            [tmpx,tmpy,tmpth]= irToXy(index,a(index));
            if abs(tmpth)<=  deg2rad(90)
                [x(end+1),y(end+1),th(end+1)] = irToXy(index,a(index));
            end
        end
    end
    
    f = scatter(x,y,"x");
    xlim([-2 2]);
    ylim([-2 2]);
    
%     plot(a);
    pause(0.2);
    
end
