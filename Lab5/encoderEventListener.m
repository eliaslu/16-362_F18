function encoderEventListener(~,event)
    global encoderDataTimestamp;
    global encoderLeft;
    global encoderRight;

    global vlFeedback;
    global vrFeedback;

    global timeArrayRob;
    global leftArrayRob;
    global rightArrayRob;
    global vlSoFarRob;
    global vrSoFarRob;
    global xSoFarRob;
    global ySoFarRob;
    global thSoFarRob;
    global timeDiff;


    encoderLeft = double(event.Vector.X);
    encoderRight = double(event.Vector.Y);
    encoderDataTimestamp = double(event.Header.Stamp.Sec) + ...
        double(event.Header.Stamp.Nsec)/1000000000.0;

    if isempty(timeArrayRob)
        timeArrayRob(end+1) = encoderDataTimestamp;
        leftArrayRob(end+1) = encoderLeft;
        rightArrayRob(end+1) = encoderRight;
        vlSoFarRob = zeros(1);
        vrSoFarRob = zeros(1);
        xSoFarRob= zeros(1);
        ySoFarRob= zeros(1);
        thSoFarRob= zeros(1);
    end

    if timeArrayRob(length(timeArrayRob)) ~= encoderDataTimestamp
        %increase timeArray, leftArray, rightArray, and compute vl vr
        timeArrayRob(end+1) = encoderDataTimestamp;
        leftArrayRob(end+1) = encoderLeft;
        rightArrayRob(end+1) = encoderRight;
        timeDiff = (timeArrayRob(length(timeArrayRob)) - timeArrayRob(length(timeArrayRob)-1));
        timeDiff = double(timeDiff); %dt
        vlFeedback = (leftArrayRob(length(leftArrayRob)) - leftArrayRob(length(leftArrayRob)-1))./timeDiff;
        vrFeedback = (rightArrayRob(length(rightArrayRob)) - rightArrayRob(length(rightArrayRob)-1))./timeDiff;
        vlFeedback = double(vlFeedback);
        vrFeedback = double(vrFeedback);

        vlSoFarRob(end+1) = vlFeedback;
        vrSoFarRob(end+1) = vrFeedback;
        V = (vrFeedback + vlFeedback)/2;
    
        omega = (vrFeedback - vlFeedback)/robotModel.W;
    
        xSoFarRob(end+1)  = xSoFarRob(end)  + V*cos(thSoFarRob(end))*timeDiff;
        ySoFarRob(end+1)  = ySoFarRob(end)  + V*sin(thSoFarRob(end))*timeDiff;
   
        thSoFarRob(end+1) = thSoFarRob(end) + omega*timeDiff;

    end

    pause(0.005);

end