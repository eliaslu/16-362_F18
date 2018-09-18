function encoderEventListener(~,event)
    global encoderDataTimestamp;
    global encoderLeft;
    global encoderRight;

    global vlFeedback;
    global vrFeedback;

    global timeArray;
    global leftArray;
    global rightArray;
    global vlSoFar;
    global vrSoFar;

    global timeDiff;


    encoderLeft = double(event.Vector.X);
    encoderRight = double(event.Vector.Y);
    encoderDataTimestamp = double(event.Header.Stamp.Sec) + ...
        double(event.Header.Stamp.Nsec)/1000000000.0;

    if isempty(timeArray)
        timeArray(end+1) = encoderDataTimestamp;
        leftArray(end+1) = encoderLeft;
        rightArray(end+1) = encoderRight;
        vlSoFar = zeros(1);
        vrSoFar = zeros(1);
    end

    if timeArray(length(timeArray)) ~= encoderDataTimestamp
        %increase timeArray, leftArray, rightArray, and compute vl vr
        timeArray(end+1) = encoderDataTimestamp;
        leftArray(end+1) = encoderLeft;
        rightArray(end+1) = encoderRight;
        timeDiff = (timeArray(length(timeArray)) - timeArray(length(timeArray)-1));
        timeDiff = double(timeDiff); %dt
        vlFeedback = (leftArray(length(leftArray)) - leftArray(length(leftArray)-1))./timeDiff;
        vrFeedback = (rightArray(length(rightArray)) - rightArray(length(rightArray)-1))./timeDiff;
        vlFeedback = double(vlFeedback);
        vrFeedback = double(vrFeedback);

        vlSoFar(end+1) = vlFeedback;
        vrSoFar(end+1) = vrFeedback;
    end

    pause(0.005);

end