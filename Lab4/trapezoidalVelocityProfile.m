function uref = trapezoidalVelocityProfile(t,amax,vmax,dist,sgn)
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
