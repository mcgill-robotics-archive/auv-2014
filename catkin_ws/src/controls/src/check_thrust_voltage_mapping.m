thrustList = -30:0.001:30;
voltageList = [];

%{
Matt's stuff

for thrust = thrustList
    if (thrust<0.4641 & thrust>-0.01095 )
        voltage = 0;
    elseif (thrust>0.4641 & thrust<0.79423)
        voltage = (-0.1419 + sqrt(0.1419*0.1419-4*0.0676*(-0.3668-thrust)))/(2*0.0676);
    elseif (thrust>0.79423) 
        voltage=(thrust+3.2786)/1.047;
    elseif (thrust<-0.01095 & thrust>-1.179701)
        voltage = -0.0314 + sqrt(0.0314*0.0314-4*-0.0851*(0.0042-thrust));
    elseif (thrust<-1.179701)
        voltage = (thrust-2.5582)/0.9609;
    end
    voltageList = [voltageList voltage];
end

%}

%{
Redone by nick June 6 2014

%}



for thrust = thrustList
    if (thrust<=-1.05)
        voltage = 1.09*thrust - 3.1542;
    elseif (thrust>-1.05 & thrust<=0)
        voltage = -4.3968*(-1*thrust)^0.4534;
    elseif (thrust>0 & thrust<=1.25)
        voltage = 4.3968*thrust^0.4534;
    elseif (thrust > 1.25)
        voltage = thrust + 3.6857;
    end
    voltageList = [voltageList voltage];
end


plot(thrustList,voltageList,'.')