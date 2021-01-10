function [ Pa ] = get_lc_prob(Xy, Xa, Vy, Va , LW)

%估计车辆换道概率
LW=LW;
if Xy>Xa%目标车辆为左侧车辆
    delta_x=Xy-Xa;
    Pa=min(1,max(0,(LW-delta_x)/LW*1));
elseif Xy<Xa%目标车辆为右侧车辆
    delta_x=Xa-Xy;
    Pa=min(1,max(0, (LW-delta_x)/LW*1));
else
    Pa=0;
end

end