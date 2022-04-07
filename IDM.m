function [acceleration] = IDM(veh_type1, veh_speed, front_speed, front_gap)
%输入：本车速度、前车速度、前车间距
%输出：本车加速度

%IDM参数- 包含了2列，分别对应于2种车辆类型
net_dist=[0.8,1]; %停车间距（m）
reac_time=[1.2,1.8]; %反应时间（s）
desire_speed=[30,25]; %期望速度（m/s）
max_acc=[4.0,3.5]; %最大加速度（m/s2）
comfort_dec=[2.0,1.5]; %舒适减速度（m/s2）

%IDM公式
desire_dist=net_dist(veh_type1)+2*sqrt(veh_speed/desire_speed(veh_type1))+...
    reac_time(veh_type1)*veh_speed+veh_speed*(veh_speed-front_speed)/2/sqrt(max_acc(veh_type1)*comfort_dec(veh_type1));
acceleration=max_acc(veh_type1)*(1-power(veh_speed/desire_speed(veh_type1),4)-power(desire_dist/front_gap,2));
                    
end