%   快速路汇入区基本交通流仿真DEMO
%
%	涉及的交通仿真内容包括：
%	车辆生成（发车间隔/车辆初始化）、车辆删除、
%	跟驰模型、换道模型（动机生成/车道选择/间隙选择/换道执行）
%
%   有疑问请咨询：ye_yingjun@qq.com
%
%	Copyright (c) TOPS GROUP

%% 模块1：仿真参数设置
sim_reso=0.1;%仿真精度 (s)
sim_period=900; %仿真时长（s）
ramp_flow=[600*ones(1,sim_period)];% 流量输入: 小时流量*(1,持续时长)
exp_flow=[600*ones(1,sim_period)];% 单位：小时流量-veh/h/ln 持续时长-s
% ramp_flow=[600*ones(1,200),900*ones(1,200),1200*ones(1,200),800*ones(1,300)];% 流量输入: 小时流量*(1,持续时长)
% exp_flow=[800*ones(1,200),1000*ones(1,200),1000*ones(1,200),600*ones(1,300)];% 单位：小时流量-veh/h/ln 持续时长-s

%道路设置，目前暂时不支持修改
global expw_num ramp_num lane_width
expw_num=3; %高快速路主线车道数量
ramp_num=1; %匝道数量
lane_width=4;%车道宽度(m)
lane_speed_limit=[80,80,100,120]./3.6;%车道限速（从外到内） km/h

%车辆设置
global veh_type
veh_type=struct('veh_width',{},'veh_length',{},'veh_ratio',{}); %车辆类型: 1-car, 2-bus
veh_type(1).veh_width=1.8;%车辆宽度(m)
veh_type(1).veh_length=4;%车辆长度(m)
veh_type(1).veh_ratio=0.95;%车辆类型比例
veh_type(2).veh_width=2;%(m)
veh_type(2).veh_length=10;%(m)
veh_type(2).veh_ratio=0.05;

i=0; %仿真帧号
lane_num=expw_num+ramp_num;%车道总数
flow_input=[ramp_flow;exp_flow;exp_flow;exp_flow]; %主线不同车道的
hdw=zeros(lane_num,1);%headway车头时距
veh_num=zeros(lane_num,1);
frame_last_veh=zeros(lane_num,1);
global lanes %1-车辆编号, 2-位置, 3-速度, 4-加速度 5-车辆类型 %6-换道标志(0-不换道 1-向左 2-向右)
lanes=cell(lane_num,1);
expw_veh_id=1; %初始的车辆编号, 后续新增的车辆依次编号2,3,4,...
start_pos=150;
lc_record=[];


%% 开始仿真，对每一帧进行迭代更新
while(i*sim_reso<sim_period)
    lanes0=lanes;

    for k=1:4
        %% 模块2：车辆生成模块
        if (i==0 || (~isempty(lanes{k,1}) && veh_num(k)~=sum(lanes{k,1}(:,1)>0)))  %在仿真开始或者新生成车辆，需要计算下一辆车的时间间隔
            hdw(k)=Rand_Hdw(flow_input(k,ceil((i+1)*sim_reso)));
            frame_last_veh(k)=i;
        end
        if (~isempty(lanes{k,1}))
            veh_num(k)=sum(lanes{k,1}(:,1)>0);
        end

        if (i-frame_last_veh(k)<hdw(k)*10)
            i=i+1;
        else
            % 生成新的车辆
            if k<lane_num && rand()>veh_type(1).veh_ratio
                veh_type0=2; %bus
            else
                veh_type0=1; %car
            end
            if isempty(lanes{k,1})
                if k==1 %在匝道列表的末端添加一辆静止的车辆，这样根据跟驰模型未完成汇入的匝道车辆会停在匝道末端
                    lanes{k,1}=[lanes{k,1};[10000,405,0,0,1,0];[expw_veh_id,0,lane_speed_limit(k),0,veh_type0,0]]; 
                else
                    lanes{k,1}=[lanes{k,1};[expw_veh_id,200+100*rand(),lane_speed_limit(k),0,veh_type0,0]];
                end
            else %新生成的车辆加在列表最后
                if lanes{k,1}(end,2)-start_pos>2*veh_type(lanes{k,1}(end,5)).veh_length
                    lanes{k,1}=[lanes{k,1};[expw_veh_id,start_pos,lanes{k,1}(end,3),0,veh_type0,0]];
                else % 如果车辆拥堵蔓延到路段起点，让新生成的车辆在后方排队
                    lanes{k,1}=[lanes{k,1};[expw_veh_id,lanes{k,1}(end,2)-2*veh_type(lanes{k,1}(end,5)).veh_length,lanes{k,1}(end,3),0,veh_type0,0]];%lanes{k,1}(end,3)+rand()
                end
            end
            veh_traj{expw_veh_id,1}=[];
            expw_veh_id=expw_veh_id+1;
        end
        
       %% 模块3：车辆更新模块
        if (~isempty(lanes{k,1}))

            lanes{k,1}(1,2)=lanes{k,1}(1,2)+lanes{k,1}(1,3)*sim_reso;
            if (length(lanes{k,1}(:,1))>1)
                for j=2:length(lanes{k,1}(:,1))
                    % 跟驰模型: 这里用IDM(可以自行替换)
                    veh_type0 = lanes{k,1}(j,5);
                    veh_id = lanes{k,1}(j,1);
                    veh_pos = lanes{k,1}(j,2);
                    veh_speed = lanes{k,1}(j,3);
                    front_speed = lanes{k,1}(j-1,3);
                    front_gap = lanes{k,1}(j-1,2)-lanes{k,1}(j,2)-veh_type(lanes{k,1}(j-1,5)).veh_length;
                    new_veh_acc=IDM(veh_type0, veh_speed, front_speed, front_gap); %TODO：此处可自行对IDM函数做替换（比如GM/Gipps模型），返回加速度
                    if veh_speed+new_veh_acc*sim_reso<0
                        new_veh_acc=(0-veh_speed)/sim_reso;
                        new_veh_speed=0;  % 避免倒车
                    else
                        if veh_speed+new_veh_acc*sim_reso>lane_speed_limit(k)
                            new_veh_acc=(lane_speed_limit(k)-veh_speed)/sim_reso;
                            new_veh_speed=lane_speed_limit(k); % 车速限速
                        else
                            new_veh_speed=veh_speed+new_veh_acc*sim_reso;
                        end
                    end
                    new_veh_pos=max(veh_pos+veh_speed*sim_reso+0.5*new_veh_acc*sim_reso*sim_reso,veh_pos);
                    if new_veh_pos>450
                        lanes{k,1}(j,1)=0; % 如果车辆行驶到路段终点，将其编号标记为0，用于统一删除
                    end
                    lanes{k,1}(j,2)=new_veh_pos;
                    lanes{k,1}(j,3)=new_veh_speed;
                    lanes{k,1}(j,4)=new_veh_acc;
                    
                    % 换道模型 （Line126~185）
                    veh_pos = lanes{k,1}(j,2);
                    veh_speed = lanes{k,1}(j,3);
                    veh_lc_flag = lanes{k,1}(j,6);
                    front_lc_flag = lanes{k,1}(j-1,6);
                    if (veh_lc_flag==0 && veh_pos>50 && lanes{k,1}(j,1)~=0 && front_lc_flag==0)%isempty(veh_traj{lanes{k,1}(j-1,1),1})
                        surr_vehs=find_surr(k,j);%找相邻车道前后车，返回一个4*2的矩阵，其含义如下
                        %index1:1-左前; 2-左后; 3-右前; 4-右后
                        %index2:1-距离; 2-速度
                        lf_gap=surr_vehs(1,1); lf_speed=surr_vehs(1,2);
                        lb_gap=surr_vehs(2,1); lb_speed=surr_vehs(2,2);
                        rf_gap=surr_vehs(3,1); rf_speed=surr_vehs(3,2);
                        rb_gap=surr_vehs(4,1); rb_speed=surr_vehs(4,2);
                        
                        if k<=ramp_num %匝道MLC
                            %MLC_gap MLC的临界间隙 lf-左前; lb-左后; rf-右前; rb-右后
                            ramp_rest_length = lanes{1,1}(1,2)-lanes{k,1}(j,2); %本车距离匝道终点（强制换道点）的距离
                            MLC_gap_lf=1+0.15*2.237*max(0,veh_speed-lf_speed)+0.3*2.237*min(0,veh_speed-lf_speed)+0.2*2.237*veh_speed+0.1*2.237*(1-exp(-0.008*ramp_rest_length))+randn();
                            MLC_gap_rf=1+0.15*2.237*max(0,veh_speed-rf_speed)+0.3*2.237*min(0,veh_speed-rf_speed)+0.2*2.237*veh_speed+0.1*2.237*(1-exp(-0.008*ramp_rest_length))+randn();
                            MLC_gap_lb=1.5+0.1*2.237*max(0,lb_speed-veh_speed)+0.35*2.237*min(0,lb_speed-veh_speed)+0.25*2.237*lb_speed+0.1*2.237*(1-exp(-0.008*ramp_rest_length))+1.5*randn();
                            MLC_gap_rb=1.5+0.1*2.237*max(0,rb_speed-veh_speed)+0.35*2.237*min(0,rb_speed-veh_speed)+0.25*2.237*rb_speed+0.1*2.237*(1-exp(-0.008*ramp_rest_length))+1.5*randn();
                            if veh_pos>200 %表示位于加速车道上
                                if ((front_speed<lf_speed-10/3.6 || ramp_rest_length<100+50*rand()) ... %MLC-①换道动机（目标车道前车速度高于本车10km/h以上，且距离加速车道末端小于100m）
                                        && lf_gap>MLC_gap_lf && lb_gap>MLC_gap_lb) %MLC-③间隙选择
                                    lanes{k,1}(j,6)=1;
                                    veh_traj{veh_id,1}=LC_traj(k,j);%MLC-④换道执行
                                    lc_record=[lc_record;[veh_id,k,k+1]];
                                end
                            end
                        else %主线DLC
                            %DLC_gap DLC的临界间隙 lf-左前; lb-左后; rf-右前; rb-右后
                            DLC_gap_lf=1+0.2*2.237*max(0,veh_speed-lf_speed)+0.35*2.237*min(0,veh_speed-lf_speed)+0.25*2.237*veh_speed+randn();
                            DLC_gap_rf=1+0.2*2.237*max(0,veh_speed-rf_speed)+0.35*2.237*min(0,veh_speed-rf_speed)+0.25*2.237*veh_speed+randn();
                            DLC_gap_lb=1.5+0.15*2.237*max(0,lb_speed-veh_speed)+0.45*2.237*min(0,lb_speed-veh_speed)+0.30*2.237*lb_speed+1.5*randn();
                            DLC_gap_rb=1.5+0.15*2.237*max(0,rb_speed-veh_speed)+0.45*2.237*min(0,rb_speed-veh_speed)+0.30*2.237*rb_speed+1.5*randn();
                            
                            front_pos = lanes{k,1}(j-1,2);
                            front_type = lanes{k,1}(j-1,5);
                            if (front_type==2 && front_pos-veh_pos<2*veh_speed && front_speed<lf_speed...%DLC-①★换道动机1：如果前车是大型车辆,且左车道前车速度高于当前车道前车速度
                                    && lf_gap>DLC_gap_lf && lb_gap>DLC_gap_lb) %DLC-③★间隙选择：且左侧间隙大于阈值，就换道
                                lanes{k,1}(j,6)=1;
                                veh_traj{veh_id,1}=LC_traj(k,j); %DLC-④换道执行（生成换道轨迹）
                                lc_record=[lc_record;[veh_id,k,k+1]];
                            elseif (front_type==2 && front_pos-veh_pos<2*veh_speed && front_speed<rf_speed ...%DLC-②车道选择：高快速路先考虑向左侧换道，然后再考虑右侧
                                    && rf_gap>DLC_gap_rf && rb_gap>DLC_gap_rb)
                                lanes{k,1}(j,6)=2;
                                veh_traj{veh_id,1}=LC_traj(k,j);
                                lc_record=[lc_record;[veh_id,k,k-1]];
                            end
                            
                            if (lanes{k,1}(j,6)==0 && veh_speed<lane_speed_limit(k)-20/3.6 && front_speed<veh_speed*0.8 && front_speed<lf_speed-10/3.6 ...%DLC-①换道动机2：前车速低于期望速度20km/h以下，且前车速度比本车速度低20%，目标车道前车速度高于本车道前车10km/h以上
                                    && lf_gap>DLC_gap_lf && lb_gap>DLC_gap_lb) %DLC-③间隙选择：且左侧间隙大于阈值，就换道
                                lanes{k,1}(j,6)=1;
                                veh_traj{veh_id,1}=LC_traj(k,j); %DLC-④换道执行
                                lc_record=[lc_record;[veh_id,k,k+1]];
                            elseif (lanes{k,1}(j,6)==0 && veh_speed<lane_speed_limit(k)-20/3.6&& front_speed<veh_speed*0.8 &&front_speed<rf_speed-10/3.6 ...
                                    && rf_gap>DLC_gap_rf && rb_gap>DLC_gap_rb) %右侧同理
                                lanes{k,1}(j,6)=2;
                                veh_traj{veh_id,1}=LC_traj(k,j);
                                lc_record=[lc_record;[veh_id,k,k-1]];
                            end
                        end
                    end
                end
            end
            lanes{k,1}=lanes{k,1}(lanes{k,1}(:,1)>0,:); % % 删除编号为0车辆（当车辆驶离路网会被编为0 → Line116）
        end
       %% 模块4：结果输出
        % 仿真可视化：画车道线（加速车道长度200m）
        if (~isempty(lanes0{k,1}))
            line([50 200],[-lane_width*6 0],'Color','k');hold on;
            line([50 200],[-lane_width*6-lane_width/cos(tanh(lane_width/25)) -lane_width],'Color','k');hold on;
            line([200 400],[-lane_width -lane_width],'Color','k');hold on;
            line([400 410],[-lane_width 0],'Color','k');hold on;
            line([0 200],[0 0],'Color','k');hold on;
            line([410 600],[0 0],'Color','k');hold on;
            line([200 410],[0 0],'linestyle',':','Color','k');hold on;
            line([0 600],[lane_width lane_width],'linestyle',':','Color','k');hold on;
            line([0 600],[lane_width*2 lane_width*2],'linestyle',':','Color','k');hold on;
            line([0 600],[lane_width*3 lane_width*3],'Color','k');hold on;
            text(400, -60, ['simulation time: ',num2str(i/10), ' s']);
            
            % 画仿真车辆
            for j=length(lanes0{k,1}(:,1)):-1:1
                if lanes0{k,1}(j,6)==0
                    if (k==1 && lanes0{k,1}(j,2)>50 && lanes0{k,1}(j,2)<405)
                        if lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length>200  %加速车道车辆
                            h=rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2-lane_width,veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,0);
                        else %匝道车辆（倾斜）
                            h=rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,4/25*(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length)-32-(lane_width+veh_type(lanes0{k,1}(j,5)).veh_width)/2,veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,tanh(4/25));
                        end
                    elseif k==2 %主线车辆
                        h=rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2,veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,0);
                    elseif k==3
                        h=rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2+lane_width,veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,0);
                    elseif k==4
                        h=rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2+lane_width*2,veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,0);
                    end
                else
                    if ~isempty(veh_traj{lanes0{k,1}(j,1),1}) && lanes0{k,1}(j,2)<veh_traj{lanes0{k,1}(j,1),1}(end-2,1) %
                        for m=1:length(veh_traj{lanes0{k,1}(j,1),1})-5
                            if veh_traj{lanes0{k,1}(j,1),1}(m,1)<lanes0{k,1}(j,2)
                                continue
                            else
                                %在换道过程中，如果车身压到车道线，就认为它会同时对两个车道的后车产生影响
                                veh_heading=tanh((veh_traj{lanes0{k,1}(j,1),1}(m+5,2)-veh_traj{lanes0{k,1}(j,1),1}(m,2))/(veh_traj{lanes0{k,1}(j,1),1}(m+5,1)-veh_traj{lanes0{k,1}(j,1),1}(m,1)));
                                
                                if (lanes0{k,1}(j,6)==1 && ceil((veh_traj{lanes0{k,1}(j,1),1}(m,2)+veh_type(lanes0{k,1}(j,5)).veh_width+(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2)/lane_width)+1~=k)
                                    new_lane=ceil((veh_traj{lanes0{k,1}(j,1),1}(m,2)+veh_type(lanes0{k,1}(j,5)).veh_width+(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2)/lane_width)+1;
                                    if ~ismember(lanes0{k,1}(j,1),lanes{new_lane,1}(:,1))
                                        lanes{new_lane,1}(end+1,:)=lanes0{k,1}(j,:);
                                        lanes{new_lane,1}=sortrows(lanes{new_lane,1},-2);
                                        new_order=find(lanes0{new_lane,1}(:,1)==lanes0{k,1}(j,1));
                                    end
                                elseif (lanes0{k,1}(j,6)==2 && ceil((veh_traj{lanes0{k,1}(j,1),1}(m,2)-(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2)/lane_width)+1~=k)
                                    new_lane=ceil((veh_traj{lanes0{k,1}(j,1),1}(m,2)-(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2)/lane_width)+1;
                                    if ~ismember(lanes0{k,1}(j,1),lanes{new_lane,1}(:,1))
                                        lanes{new_lane,1}(end+1,:)=lanes0{k,1}(j,:);
                                        lanes{new_lane,1}=sortrows(lanes{new_lane,1},-2);
                                        new_order=find(lanes0{new_lane,1}(:,1)==lanes0{k,1}(j,1));
                                    end
                                end
                                %在换道末期，如果车身完全进入目标车道，就把这辆车从原车道挪到目标车道上去
                                if (lanes0{k,1}(j,6)==1 && ceil((veh_traj{lanes0{k,1}(j,1),1}(m,2)+(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2)/lane_width)+1~=k)
                                    if k==lc_record(lc_record(:,1)==lanes0{k,1}(j,1),2)
                                        lanes{k,1}(lanes{k,1}(:,1)==lanes0{k,1}(j,1),:)=[];
                                    end
                                elseif (lanes0{k,1}(j,6)==2 && ceil((veh_traj{lanes0{k,1}(j,1),1}(m,2)+veh_type(lanes0{k,1}(j,5)).veh_width-(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2)/lane_width)+1~=k)
                                    if k==lc_record(lc_record(:,1)==lanes0{k,1}(j,1),2)
                                        lanes{k,1}(lanes{k,1}(:,1)==lanes0{k,1}(j,1),:)=[];
                                    end
                                end
                                break
                            end
                        end

                        
                        if (ismember(lanes0{k,1}(j,1),lanes0{lc_record(lc_record(:,1)==lanes0{k,1}(j,1),3),1}(:,1)) && k==lc_record(lc_record(:,1)==lanes0{k,1}(j,1),2)) 
                            continue;
                        else
                            h=rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,veh_traj{lanes0{k,1}(j,1),1}(m,2),veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,veh_heading);
                        end

                    else
                        h=rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2+lane_width*(k-2),veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,0);
                        if k==lc_record(lc_record(:,1)==lanes0{k,1}(j,1),3)
                            lanes{k,1}(lanes{k,1}(:,1)==lanes0{k,1}(j,1),6)=0;
                            origin_order=find(lanes{lc_record(lc_record(:,1)==lanes0{k,1}(j,1),2),1}(:,1)==lanes0{k,1}(j,1));
                            if ~isempty(origin_order)
                                lanes{lc_record(lc_record(:,1)==lanes0{k,1}(j,1),2),1}(origin_order,:)=[];
                            end
                            veh_traj{lanes0{k,1}(j,1),1}=[];
                            lc_record(lc_record(:,1)==lanes0{k,1}(j,1),:)=[];
                        end
                    end
                    
                end

                axis([150 450 -100 100]);
            end
        end
    end

    drawnow %显示可视化图像
    clf %清除图像
end