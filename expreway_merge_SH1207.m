%users's setting
datano=zeros(4,10,1000);
datano_count=1;
datayes=zeros(4,10,1000);
datayes_count=1;
plot_yes=1;
not_yield=1;
%simulation parameter
sim_reso=0.12;%simulation resolution (s)
ramp_flow=[1500*ones(1,100),1500*ones(1,300),1500*ones(1,300),1800*ones(1,300),820*ones(1,300),300*ones(1,300),...
    600*ones(1,200),800*ones(1,1000),400*ones(1,600),200*ones(1,400)];% flow input: volume*(1,duration)
exp_flow=[2300*ones(1,300),2600*ones(1,200),2600*ones(1,600),2800*ones(1,700),1000*ones(1,1000),700*ones(1,600),400*ones(1,400)];% veh/h/ln
sim_period=1000; %unit:s
%road setting
global expw_num
expw_num=3; %numbers of lanes on expressway
global ramp_num
ramp_num=1; %numbers of lanes on ramp, only 1 ramp lane is available so far
global lane_width
lane_width=3.5;%unit:m
lane_speed_limit=[80,80,80,80]./3.6;
start_pos=100;

%vehicle setting
global veh_type
veh_type=struct('veh_width',{},'veh_length',{},'veh_ratio',{}); %two types: 1-car, 2-bus
veh_type(1).veh_width=1.8;%(m)
veh_type(1).veh_length=4;%(m)
veh_type(1).veh_ratio=1.0;
veh_type(2).veh_width=2;%(m)
veh_type(2).veh_length=10;%(m)
veh_type(2).veh_ratio=0.0;
%model parameters- dimension corresponds to different vehicle types
net_dist=[0.5,1.5];
reac_time=[1.0,1.5];
desire_speed=[75/3.6,60/3.6];
max_acc=[4.0,3.0];
comfort_dec=[3.0,2.0];

%data strcutures setting
i=0; %frame id
lane_num=expw_num+ramp_num;%amount of lanes
flow_input=[ramp_flow;exp_flow;exp_flow;exp_flow*0.86]; %flow rate of the lanes above, sort from the outer(ramp) to the inner(mainline)
hdw=zeros(lane_num,1);%headway
veh_num=zeros(lane_num,1);
frame_last_veh=zeros(lane_num,1);
global lanes
%1-vehicle id, 2-position, 3-speed, 4-accelearation 5-vehicle type 
%6-lane change flag 7-color type 8-posx 9-speedx 10-heading 11-jerk
lanes=cell(lane_num,1); 
expw_veh_id=1;%initial vehicle id, new vehicle will be marked with 2,3,4,...
lc_record=[];
Tn_record=zeros(10000,1);
traj_dict = containers.Map;
lc_last=containers.Map;
ramp_endpos=200+145;
mergein=cell(3000,1);  %1-X, 2-Y, 3-Vx, 4-Vy, 5-Xf, 6-Yf, 7-Vyf, 8-Xl, 9-Yl, 10-Vyl
ramp_vehid=[];
time_eva=[];
c_center=-1.5;
border_x=0;
drt_org=1.26;
drt_red=0.65;
while(i*sim_reso<sim_period)
    i=i+1;
    
    lanes0=lanes;
    
    for k=1:4
        if ~isempty(lanes0{k,1}) lanes0{k,1}=lanes0{k,1}(lanes0{k,1}(:,1)>0,:);  end % delete vehicles that leaves the network
        % When the simulation starts or new vehicle is added, a new time headway is calculated
        if (i==0 || (~isempty(lanes{k,1}) && veh_num(k)))
            hdw(k)=Rand_Hdw(flow_input(k,ceil((i+1)*sim_reso)));
            veh_num(k)=0;
            frame_last_veh(k)=i;
        end
                
        if (i-frame_last_veh(k)>=hdw(k)*10)
            %% Add new vehicle
            if k<lane_num && rand()>veh_type(1).veh_ratio
                veh_type0=2; %bus
            else
                veh_type0=1; %car
            end
            if k==ramp_num
                flow_direction0=2;
                ramp_vehid=[ramp_vehid;expw_veh_id];
            else
                flow_direction0=1;
            end
            if isempty(lanes{k,1})
                if k==1 %a virtual vehicle is put at the end of the ramp, to make the other vehicles to stop before the end of ramp
                    lanes{k,1}=[lanes{k,1};[10000,ramp_endpos+5,0,0,1,0,1,-1.75,0,0,0];[expw_veh_id,0,lane_speed_limit(k),0,veh_type0,0,flow_direction0,3.5*k-5.25,0,0,0]]; 
                else
                    lanes{k,1}=[lanes{k,1};[expw_veh_id,3*3600/flow_input(k,ceil((i+1)*sim_reso))*rand(),lane_speed_limit(k),0,veh_type0,0,flow_direction0,3.5*k-5.25,0,0,0]];
                end
            else %new vehicles are put at the end of the vehicle array
                if lanes{k,1}(end,2)>2*veh_type(lanes{k,1}(end,5)).veh_length+start_pos
                    lanes{k,1}=[lanes{k,1};[expw_veh_id,start_pos,lanes{k,1}(end,3),0,veh_type0,0,flow_direction0,3.5*k-5.25,0,0,0]];
                else % if the traffic congested to the start point of flow input, following vehicles have to wait in line
                    lanes{k,1}=[lanes{k,1};[expw_veh_id,lanes{k,1}(end,2)-2*veh_type(lanes{k,1}(end,5)).veh_length,lanes{k,1}(end,3),0,veh_type0,0,flow_direction0,3.5*k-5.25,0,0,0]];%lanes{k,1}(end,3)+rand()
                end
            end
            veh_traj{expw_veh_id,1}=[];
            expw_veh_id=expw_veh_id+1;
            veh_num(k)=1;
        end
        
        %% update vehicle trajectory
        if (~isempty(lanes{k,1}))
            lanes{k,1}(1,2)=lanes{k,1}(1,2)+lanes{k,1}(1,3)*sim_reso;
            if (length(lanes{k,1}(:,1))>1)
                for j=2:length(lanes{k,1}(:,1))
                    % Car-following model: IDM(intelligent driver model)
                    veh_type1=lanes{k,1}(j,5);
                    surr_vehs=find_surr(k,j);%find surrounding vehicles
                    %index1:1-left lead; 2-left lag; 3-right lead; 4-right lag
                    %index2:1-distance; 2-velocity;3-id
                    right_front_id=surr_vehs(3,3);
                    right_lane=lanes{ramp_num,1};
                    rf_index=find(right_lane(:,1)==right_front_id);
                    rf_lc_flag=right_lane(rf_index,6);
                    %如果是路肩车道车辆，而且右前方的匝道车辆正要换道
                    if k==ramp_num+1 & surr_vehs(3,3)~=0 & lanes{k,1}(j,2)>150 & surr_vehs(3,3)~=10000 & rf_lc_flag~=0
                        %获得右前方车辆状态
                        right_front_id=surr_vehs(3,3);
                        rf_data=traj_dict(num2str(right_front_id));
                        rf_v=rf_data(end,3);
                        rf_y=rf_data(end,2);
                        rf_x=rf_data(end,8);
                        f_v=lanes{k,1}(j-1,3);
                        f_y=lanes{k,1}(j-1,2);
                        ego_data=traj_dict(num2str(lanes{k,1}(j,1)));
                        ego_v=ego_data(end-putin(1),3);
                        ego_y=ego_data(end-putin(1),2);
                        ego_x=ego_data(end-putin(1),8);
                        %IDM模型
                        Prf=b_get_lc_prob(ego_x,rf_x,ego_v,rf_v,lane_width); %车辆按照一定概率跟随右前车/本车道前车
                        desire_fdist=net_dist(veh_type1)+2*sqrt(lanes{k,1}(j,3)/desire_speed(veh_type1))+reac_time(veh_type1)*lanes{k,1}(j,3)+lanes{k,1}(j,3)*(lanes{k,1}(j,3)-lanes{k,1}(j-1,3))/2/sqrt(max_acc(veh_type1)*comfort_dec(veh_type1));
                        fa=max_acc(veh_type1)*(1-power(lanes{k,1}(j,3)/desire_speed(veh_type1),4)-power(desire_fdist/(lanes{k,1}(j-1,2)-lanes{k,1}(j,2)-veh_type(lanes{k,1}(j-1,5)).veh_length),2));
                        rf_dist=surr_vehs(3,1);
                        rf_v=surr_vehs(3,2);
                        desire_rfdist=net_dist(veh_type1)/10+2*sqrt(lanes{k,1}(j,3)/desire_speed(veh_type1))+reac_time(veh_type1)*lanes{k,1}(j,3)+lanes{k,1}(j,3)*(lanes{k,1}(j,3)-rf_v)/2/sqrt(max_acc(veh_type1)*comfort_dec(veh_type1)); %the longitudinal distance of turning vehicle is less
                        rfa=max_acc(veh_type1)*(1-power(lanes{k,1}(j,3)/desire_speed(veh_type1),4)-power(desire_rfdist/rf_dist,2));
                        rf_id=surr_vehs(3,3);
                        rlane=lanes{k-1,1};
                        if isempty(rlane)
                            rf_rank=1;
                            rf_lc_flag=0;
                        else
                            rf_rank=find(rlane(:,1)==rf_id);
                            rf_lc_flag=rlane(rf_rank,6);
                        end
                        if rf_rank==2 && rf_lc_flag==3 %如果右前方是第一辆车且准备换道
                            thre_p=0.01*(550-lanes{k,1}(j,2)); %阈值设定：小于该阈值只跟随前车；大于1-阈值，只跟随右前车；在之间，按比例分配
                        else
                            thre_p=0.01*(450-lanes{k,1}(j,2));
                        end
                        if Prf<thre_p 
                            selfa=fa;
                        elseif Prf<=1-thre_p
                            p_new=(Prf-thre_p)/(1-2*thre_p);
                            selfa=p_new*rfa+(1-p_new)*fa;
                        else
                            selfa=rfa;
                        end
                        if rf_lc_flag==3 & abs(rf_x-ego_x)<3 & ego_y-rf_y>max(ego_v*drt_red/2,4) %有一定概率协作减速
                            if rf_y > ramp_endpos-80 && mod(right_front_id,2)==0
                                selfa=max(-4,selfa-6);
                            end
                        end
                    else
                        if k==ramp_num %& 匝道车辆
                            if (lanes{k,1}(j,2)>300 | (lanes{k,1}(j,2)>200 & surr_vehs(1,1)>0 & surr_vehs(2,1)>0))
                                desired_v=(surr_vehs(1,2)+surr_vehs(2,2))/2;
                            else
                                desired_v=desire_speed(veh_type1);
                            end
                        else
                            desired_v=desire_speed(veh_type1);
                        end
                        desire_dist=net_dist(veh_type1)+2*sqrt(lanes{k,1}(j,3)/desired_v)+reac_time(veh_type1)*lanes{k,1}(j,3)+lanes{k,1}(j,3)*(lanes{k,1}(j,3)-lanes{k,1}(j-1,3))/2/sqrt(max_acc(veh_type1)*comfort_dec(veh_type1));
                        selfa=max_acc(veh_type1)*(1-power(lanes{k,1}(j,3)/desired_v,4)-power(desire_dist/(lanes{k,1}(j-1,2)-lanes{k,1}(j,2)-veh_type(lanes{k,1}(j-1,5)).veh_length),2));
                        if k<ramp_num+expw_num
                            llane=lanes{k+1};
                        else
                            llane=[];
                        end
                        if ~isempty(llane) %找路肩车道上未完全换道的最近的车，同样在换道过程中按一定比例跟随前车/路肩车道前车
                            lfvehs=llane(find(llane(:,6)==4 & llane(:,2)>lanes{k,1}(j,2)),:);
                            for m=length(lfvehs(:,1)):-1:2
                                ego_x=lanes{k,1}(j,8);
                                ego_y=lanes{k,1}(j,2);
                                ego_v=lanes{k,1}(j,3);
                                lf_x=lfvehs(m,8);
                                lf_y=lfvehs(m,2);
                                lf_v=lfvehs(m,3);
                                Plf=b_get_lc_prob(ego_x,lf_x,ego_v,lf_v,lane_width);
                                desire_fdist=net_dist(veh_type1)+2*sqrt(lanes{k,1}(j,3)/desired_v)+reac_time(veh_type1)*lanes{k,1}(j,3)+lanes{k,1}(j,3)*(lanes{k,1}(j,3)-lanes{k,1}(j-1,3))/2/sqrt(max_acc(veh_type1)*comfort_dec(veh_type1));
                                fa=max_acc(veh_type1)*(1-power(lanes{k,1}(j,3)/desired_v,4)-power(desire_fdist/(lanes{k,1}(j-1,2)-lanes{k,1}(j,2)-veh_type(lanes{k,1}(j-1,5)).veh_length),2));
                                lf_dist=lf_y-ego_y;
                                if lf_dist>1
                                    desire_lfdist=net_dist(veh_type1)/10+2*sqrt(lanes{k,1}(j,3)/desired_v)+reac_time(veh_type1)*lanes{k,1}(j,3)+lanes{k,1}(j,3)*(lanes{k,1}(j,3)-lf_v)/2/sqrt(max_acc(veh_type1)*comfort_dec(veh_type1)); %the longitudinal distance of turning vehicle is less
                                    lfa=max_acc(veh_type1)*(1-power(lanes{k,1}(j,3)/desired_v,4)-power(desire_lfdist/(lf_dist-veh_type(lanes{k,1}(j,5)).veh_length),2));
                                else
                                    lfa=-10;
                                end
                                thre_p=0.15;
                                if Plf<thre_p
                                    selfa=fa;
                                else
                                    if Plf<=1-thre_p
                                        p_new=(Plf-thre_p)/(1-2*thre_p);
                                        selfa=p_new*lfa+(1-p_new)*fa;
                                    else
                                        selfa=lfa;
                                    end
                                    break
                                end
                            end
                        end
                    end
                    if selfa<-6 %加速度大小限制
                        selfa=-6;
                    elseif selfa>6
                        selfa=6;
                    end
                    if k==ramp_num & lanes{k,1}(j-1,1)==10000 & selfa<-2
                        selfa=-2;
                    end
                    if j>1 & lanes{k,1}(j,2)+lanes{k,1}(j,3)*sim_reso+0.5*selfa*sim_reso*sim_reso>lanes{k,1}(j-1,2)-(veh_type(lanes{k,1}(j,5)).veh_length+veh_type(lanes{k,1}(j-1,5)).veh_length)/2
                        if j==2 & k==ramp_num & lanes{k,1}(j,3)<3 & surr_vehs(1,1)>lanes{k,1}(j,3)%排除加速车道末端车辆跟随虚拟车辆无法前进的问题
                        else %避免碰撞
                            selfa=(lanes{k,1}(j-1,2)-lanes{k,1}(j,2)-(veh_type(lanes{k,1}(j,5)).veh_length+veh_type(lanes{k,1}(j-1,5)).veh_length)/2-lanes{k,1}(j,3)*sim_reso)/0.5/sim_reso/sim_reso;
                        end
                    end
                    if lanes{k,1}(j,3)+selfa*sim_reso<0 %速度不要小于0
                        selfa=(0-lanes{k,1}(j,3))/sim_reso;% forbid reverse
                    end
                    lanes{k,1}(j,3)=lanes{k,1}(j,3)+selfa*sim_reso;%速度更新
                    if lanes{k,1}(j,3)<0 & lanes{k,1}(j,3)>-1e-2
                        lanes{k,1}(j,3)=0;
                    end
                    %计算jerk值
                    if lanes{k,1}(j,2)>start_pos
                        lanes{k,1}(j,11)=(selfa-lanes{k,1}(j,4))/0.1;
                    end
                    lanes{k,1}(j,4)=selfa;
                    %位置更新
                    new_pos=max(lanes{k,1}(j,2)+lanes{k,1}(j,3)*sim_reso+0.5*lanes{k,1}(j,4)*sim_reso*sim_reso,lanes{k,1}(j,2));
                    if lanes{k,1}(j,2)<205 && new_pos>205
                        time_eva=[time_eva;[lanes{k,1}(j,1),i*sim_reso,new_pos,0,0]];
                    elseif lanes{k,1}(j,2)<ramp_endpos && new_pos>ramp_endpos
                        id_idx=find(time_eva(:,1)==lanes{k,1}(j,1));
                        time_eva(id_idx,4)=i*sim_reso;
                        time_eva(id_idx,5)=new_pos;
                    end
                    lanes{k,1}(j,2)=new_pos;
                    if lanes{k,1}(j,2)>600
                        if isKey(traj_dict, num2str(lanes{k,1}(j,1)))
                            traj_dict.remove(num2str(lanes{k,1}(j,1)));
                        end
                        lanes{k,1}(j,1)=0; % If vehicle leaves the road, it is marked with id=0
                    end
                    
                    %位于加速车道的匝道车辆，需要计算左后方被插入车辆的舒适空间（DRT）
                    if k==ramp_num && lanes{k,1}(j,2)>200
                        left_lag_id=surr_vehs(2,3); %不放在换道模型部分是为了能让匝道车辆进来的时候，主线后车就有舒适空间展示
                        left_lag_idx=find(lanes{k+1}(:,1)==left_lag_id);
                        left_lead_id=surr_vehs(1,3);
                        left_lead_idx=find(lanes{k+1}(:,1)==left_lead_id);
                        if ~isempty(left_lag_idx)
                            left_surr_vehs=find_surr(k+1,left_lag_idx); %找左后车的周车
                            [ d1a, d2a, dlefta, drighta ] = b_PPfield_veh_params_drt( left_surr_vehs, left_lag_id, k+1, 1); %碰撞、危险、安全三层舒适空间
                            [ d1b, d2b, dleftb, drightb ] = b_PPfield_veh_params_drt( left_surr_vehs, left_lag_id, k+1, 2);
                            left_lag_x=lanes{k+1}(left_lag_idx,8);
                            left_lag_y=lanes{k+1}(left_lag_idx,2)-veh_type(lanes{k+1}(left_lag_idx,5)).veh_length/2;
                            lb_a=lanes{k+1}(left_lag_idx,4);
                            lb_jerk=lanes{k+1}(left_lag_idx,11);
                            ego_v=lanes{k,1}(j,3);
                            lf_v=surr_vehs(1,2);
                            lb_v=surr_vehs(2,2);
                            lb_x=surr_vehs(2,1);
                            gap_lf=1+0.15*2.237*max(0,ego_v-lf_v)+0.3*2.237*min(0,ego_v-lf_v)+0.2*2.237*ego_v;
                            gap_lb=0.5+0.1*2.237*max(0,lb_v-ego_v)+0.35*2.237*min(0,lb_v-ego_v)+0.25*2.237*lb_v;
                            veh_heading=lanes{k+1}(left_lag_idx,10);
                            [ego_xs,ego_ys]=b_rectA(lanes{k,1}(j,2)-veh_type(lanes{k,1}(j,5)).veh_length,lanes{k,1}(j,8)-veh_type(lanes0{k,1}(j,5)).veh_width/2,veh_type(lanes{k,1}(j,5)).veh_length,veh_type(lanes{k,1}(j,5)).veh_width,lanes{k,1}(j,10),0);
                            if ~isempty(d1a) %判断是否在碰撞区
                                field_bordera=b_rectA2(left_lag_x,left_lag_y, d1a, d2a, dlefta, drighta);
                                field_bordera0 = b_get_border(d1a, d2a, drighta, dlefta, 1);
                                field_bordera0(:,1)=field_bordera0(:,1)+left_lag_x;
                                field_bordera0(:,2)=field_bordera0(:,2)+left_lag_y;
                                [field_bordera0(:,1), field_bordera0(:,2)]=b_rotate_xy(left_lag_x, left_lag_y, field_bordera0(:,1), field_bordera0(:,2), veh_heading);
                                if plot_yes==1
                                    plot(field_bordera0(:,2),field_bordera0(:,1),'r');hold on;
                                end
                                if lanes{k,1}(j,2)-left_lag_y<d1a || surr_vehs(1,1)<gap_lf*0.5
                                    dangerous=1;
                                else
                                    dangerous=0;
                                end
                            end
                            
                            if ~isempty(d1b)%判断是否在危险区
                                field_borderb=b_rectA2(left_lag_x,left_lag_y,d1b, d2b, drightb, dleftb);
                                field_borderb0 = b_get_border(d1b, d2b, drightb, dleftb, 1);
                                field_borderb0(:,1)=field_borderb0(:,1)+left_lag_x;
                                field_borderb0(:,2)=field_borderb0(:,2)+left_lag_y;
                                [field_borderb0(:,1), field_borderb0(:,2)]=b_rotate_xy(left_lag_x, left_lag_y, field_borderb0(:,1), field_borderb0(:,2), veh_heading);
                                if plot_yes==1
                                    plot(field_borderb0(:,2),field_borderb0(:,1),'Color',[1,0.5,0]);hold on;
                                end
                                if lanes{k,1}(j,2)-left_lag_y<gap_lb
                                    not_comf=1;
                                else
                                    not_comf=0;
                                end
                            end
                            
                            if isKey(traj_dict, num2str(left_lag_id))
                                lb_data=traj_dict(num2str(left_lag_id));
                                if length(lb_data(:,1))==8 & mod(i,8)==mod(lanes{k,1}(j,1),8)%后车可能刚生成，出现时间不足10帧
%                                     %GM-HMM判断后车驾驶意图
                                    ego_data=traj_dict(num2str(lanes{k,1}(j,1)));
                                    delta_x=-(ego_data(:,8)-lb_data(:,8));
                                    delta_y=ego_data(:,2)-lb_data(:,2);
                                    lb_v=lb_data(:,3);
                                    border_d=-ego_data(:,8);
                                    rest_d=350-ego_data(:,2);
                                    testdata=[delta_x,delta_y,lb_v,border_d,rest_d];
                                    a1=mhmm_logprob(reshape(testdata',[5,8,1]), prior1, transmat1, mu1, Sigma1, mixmat1);
                                    a2=mhmm_logprob(reshape(testdata',[5,8,1]), prior2, transmat2, mu2, Sigma2, mixmat2);
                                    a=[a1,a2];
                                    if max(a)==a1
                                        not_yield=1;
                                    else
                                        not_yield=0;
                                    end
                                end
                            end
                        end
                    end
                    
                    % Lane-changing model
                    if (lanes{k,1}(j,2)<550 && lanes{k,1}(j,2)>50 && lanes{k,1}(j,1)~=0)%isempty(veh_traj{lanes{k,1}(j-1,1),1})
                        %index1:1-left lead; 2-left lag; 3-right lead; 4-right lag
                        %index2:1-distance; 2-velocity;3-id
                        dis_gap_lb=1.5+0.15*2.237*max(0,surr_vehs(2,2)-lanes{k,1}(j,3))+0.45*2.237*min(0,surr_vehs(2,2)-lanes{k,1}(j,3))+0.30*2.237*surr_vehs(2,2)+1.5*randn();
                        dis_gap_lf=1+0.15*2.237*max(0,lanes{k,1}(j,3)-surr_vehs(1,2))+0.3*2.237*min(0,lanes{k,1}(j,3)-surr_vehs(1,2))+0.2*2.237*surr_vehs(1,2)+randn();
                        dis_gap_rf=Inf;%1+0.15*2.237*max(0,lanes{k,1}(j,3)-surr_vehs(3,2))+0.3*2.237*min(0,lanes{k,1}(j,3)-surr_vehs(3,2))+0.2*2.237*surr_vehs(3,2)+randn();
                        dis_gap_rb=Inf;%1.5+0.15*2.237*max(0,surr_vehs(4,2)-lanes{k,1}(j,3))+0.35*2.237*min(0,surr_vehs(4,2)-lanes{k,1}(j,3))+0.25*2.237*surr_vehs(4,2)+0.5*randn();
                        
                        if k<=ramp_num %匝道车辆的横向偏移模型
                            if lanes{k,1}(j,2)>200 && lanes{k,1}(j,6)~=4 && surr_vehs(1,1)>0 &&...
                                    ( lanes{k,1}(j,6)==3 | (lanes{k,1}(j,6)==0 ))
                                lanes{k,1}(j,6)=3;
                                S=3.5;
                                ax=0;
                                if dangerous | (not_comf && not_yield && lanes{k,1}(j,2)<ramp_endpos-10) % 
                                    ax=(0-lanes{k,1}(j,9))/sim_reso;
                                elseif not_comf==0
                                    if Tn_record(lanes{k,1}(j,1),1)==0
                                        lc_last(num2str(lanes{k,1}(j,1)))=0;
                                        b0=1.1;b1=0.01;b2=0.03;b3=-0.001;b4=0.013;b5=-0.012;b6=-0.001;
                                        d=(length(lanes{k+1,1}(:,1))-1)/0.6;
                                        delta_vp=lanes{k,1}(j-1,3)-lanes{k,1}(j,3);
                                        gp=lanes{k,1}(j-1,2)-lanes{k,1}(j,2);
                                        delta_vlf=surr_vehs(1,2)-surr_vehs(2,2);
                                        glf=surr_vehs(1,1)+surr_vehs(2,1);
                                        Tn=b0+b1*d+b2*min(0,delta_vp)+b3*gp+b4*min(0,delta_vlf)+b5*max(0,delta_vlf)+b6*glf+(randn()+1)/2;
                                        Tn=exp(Tn);
                                        if Tn<=2
                                            Tn_record(lanes{k,1}(j,1),1)=3;
                                        else
                                            Tn_record(lanes{k,1}(j,1),1)=Tn;
                                        end
                                    end
                                    if Tn_record(lanes{k,1}(j,1),1)~=0
                                        tt=lc_last(num2str(lanes{k,1}(j,1)));
                                        tt=tt+sim_reso;
                                        lc_last(num2str(lanes{k,1}(j,1)))=tt;
                                        Tn=Tn_record(lanes{k,1}(j,1),1);
                                        ax=6*S/Tn/Tn-12*S/Tn/Tn/Tn*tt;
                                    end
                                elseif lanes{k,1}(j,2)>200+50
                                    k1=0.112;k2=0.0007;k3=1.317;k4=-0.072;
                                    q=-lanes{k,1}(j,8)/1.75;
                                    gl=surr_vehs(1,1);
                                    gf=surr_vehs(2,1);
                                    delta_vl=surr_vehs(1,2)-lanes{k,1}(j,3);
                                    delta_vf=lanes{k,1}(j,3)-surr_vehs(2,1);
                                    ax=(k1*gl+k2*gf+k3*delta_vl+k4*delta_vf)*q*q*q;
                                    if not_comf==0 & abs(q)<0.1 & abs(lanes{k,1}(j,9))<0.5
                                        ax=0.5;
                                    end
                                end
                                ax=min(ax,min(3,lanes{k,1}(j,3)*0.6));
                                lanes{k,1}(j,9)=lanes{k,1}(j,9)+ax*sim_reso;
                                lanes{k,1}(j,9)=max(0,min(lanes{k,1}(j,9),lanes{k,1}(j,3)*tand(15)));
                                new_posx=max(lanes{k,1}(j,8)+lanes{k,1}(j,9)*sim_reso+0.5*ax*sim_reso*sim_reso,-1.75);
                                lanes{k,1}(j,8)=new_posx;
                                if lanes{k,1}(j,9)~=0 & lanes{k,1}(j,3)~=0
                                    lanes{k,1}(j,10)=atan(lanes{k,1}(j,9)/lanes{k,1}(j,3));
                                end
                            end
                        elseif lanes{k,1}(j,6)==0 %主线车辆换道模型
                            if (lanes{k,1}(j-1,5)==2 &&  lanes{k,1}(j-1,2)-lanes{k,1}(j,2)<2*lanes{k,1}(j,3)) %following a bus or truck
                                if (lanes{k,1}(j-1,3)<surr_vehs(1,2) && surr_vehs(1,1)>lanes{k,1}(j-1,2)-lanes{k,1}(j,2)&& surr_vehs(1,1)>dis_gap_lf && surr_vehs(2,1)>dis_gap_lb)%LC incentive
                                    lanes{k,1}(j,6)=1;
                                    veh_traj{lanes{k,1}(j,1),1}=LC_traj(k,j);
                                    lc_record=[lc_record;[lanes{k,1}(j,1),k,k+1]];
                                elseif (lanes{k,1}(j-1,3)<surr_vehs(3,2) && surr_vehs(1,2)>lanes{k,1}(j-1,2)-lanes{k,1}(j,2) &&surr_vehs(3,1)>dis_gap_rf && surr_vehs(4,1)>dis_gap_rb) && k>ramp_num+1
                                    lanes{k,1}(j,6)=2;
                                    veh_traj{lanes{k,1}(j,1),1}=LC_traj(k,j);
                                    lc_record=[lc_record;[lanes{k,1}(j,1),k,k-1]];
                                end
                            end
                            if (lanes{k,1}(j,6)==0 && lanes{k,1}(j,3)<min(desire_speed(veh_type1),lane_speed_limit(k))-20/3.6 && lanes{k,1}(j-1,3)<lanes{k,1}(j,3)) %LC incentive
                                if ((lanes{k,1}(j-1,3)<surr_vehs(1,2)-10/3.6 | (lanes{k,1}(j-1,3)<surr_vehs(1,2) && lanes{k,1}(j-1,4)<0)) && surr_vehs(1,1)>dis_gap_lf && surr_vehs(2,1)>dis_gap_lb && surr_vehs(1,1)>lanes{k,1}(j-1,2)-lanes{k,1}(j,2))
                                    lanes{k,1}(j,6)=1;
                                    veh_traj{lanes{k,1}(j,1),1}=LC_traj(k,j);
                                    lc_record=[lc_record;[lanes{k,1}(j,1),k,k+1]];
                                elseif (lanes{k,1}(j-1,3)<surr_vehs(3,2)-10/3.6 | (lanes{k,1}(j-1,3)<surr_vehs(3,2) && lanes{k,1}(j-1,4)<0))&& surr_vehs(3,1)>dis_gap_rf && surr_vehs(4,1)>dis_gap_rb && surr_vehs(3,1)>lanes{k,1}(j-1,2)-lanes{k,1}(j,2) && k>ramp_num+1
                                    lanes{k,1}(j,6)=2;
                                    veh_traj{lanes{k,1}(j,1),1}=LC_traj(k,j);
                                    lc_record=[lc_record;[lanes{k,1}(j,1),k,k-1]];
                                end
                            end
                        elseif k==ramp_num+1 && lanes{k,1}(j,6)==4 %已经从匝道换到主线上，但还处于换道状态
                            if j<length(lanes{k,1}(:,1))
                                behind_surr_vehs=find_surr(k,j+1); %找后车的周车
                                [ d1a, d2a, dlefta, drighta ] = b_PPfield_veh_params_drt( behind_surr_vehs, lanes{k,1}(j+1,1), k, 1); %红黄绿三层舒适空间
                                [ d1b, d2b, dleftb, drightb ] = b_PPfield_veh_params_drt( behind_surr_vehs, lanes{k,1}(j+1,1), k, 2);
                                behind_x=lanes{k,1}(j+1,8);
                                behind_y=lanes{k,1}(j+1,2)-veh_type(lanes{k,1}(j+1,5)).veh_length/2;
                                veh_heading=lanes{k,1}(j+1,10);
                                [ego_xs,ego_ys]=b_rectA(lanes{k,1}(j,2)-veh_type(lanes{k,1}(j,5)).veh_length,lanes{k,1}(j,8)-veh_type(lanes{k,1}(j,5)).veh_width/4,veh_type(lanes{k,1}(j,5)).veh_length,veh_type(lanes{k,1}(j,5)).veh_width,lanes{k,1}(j,10),0);
                                if ~isempty(d1a)
                                    field_bordera = b_get_border(d1a, d2a, drighta, dlefta, 1);
                                    field_bordera(:,1)=field_bordera(:,1)+behind_x;
                                    field_bordera(:,2)=field_bordera(:,2)+behind_y;
                                    [field_bordera(:,1), field_bordera(:,2)]=b_rotate_xy(behind_x, behind_y, field_bordera(:,1), field_bordera(:,2), veh_heading);
                                    point_in=inpolygon(ego_ys,ego_xs,field_bordera(:,1),field_bordera(:,2));
                                    if plot_yes==1
                                        plot(field_bordera(:,2),field_bordera(:,1),'r');hold on;
                                    end
                                    if any(point_in)
                                        dangerous=1;
                                    else
                                        dangerous=0;
                                    end
                                end
                                
                                if ~isempty(d1b)
                                    field_borderb = b_get_border(d1b, d2b, drightb, dleftb, 1);
                                    field_borderb(:,1)=field_borderb(:,1)+behind_x;
                                    field_borderb(:,2)=field_borderb(:,2)+behind_y;
                                    [field_borderb(:,1), field_borderb(:,2)]=b_rotate_xy(behind_x, behind_y, field_borderb(:,1), field_borderb(:,2), veh_heading);
                                    point_in=inpolygon(ego_ys,ego_xs,field_borderb(:,1),field_borderb(:,2));
                                    if plot_yes==1
                                        plot(field_borderb(:,2),field_borderb(:,1),'Color',[1,0.5,0]);
                                    end
                                    if any(point_in)
                                        not_comf=1;
                                    else
                                        not_comf=0;
                                    end
                                end
                            end
                            
                            %计算匝道车辆已经进入路肩车道后的横向加速度
                            k1=0.112;k2=0.0007;k3=1.317;k4=-0.072;
                            q=-lanes{k,1}(j,8)/1.75;
                            if j~=1
                                gl=lanes{k,1}(j-1,2)-lanes{k,1}(j,2);
                                delta_vl=lanes{k,1}(j-1,3)-lanes{k,1}(j,3);
                            else
                                g1=200;
                                delta_vl=20-lanes{k,1}(j,3);
                            end
                            if j~=length(lanes{k,1}(:,1))
                                gf=lanes{k,1}(j,2)-lanes{k,1}(j+1,2);
                                delta_vf=lanes{k,1}(j,3)-lanes{k,1}(j+1,3);
                            else
                                gf=200;
                                delta_vf=lanes{k,1}(j,3);
                            end
                            ax=(k1*gl+k2*gf+k3*delta_vl+k4*delta_vf)*q*q*q;
                            ax=min(ax,min(3,lanes{k,1}(j,3)*0.6));
                            if not_comf==0 & abs(q)<0.1 & abs(lanes{k,1}(j,9))<0.5
                                ax=0.5;
                            end
                            lanes{k,1}(j,9)=lanes{k,1}(j,9)+ax*sim_reso;
                            lanes{k,1}(j,9)=max(0,min(lanes{k,1}(j,9),lanes{k,1}(j,3)*tand(15)));
                            new_posx=max(lanes{k,1}(j,8)+lanes{k,1}(j,9)*sim_reso+0.5*ax*sim_reso*sim_reso,-1.75);
                            lanes{k,1}(j,8)=new_posx;
                            if lanes{k,1}(j,9)~=0 && lanes{k,1}(j,3)~=0
                                lanes{k,1}(j,10)=atan(lanes{k,1}(j,9)/lanes{k,1}(j,3));
                            end

                            if lanes{k,1}(j,8)>1.75-veh_type(lanes{k,1}(j,5)).veh_width/2; %汇入车辆正式转为加速车道车辆
                                lanes{k,1}(j,8)=1.75;
                                lanes{k,1}(j,9)=0;
                                lanes{k,1}(j,6)=0;
                                lanes{k,1}(j,10)=0;
                            end
                        end
                    end
                end
            end
            lanes{k,1}=lanes{k,1}(lanes{k,1}(:,1)>0,:); % delete vehicles marked with id 0
        end
        
       %% figure output
        % draw road lines
        % three lane in mainline with 600m
        % one lane on ramp with 200m long acceleartion lane
        if (~isempty(lanes0{k,1}))
            if plot_yes==1
                line([50 200],[-lane_width*6 0],'Color','k');hold on;
                line([50 200],[-lane_width*6-lane_width/cos(atan(lane_width/25)) -lane_width],'Color','k');hold on;
                line([200 ramp_endpos],[-lane_width -lane_width],'Color','k');hold on;
                line([ramp_endpos ramp_endpos+20],[-lane_width 0],'Color','k');hold on;
                line([0 200],[0 0],'Color','k');hold on;
                line([ramp_endpos+20 600],[0 0],'Color','k');hold on;
                line([200 ramp_endpos+20],[0 0],'linestyle',':','Color','k');hold on;
                line([0 600],[lane_width lane_width],'linestyle',':','Color','k');hold on;
                line([0 600],[lane_width*2 lane_width*2],'linestyle',':','Color','k');hold on;
                line([0 600],[lane_width*3 lane_width*3],'Color','k');hold on;
                text(500, -100, ['simulation time: ',num2str(i/10)]);
            end
            
            for j=length(lanes0{k,1}(:,1)):-1:1
                %Method1：Use recangle function to draw vehicles
                if lanes0{k,1}(j,6)==0
                    if (k==1 && lanes0{k,1}(j,2)>50 && lanes0{k,1}(j,2)<ramp_endpos+5)
                        if lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length>200  %acceleration lane
                            b_rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2-lane_width,veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,0,lanes0{k,1}(j,7));
                        else %on-ramp
                            b_rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,3.5/25*(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length)-32-(veh_type(lanes0{k,1}(j,5)).veh_width-lane_width)/2,veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,atan(3.5/25),lanes0{k,1}(j,7));
                        end
                    elseif k==2
                        b_rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2,veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,0,lanes0{k,1}(j,7));
                    elseif k==3
                        b_rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2+lane_width,veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,0,lanes0{k,1}(j,7));
                    elseif k==4
                        b_rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2+lane_width*2,veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,0,lanes0{k,1}(j,7));
                    end
                elseif lanes0{k,1}(j,6)==3 | lanes0{k,1}(j,6)==4
                    b_rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,lanes0{k,1}(j,8)-veh_type(lanes0{k,1}(j,5)).veh_width/2,veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,lanes0{k,1}(j,10),lanes0{k,1}(j,7));
                    if j>1
                        front_id=lanes0{k,1}(j-1,1);
                    else
                        front_id=0;
                    end
                    surr_vehs=find_surr(k,j);
                    if k==ramp_num && front_id==10000 && lanes0{k,1}(j,8)+veh_type(lanes0{k,1}(j,5)).veh_width*0.99>0 && lanes0{k,1}(j,6)==3 &&...
                            ((lanes0{k,1}(j,2)>ramp_endpos-40 && lanes0{k,1}(j,2)<ramp_endpos-10 && surr_vehs(2,1)>surr_vehs(2,2)*drt_red*0.5) ||... %加速车道末端必须汇入 turn-taking
                            (lanes0{k,1}(j,2)>ramp_endpos-10 && surr_vehs(2,1)>surr_vehs(2,2)*drt_red*0.7)) %模拟压迫性汇入
                        %如果在匝道末端5m内，让车辆直接汇入
                        lanes{k+1,1}(end+1,:)=lanes0{k,1}(j,:);
                        lanes{k+1,1}(end,6)=4;
                        lanes{k+1,1}(end,7)=3; 
                        lanes{k+1,1}=sortrows(lanes{k+1,1},-2);
                        origin_order=find(lanes{k,1}(:,1)==lanes0{k,1}(j,1));
                        lanes{k,1}(origin_order,:)=[];
                    elseif lanes0{k,1}(j,8)+veh_type(lanes0{k,1}(j,5)).veh_width/2>0 && lanes0{k,1}(j,6)==3
                        lanes{k+1,1}(end+1,:)=lanes0{k,1}(j,:);
                        lanes{k+1,1}(end,6)=4;
                        lanes{k+1,1}(end,7)=3;
                        lanes{k+1,1}=sortrows(lanes{k+1,1},-2);
                        origin_order=find(lanes{k,1}(:,1)==lanes0{k,1}(j,1));
                        lanes{k,1}(origin_order,:)=[];
                    end
                else
                    if lanes0{k,1}(j,2)<veh_traj{lanes0{k,1}(j,1),1}(end-2,1) %车辆还未行驶到轨迹终点
                        for m=1:length(veh_traj{lanes0{k,1}(j,1),1})-5
                            if veh_traj{lanes0{k,1}(j,1),1}(m,1)<lanes0{k,1}(j,2)
                                continue
                            else
                                %在换道过程中，如果车辆中心越过车道线就把车辆分配到新车道去
                                %calculate the heading angle
                                veh_heading=atan((veh_traj{lanes0{k,1}(j,1),1}(m+5,2)-veh_traj{lanes0{k,1}(j,1),1}(m,2))/(veh_traj{lanes0{k,1}(j,1),1}(m+5,1)-veh_traj{lanes0{k,1}(j,1),1}(m,1)));
                                lanes_idx=find(lanes{k,1}(:,1)==lanes0{k,1}(j,1));
                                lanes{k,1}(lanes_idx,10)=veh_heading;
                                if (lanes0{k,1}(j,6)==1 && veh_traj{lanes0{k,1}(j,1),1}(m,2)+veh_type(lanes0{k,1}(j,5)).veh_width>lc_record(find(lc_record(:,1)==lanes0{k,1}(j,1)),2)*3.5-3.5)
                                    new_lane=lc_record(find(lc_record(:,1)==lanes0{k,1}(j,1)),3);
                                    if ~ismember(lanes0{k,1}(j,1),lanes{new_lane,1}(:,1))
                                        lanes{new_lane,1}(end+1,:)=lanes0{k,1}(j,:);
                                        lanes{new_lane,1}=sortrows(lanes{new_lane,1},-2);
                                        origin_order=find(lanes{k,1}(:,1)==lanes0{k,1}(j,1));
                                        lanes{k,1}(origin_order,:)=[];
                                    end
                                else if (lanes0{k,1}(j,6)==2 && veh_traj{lanes0{k,1}(j,1),1}(m,2)+veh_type(lanes0{k,1}(j,5)).veh_width>lc_record(find(lc_record(:,1)==lanes0{k,1}(j,1)),3)*3.5-3.5)
                                        new_lane=lc_record(find(lc_record(:,1)==lanes0{k,1}(j,1)),2);
                                        if ~ismember(lanes0{k,1}(j,1),lanes{new_lane,1}(:,1))
                                            lanes{new_lane,1}(end+1,:)=lanes0{k,1}(j,:);
                                            lanes{new_lane,1}=sortrows(lanes{new_lane,1},-2);
                                            origin_order=find(lanes{k,1}(:,1)==lanes0{k,1}(j,1));
                                            lanes{k,1}(origin_order,:)=[];
                                        end
                                    end
                                end
                                break
                            end
                        end
                        b_rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,veh_traj{lanes0{k,1}(j,1),1}(m,2),veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,veh_heading,lanes0{k,1}(j,7));
                    else
                        b_rectA(lanes0{k,1}(j,2)-veh_type(lanes0{k,1}(j,5)).veh_length,(lane_width-veh_type(lanes0{k,1}(j,5)).veh_width)/2+lane_width*(k-2),veh_type(lanes0{k,1}(j,5)).veh_length,veh_type(lanes0{k,1}(j,5)).veh_width,0,lanes0{k,1}(j,7));
                        if k==lc_record(find(lc_record(:,1)==lanes0{k,1}(j,1)),3)
                            lanes{k,1}(find(lanes{k,1}(:,1)==lanes0{k,1}(j,1)),6)=0;
                            veh_traj{lanes0{k,1}(j,1),1}=[];
                            lc_record(find(lc_record(:,1)==lanes0{k,1}(j,1)),:)=[];
                        end
                    end
                    
                end
                
                if k>ramp_num+1
                    continue
                end
                
                if j>length(lanes{k,1}(:,1))
                    continue
                end
                if ~isKey(traj_dict, num2str(lanes{k,1}(j,1)))
                    traj_dict(num2str(lanes{k,1}(j,1))) =lanes{k,1}(j,:);   % 新生成的车辆，添加数据
                else
                    temp_traj=traj_dict(num2str(lanes{k,1}(j,1)));
                    temp_traj(end+1,:)=lanes{k,1}(j,:);
                    if length(temp_traj(:,1))>8
                        temp_traj(1,:)=[];
                    end
                    traj_dict(num2str(lanes{k,1}(j,1)))=temp_traj;
                end
            end
        end
    end
    if mod(i,10)==0
        disp(['simulation time: ',num2str(i/10)]);
    end
    if plot_yes==1
        axis([150 450 -60 60]);
        drawnow %show picture
        clf %clear picture
    end
end