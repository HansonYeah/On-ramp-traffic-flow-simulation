function [ d1, d2, dleft, dright ] = b_PPfield_veh_params_drt( left_surr_vehs, veh_id, k, rank) %TODO:单位转化b_PPfield_veh_params( behind_surr_vehs, lanes0, lanes{k,1}(j+1,1), 1.0, 1.0, 1.0, 1.0, k)
global lanes
global veh_type
global ramp_num

if find(lanes{ramp_num,1}(:,1)==veh_id)
    lc_phase=3;
else
    lc_phase=4;
end

LW=3.5;
dec_max=6;
drt_red=0.65;
drt_org=1.26;

ego_index=find(lanes{k,1}(:,1)==veh_id);
if ego_index==1
    front_index=[];
else
    front_index=ego_index-1;
end
if ego_index==length(lanes{k,1}(:,1))
    behind_index=[];
else
    behind_index=ego_index+1;
end
if isempty(lanes{k+1,1})
    lfront_index=[];
else
    lfront_index=find(lanes{k+1,1}(:,1)==left_surr_vehs(1,3));
end
if isempty(lanes{k-1,1})
    rfront_index=[];
    rbehind_index=[];
else
    rfront_index=find(lanes{k-1,1}(:,1)==left_surr_vehs(3,3));
    if rfront_index ==1
        rfront_index=[];
    end
    rbehind_index=find(lanes{k-1,1}(:,1)==left_surr_vehs(4,3));
end
%中间变量
%1-vehicle id, 2-position, 3-speed, 4-accelearation 5-vehicle type 
%6-lane change flag 7-color type 8-posx 9-speedx
veh_length=veh_type(lanes{k,1}(ego_index,5)).veh_length;
veh_width=veh_type(lanes{k,1}(ego_index,5)).veh_width;
Vy=lanes{k,1}(ego_index,3);
Vx=lanes{k,1}(ego_index,9);
X=lanes{k,1}(ego_index,8);
if isempty(front_index)
    Vfy=30;
    Vfx=Vx;
    Xf=X;
else
    Vfy=lanes{k,1}(front_index,3);
    Vfx=lanes{k,1}(front_index,9);
    Xf=lanes{k,1}(front_index,8);
end
if isempty(behind_index)
    Vby=0;
    Vbx=Vy;
    Xb=X;
else
    Vby=lanes{k,1}(behind_index,3);
    Vbx=lanes{k,1}(behind_index,9);
    Xb=lanes{k,1}(behind_index,8);
end
if isempty(lfront_index)
    Vlfx=Vx;
else
    Vlfx=lanes{k+1,1}(lfront_index,9);
end

if isempty(rfront_index)
    Vrfy=Vy;
    Vrfx=Vx;
else
    Vrfy=lanes{k-1,1}(rfront_index,3);
    Vrfx=lanes{k-1,1}(rfront_index,9);
end
if isempty(rbehind_index)
    Vrby=0;
    Vrbx=Vx;
else
    Vrby=lanes{k-1,1}(rbehind_index,3);
    Vrbx=lanes{k-1,1}(rbehind_index,9);
end

delta_Vfx=Vfx-Vx;
delta_Vbx=Vbx-Vx;
delta_Vlfx=Vx-Vlfx;
delta_Vrfx=Vrfx-Vx;
delta_Vrbx=Vrbx-Vx;

delta_Xf=Xf-X;
delta_Xb=Xb-X;

Prf=1;
Prb=1;

if rank == 1
    Dphy1f=drt_red*Vy-(Vfy*Vfy-Vy*Vy)/2/dec_max;
elseif rank == 2
    Dphy1f=drt_org*Vy-(Vfy*Vfy-Vy*Vy)/2/dec_max;
else
    Dphy1f=drt_green*Vy-(Vfy*Vfy-Vy*Vy)/2/dec_max;
end

if rank == 1
    Dphy1rf=drt_red*Vy-(Vrfy*Vrfy-Vy*Vy)/2/dec_max;
elseif rank ==2
    Dphy1rf=drt_org*Vy-(Vrfy*Vrfy-Vy*Vy)/2/dec_max;
else
    Dphy1rf=drt_green*Vy-(Vrfy*Vrfy-Vy*Vy)/2/dec_max;
end

if rank == 1
    Dphy2b=drt_red*Vby-(Vby*Vby-Vy*Vy)/2/dec_max;
elseif rank ==2
    Dphy2b=drt_org*Vby-(Vby*Vby-Vy*Vy)/2/dec_max;
else
    Dphy2b=drt_green*Vby-(Vby*Vby-Vy*Vy)/2/dec_max;
end
    
if rank ==1
    Dphy2rb=drt_red*Vrby-(Vrby*Vrby-Vy*Vy)/2/dec_max;
elseif rank ==2
    Dphy2rb=drt_org*Vrby-(Vrby*Vrby-Vy*Vy)/2/dec_max;
else
    Dphy2rb=drt_green*Vrby-(Vrby*Vrby-Vy*Vy)/2/dec_max;
end

Dphyf=delta_Vfx;
Dphyrf=delta_Vrfx;
Dphyrb=delta_Vrbx;
Dphyb=delta_Vbx;

apsy1f=1; %TODO:mu未知
apsy1rf=Prf;
apsy2b=1/2;
apsy2rb=Prb/2;
apsyf=1;
apsyb=1/2;
apsyrf=Prf;
apsyrb=Prb/2;

d1=Dphy1f*apsy1f;
if Prf>0
    d1=max([d1,Dphy1rf*apsy1rf]);
end

d2=Dphy2b*apsy2b;
if Prb>0
    d2=min([d2,Dphy2rb*apsy2rb]);
end

dright=max([Dphyrf*apsyrf,Dphyrb*apsyrb]);
if delta_Xf>veh_width
    dright=max([dright,Dphyf*apsyf]);
end
if delta_Xb>veh_width
    dright=max([dright,Dphyb*apsyb]);
end

d1=max([d1,veh_length/2.0]);d2=max([d2,veh_length/2.0]);

dright=LW/2.0;
dleft=LW/2.0;
if lc_phase==3
    dright=LW/2.0+max(delta_Vlfx,delta_Vfx)*10;
end
end