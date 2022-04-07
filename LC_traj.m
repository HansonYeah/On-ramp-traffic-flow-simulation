function [ A ] = LC_traj( k, j ) %车道k的第j辆车
%计算换道轨迹
global lanes
global veh_type
global lane_width

if lanes{k,1}(j,3)<20/3.6 %设定换道距离，与速度相关
    lc_dist=veh_type(lanes{k,1}(j,5)).veh_length*2;
elseif lanes{k,1}(j,3)<30/3.6
    lc_dist=veh_type(lanes{k,1}(j,5)).veh_length*5;
elseif lanes{k,1}(j,3)<40/3.6
    lc_dist=veh_type(lanes{k,1}(j,5)).veh_length*6;
else
    lc_dist=veh_type(lanes{k,1}(j,5)).veh_length*8;
end

A(1,1)=0;A(1,2)=0; %start points
if lanes{k,1}(j,5)==1
    A(2,1)=lc_dist/2-1.75*1/tan((atan(lane_width/lc_dist)/pi*180+2)/180*pi);
else
    A(2,1)=lc_dist/2-1.75*1/tan((atan(lane_width/lc_dist)/pi*180+1)/180*pi);
end
A(2,2)=0; %turning points
A(3,1)=lc_dist/2;A(3,2)=1.75; %terminal points
B=[]; %temp variable
C=1;%segment length
m=1;
while (max(C)>0.1)
    m=m+1;
    C=[];
    n=2;
    for kk=1:length(A(:,1))-1
        B(2*n-3,:)=2/3*A(n-1,:)+1/3*A(n,:);
        B(2*n-2,:)=1/3*A(n-1,:)+2/3*A(n,:);
        n=n+1;
    end
    n=n-1;
    B(1,:)=A(1,:);
    B(2*n-2,:)=A(end,:);
    A=sortrows(B,1);
    for kk=1:length(B(:,1))-1
        C(kk)=sqrt((A(kk,1)-A(kk+1,1))^2+(A(kk,2)-A(kk+1,2))^2);
    end
    B=[];
end

% A:trajectory
% flip to the other lane side
for m=kk+2:2*kk+1
    A(m,:)=2*A(kk+1,:)-A(2*kk+2-m,:);
end
% convert to absolute coordinate
A(:,1)=A(:,1)+lanes{k,1}(j,2);
if lanes{k,1}(j,6)==1
    A(:,2)=A(:,2)+(lane_width-veh_type(lanes{k,1}(j,5)).veh_width)/2+lane_width*(k-2);
elseif lanes{k,1}(j,6)==2
    A(:,2)=-A(:,2)+(lane_width-veh_type(lanes{k,1}(j,5)).veh_width)/2+lane_width*(k-2);
end
    
end

