function [ surr_vehs ] = find_surr( k, j )
%find surrdounding vehicles of No.j vehicle at lane k
global expw_num;
global ramp_num;
global lanes;
global veh_type
surr_vehs=zeros(4,4);
surr_vehs(1,1)=100;surr_vehs(2,1)=100;surr_vehs(3,1)=100;surr_vehs(4,1)=100;
surr_vehs(1,2)=35;surr_vehs(2,2)=0;surr_vehs(3,2)=35;surr_vehs(4,2)=0;
%index1:1-left lead; 2-left lag; 3-right lead; 4-right lag
%index2:1-distance; 2-velocity;3-id;4-x
if k>ramp_num
    right_vehs=[lanes{k-1,1};lanes{k,1}(j,:)];
    right_vehs=sortrows(right_vehs,2);
    right_count=find(right_vehs(:,1)==lanes{k,1}(j,1));
    right_count=right_count(1);
    if right_count>1 
        surr_vehs(4,1)=right_vehs(right_count,2)-right_vehs(right_count-1,2)-veh_type(right_vehs(right_count,5)).veh_length;
        surr_vehs(4,2)=right_vehs(right_count-1,3);
        surr_vehs(4,3)=right_vehs(right_count-1,1);
        surr_vehs(4,4)=right_vehs(right_count-1,8);
    end
    if right_count<length(right_vehs(:,1))
        surr_vehs(3,1)=right_vehs(right_count+1,2)-right_vehs(right_count,2)-veh_type(right_vehs(right_count+1,5)).veh_length;
        surr_vehs(3,2)=right_vehs(right_count+1,3);
        surr_vehs(3,3)=right_vehs(right_count+1,1);
        surr_vehs(3,4)=right_vehs(right_count+1,8);
    end
else
    surr_vehs(3,1)=0;surr_vehs(4,1)=0;surr_vehs(3,2)=0;surr_vehs(4,2)=20;% the rightmost lane
end

if k<ramp_num+expw_num
    left_vehs=[lanes{k+1,1};lanes{k,1}(j,:)];
    left_vehs=sortrows(left_vehs,2);
    left_count=find(left_vehs(:,1)==lanes{k,1}(j,1));
    left_count=left_count(1);
    if left_count>1
        surr_vehs(2,1)=left_vehs(left_count,2)-left_vehs(left_count-1,2)-veh_type(left_vehs(left_count,5)).veh_length;
        surr_vehs(2,2)=left_vehs(left_count-1,3);
        surr_vehs(2,3)=left_vehs(left_count-1,1);
        surr_vehs(2,4)=left_vehs(left_count-1,8);
    end
    if left_count<length(left_vehs(:,1))
        surr_vehs(1,1)=left_vehs(left_count+1,2)-left_vehs(left_count,2)-veh_type(left_vehs(left_count+1,5)).veh_length;
        surr_vehs(1,2)=left_vehs(left_count+1,3);
        surr_vehs(1,3)=left_vehs(left_count+1,1);
        surr_vehs(1,4)=left_vehs(left_count+1,8);
    end
else
    surr_vehs(1,1)=0;surr_vehs(2,1)=0;surr_vehs(1,2)=0;surr_vehs(2,2)=20;% the leftmost lane
end

if k==2% the rightmost lane
    if surr_vehs(3,3)==0 || lanes{k,1}(j,2)>400
    surr_vehs(3,1)=0;surr_vehs(4,1)=0;
    end
    if surr_vehs(4,3)==0 || lanes{k,1}(j,2)>400
        surr_vehs(3,2)=0;surr_vehs(4,2)=20;
    end
end
if (k==1 && lanes{k,1}(j,2)<200)
     surr_vehs(1,1)=0;surr_vehs(2,1)=0;surr_vehs(1,2)=0;surr_vehs(2,2)=20;% the leftmost lane
end

end