function [ border ] = get_border( d1, d2, dleft, dright, radius )
%根据各向异性的长度生成虚拟场的边界
borderx=[];
bordery=[];
for theta=0:0.1:pi/2
    borderx=[borderx;radius*dright*cos(theta)];
    bordery=[bordery;radius*d1*sin(theta)+2];
end
for theta=pi/2:0.1:pi
    borderx=[borderx;radius*dleft*cos(theta)];
    bordery=[bordery;radius*d1*sin(theta)+2];
end
for theta=pi:0.1:3/2*pi
    borderx=[borderx;radius*dleft*cos(theta)];
    bordery=[bordery;radius*d2*sin(theta)-2];
end
for theta=3/2*pi:0.1:2*pi
    borderx=[borderx;radius*dright*cos(theta)];
    bordery=[bordery;radius*d2*sin(theta)-2];
end
border=[borderx bordery];
border(end,:)=border(1,:);
end

