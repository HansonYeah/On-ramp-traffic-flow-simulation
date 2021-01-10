function [X, Y]=rectA(x0,y0,h,w,A,color_type)
% draw rectangle with rotation
% x0 is the virtical coordinate of rectangle's left bottom(vehicle's right bottom in driving direction)
% y0 is the longitudinal coordinate of rectangle's left bottom(vehicle's right bottom in driving direction)
% h is length
% w is width
% A is the rotation angle (in rad)
X=[x0,x0+h,x0+h,x0,x0]; 
Y=[y0,y0,y0+w,y0+w,y0]; 
Z=X+Y*i;
Z=[Z-[X(1)+Y(1)*i]]*exp(i*A)+[X(1)+Y(1)*i];

if color_type==0
    return
else
    h0=plot(Z,'b');
end

% X=real(Z);
% Y=imag(Z);