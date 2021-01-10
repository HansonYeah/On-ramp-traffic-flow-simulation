function [ hdw ] = Rand_Hdw( flow )
%flow is the hourly vehicle input

%hdw is caculated by Shifted Exponential Distribution
% lamda=flow/3600;
% temp=clock;
% rand_hdw=mod(temp(6)*1000,10)/10+mod(floor(temp(6)*100),10)/100+mod(floor(temp(6)*10),10)/1000; %generate the true random number
% rand_hdw=max(rand_hdw,0.001);
% hdw=(-log(rand_hdw))/lamda;

%hdw is caculated by Normal Distribution
hdw=max(3600/flow+0.5*randn(),0.1);
end

