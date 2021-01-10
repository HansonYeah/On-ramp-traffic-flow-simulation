function [ ego_xy ] = b_rectA2( y0, x0, d1, d2, dleft, dright )
x1=x0+d1;y1=y0-dright;
x2=x0+d1;y2=y0+dleft;
x3=x0-d2;y3=y0+dleft;
x4=x0-d2;y4=y0-dright;
ego_xs=[x1;x2;x3;x4;x1];
ego_ys=[y1;y2;y3;y4;y1];
ego_xy=[ego_ys,ego_xs];
end