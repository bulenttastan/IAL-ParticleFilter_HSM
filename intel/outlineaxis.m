function [ x1,x2,y1,y2 ] = outlineaxis( Px, Py )

x1 = floor(min(Px));
x2 = ceil(max(Px));
y1 = floor(min(Py));
y2 = ceil(max(Py));