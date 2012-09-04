function index=findfirst(t,x,varargin)
%finds first occurrence of x in t after ind
%inputs: t,x,ind
%usage
% ind=findfirst(t,1);
% disp(num2str(t(ind)));

index = numel(t); ind=1;
if nargin==3; ind=varargin{1}; end;
for i=ind:numel(t)
    if x>t(i); continue; end
    index=i;
    break;
end