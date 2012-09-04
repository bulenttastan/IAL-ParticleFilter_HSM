function tableresult(ss,area,varargin)
% creates a table with inputs:
% sumsquare, area and name(optional)
name=''; if nargin==3; name=varargin{1}; end
f = figure('Position',[400 100 220 length(ss)*17+34],'Name',name);
if size(ss,1)==1
    dat = [ss' area'];
else
    dat = [ss area];
end
cnames = {'Sum Square','Area'};
uitable('Data',dat,'ColumnNames',cnames,...
            'Parent',f,'Position',[20 4 200 length(ss)*17+28]);