function [unsorteduniques ia ib] = uunique(vec,varargin)
%     vec = vec(:)';
    [v a b] = unique(vec, 'first','rows');
    if nargout > 2
        [ia v] = sort(a);
        [v ib] = ismember(b, v);
    else
       ia = sort(a);
    end
    unsorteduniques = vec(ia,:);
end