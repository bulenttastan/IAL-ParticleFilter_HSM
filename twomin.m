function [ mins, pts ] = twomin( x )
%finds the two minimums in a vector
%ie: [mins,pts]=twomin([3 4 1 0]);
%disp(mins); -> 0 1
%disp(pts); -> 4 3
mins(1) = Inf;
mins(2) = Inf;
pts = [numel(x) numel(x)];
for ind = 1:length(x)
    if x(ind) < mins(1)
        mins(2) = mins(1);
        mins(1) = x(ind);
        pts(2) = pts(1);
        pts(1) = ind;
    elseif x(ind) < mins(2)
        mins(2) = x(ind);
        pts(2) = ind;
    end
end
