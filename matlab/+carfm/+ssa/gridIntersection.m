function [i1, i2] = gridIntersection(grid1, grid2)
% GRIDINTERSECTION  Find the intersection between two n-dimensiona grid 
% defined by the grid vectors grid{1},grid{2},...,grid{n}, such that
% intersected_grid = {grid1{1}(ia{1}), ..., grid1{n}(ia{n})}
% and
% intersected_grid = {grid2{1}(i2{1}), ..., grid2{n}(i2{n})}

% Check args
arguments
    grid1 (1,:) cell
    grid2 (1,:) cell {mltsfm.common.mustBeEqualSize(grid1,grid2)}
end

% Init intersection indeces
n = numel(grid1); % grid dimension
i1 = cell(1,n);
i2 = cell(1,n);

% Loop through each dimension to find intersection indices
for dim = 1:n
    [~, i1{dim}, i2{dim}] = intersect(grid1{dim}, grid2{dim});
end