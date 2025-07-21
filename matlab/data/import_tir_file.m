function [tyre] = import_tir_file(tir_file, tyre)
% This function imports all the coefficients defined inside a selected
% *.tir file.
% Function inputs:
%   - tir_file: absolute path of the selected *.tir file (with file extention)
% Function output:
% tyre: structure containing a *.tir coefficient for each field (named as
%       the coefficient)

% Opens the file for reading only
fileid = fopen(tir_file,'r');

% Get the firts file line
fileline = fgets(fileid);

% Check if tyre provided
if nargin<2
    tyre = struct();
end

% Scan each file line for coefficients
while ischar(fileline)
    if numel(fileline)>0
        if ~strcmp(fileline(1),'!') && ~strcmp(fileline(1),'$') && ~strcmp(fileline(1),'[')
            % Searches for the "=" symbol
            index1 = find(fileline=='=',1);
            if ~isempty(index1)
                % Coefficient name
                varname = strtrim(fileline(1:index1-1));
                % Searches for "$" symbol
                index2 = find(fileline=='$',1);
                if ~isempty(index2)
                    % Coefficient value
                    varval = strtrim(fileline(index1+1:index2-1));
                else
                    % Coefficient value
                    varval = strtrim(fileline(index1+1:end));
                end
                % Checks if the value is a number or a string
                if isempty(find(varval=='''',1))
                    varval = str2double(varval);
                end
                % Creates the tyre structure field
                tyre.(varname) = varval;
                % Get the next file line
                fileline = fgets(fileid);
            else
                % Get the next file line
                fileline = fgets(fileid);
            end
        else
            % Get the next file line
            fileline = fgets(fileid);
        end
    else
        % Get the next file line
        fileline = fgets(fileid);
    end
end

% Closes the *.tir file
fclose(fileid);

end