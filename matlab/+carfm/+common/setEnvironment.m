function [flagerr, errid, errmsg] = setEnvironment(filepath, doSet)
%SETENVIRONMENT Set the environment by adding path.
%
% INPUT:
% filepath: main directory
% doSet: set (true) / unset (false) path (default true=set path)
%
% OUTPUT:
% flagerr: true in the case of error
% errid: error id
% errmsg: error message

% Checks args
arguments
    filepath (1,:) char = './';
    doSet (1,1) logical = true
end

persistent casadi_case; %[]: not found, 1: MEX found, 2: local MEX found, 3: global MEX found
persistent oldMaxNumDir; % casadi global option setMaxNumDir

% Check expiration date
today = datetime;
expdate = datetime(2026,09,30,23,59,59);
if today > expdate
    flagerr = true;
    expdate.Format = 'yyyy-MM-dd';
    errid = 'fmtools:expiredLicense';
    errmsg = ['Expired license: expiration date is ' char(expdate)];
    return;
end

% Check license number
lic = license;
lics = {'40523914', '40765951', '41067317', '41252325', '41252326', '41252328'};
if ~any(strcmp(lic,lics))
    flagerr = true;
    errid = 'fmtools:invalidLicense';
    errmsg = ['Invalid license number: current license number is ' lic];
    return;
end

% Check pc or mac
casadipath = 'casadi-3.6.7';
% if ispc
%     casadipath = [casadipath '-pc'];
% elseif ismac
%     casadipath = [casadipath '-mac'];
% end

% Check casadi
if isempty(casadi_case)
    if isfile([filepath filesep casadipath filesep 'casadiMEX.' mexext]) % search in package folder
        casadi_case = 1;
    elseif isfile([casadipath filesep 'casadiMEX.' mexext]) % search in local folder
        casadi_case = 2;
    elseif exist('casadiMEX', 'file') == 3 % search in search path
        casadi_case = 3;
    else
        casadi_case = [];
    end
end

% Give error if no casadi is found
if isempty(casadi_case)
    errid = 'fmtools:casadiNotFound';
    errmsg = 'CasADi not found in local folder or search path. Get CASADI from <a href = "https://web.casadi.org/">CASADI Web Site</a>.';
    flagerr = true;
    return
end

if doSet
    % Set env
    switch casadi_case
        case 1 % in package folder
            addpath([filepath filesep casadipath]);
        case 2 % in local folder
            addpath(casadipath);
        % case 3: no need to addpath casadi b/c already in path
    end
    % Check if casadi working
    try
        casadi.MX.sym('x');
    catch err
        errid = 'carfm:casadiNotWorking';
        errmsg = err.message;
        flagerr = true;
        return 
    end
    % CasADi general option
    oldMaxNumDir = carfm.common.setMaxNumDir(64); % 64 is CasADi default
else
    % Reset general options
    if (oldMaxNumDir) 
        carfm.common.setMaxNumDir(oldMaxNumDir);
    end
    % Unset env
    switch casadi_case
        case 1
            rmpath([filepath filesep casadipath]);
        case 2
            rmpath(casadipath);
    end
end

% Return
flagerr = false;
errid = '';
errmsg = '';

end