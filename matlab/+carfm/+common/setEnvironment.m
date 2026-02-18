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

persistent oldMaxNumDir; % casadi global option setMaxNumDir

% Expiration date and license numbers
expiration_date = datetime(2026,09,30,23,59,59);
licenses = {'40523914', '41067317', '41252325', '41252326', '41252328'};

% Check expiration date
today = datetime;
if today > expiration_date
    flagerr = true;
    expiration_date.Format = 'yyyy-MM-dd';
    errid = 'carfm:expiredLicense';
    errmsg = ['Expired license: expiration date is ' char(expiration_date)];
    return;
end

% Check license number
lic = license;
if ~any(strcmp(lic,licenses))
    flagerr = true;
    errid = 'carfm:invalidLicense';
    errmsg = ['Invalid license number: current license number is ' lic];
    return;
end

if doSet
    % Set env
    addpath([filepath filesep 'external']) 
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
    % Reset env
    rmpath([filepath filesep 'external']);
end

% Return
flagerr = false;
errid = '';
errmsg = '';

end