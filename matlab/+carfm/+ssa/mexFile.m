function [flagerr, errid, errmsg] = mexFile(filename, varargin)
%MEXFILE Generate MEX file.
%
% INPUT
% filename: name of the MEX file (w/o extension)
% varargin{:}: functions to add to the MEX file
% 
% OUTPUT
% flagerr: true in the case of error
% errid: error id
% errmsg: error message

% Checks args
arguments
    filename (1,:) char
end
arguments (Repeating)
    varargin casadi.Function
end

% Temp dir for the system and temp name for the c file
% using tmpDir and tmpName to obfuscate source files to user
[tmpDir, tmpName] = fileparts(tempname);
tmpDir = [tmpDir filesep]; % add filesep (/ or \)

% Generate code
fprintf("Generating C code.\n")
cfilename = [tmpName '.c'];
cg = casadi.CodeGenerator(cfilename, struct('mex', true));

% Append all provided functions to cg
for k = 1 : numel(varargin)
    cg.add(varargin{k});
end
cg.generate(tmpDir); % genenerate .c file into tmpDir
fprintf("Done.\n")
pause(0) % just to print out all fprintf

% Check compiler
compilerConfig = mex.getCompilerConfigurations('C', 'Selected');
if isempty(compilerConfig)
    files_to_rem = dir([tmpDir tmpName '.*']);
    for k = 1 : numel(files_to_rem)
        delete([files_to_rem(k).folder filesep files_to_rem(k).name])
    end
    errid = 'carfm:unableMexFile';
    errmsg = 'No compiler configured for MEX. Configure a compiler using ''mex -setup''.';
    flagerr = true;
    return;
end
% Set OPTIMFLAGS
optimflag = '';
if contains(compilerConfig.ShortName, 'mingw','IgnoreCase',true)
    optimflag = sprintf('COPTIMFLAGS=%s', '-O1 -fwrapv -DNDEBUG'); % MinGW optimization
elseif contains(compilerConfig.ShortName, 'msvc','IgnoreCase',true)
    optimflag = sprintf('OPTIMFLAGS=%s', '/O1 /Oy- /DNDEBUG'); % MSVC optimization
end
% Compile mex
fprintf("Compiling MEX file.\n")
pause(0) % just to print out all fprintf
clear(filename); % clear mex file
try
    % Mex into current dir with output name=filename
    mex([tmpDir cfilename],'-outdir', './', '-output', filename, optimflag);
catch err
    files_to_rem = dir([tmpDir tmpName '.*']);
    for k = 1 : numel(files_to_rem)
        delete([files_to_rem(k).folder filesep files_to_rem(k).name])
    end
    errid = 'carfm:unableMexFile';
    errmsg = err.message;
    flagerr = true;
    return
end

% Clean tmp dir
files_to_rem = dir([tmpDir tmpName '.*']);
for k = 1 : numel(files_to_rem)
    delete([files_to_rem(k).folder filesep files_to_rem(k).name])
end

% Done
pause(0) % just to print out all fprintf

% Return
flagerr = false;
errid = '';
errmsg = '';

end