function build_mex()
% BUILD_MEX
%
% This script builds the C++ PID controller into a MATLAB MEX file.
% After running this, MATLAB will have a binary called:
%
%     pid_mex.<mexext>
%
% sitting in the matlab/ folder, and all the demos and Simulink models
% can call it just like a normal MATLAB function.
%
% Typical usage:
%   cd matlab
%   build_mex

    % -------------------------------------------------------------
    % Figure out where we are in the repo
    % -------------------------------------------------------------
    %
    % mfilename('fullpath') gives the full path of THIS file
    % even if the user ran it from some other folder.
    % We use that to make the script robust.

    thisFile  = mfilename('fullpath');   % .../repo/matlab/build_mex.m
    matlabDir = fileparts(thisFile);     % .../repo/matlab
    repoRoot  = fileparts(matlabDir);    % .../repo

    % -------------------------------------------------------------
    % Define where the C++ source lives and where output goes
    % -------------------------------------------------------------

    srcDir = fullfile(repoRoot, "src");     % folder with pid_mex.cpp, PidController.cpp
    outDir = matlabDir;                    % put compiled mex here so MATLAB finds it

    % Final MEX filename (platform dependent: mexw64, mexmaci64, etc)
    mexFile = fullfile(outDir, "pid_mex." + mexext);

    % -------------------------------------------------------------
    % List of C++ files to compile
    % -------------------------------------------------------------
    %
    % These get compiled and linked together into one MEX binary.

    cppFiles = {
        fullfile(srcDir, "pid_mex.cpp")
        fullfile(srcDir, "PidController.cpp")
    };

    % -------------------------------------------------------------
    % Include paths for the compiler
    % -------------------------------------------------------------
    %
    % -I tells the compiler where to find header files.
    % Here it lets pid_mex.cpp include "PidController.hpp"

    includeFlags = {
        "-I" + srcDir
    };

    % -------------------------------------------------------------
    % Common MEX flags
    % -------------------------------------------------------------
    %
    % -R2018a  -> use the newer MEX API (mxArray handling etc)
    % -v       -> verbose, so we can see compiler commands
    % -outdir  -> where the final .mex* file is written

    commonFlags = {
        "-R2018a"
        "-v"
        "-outdir"
        outDir
    };

    fprintf("Building MEX to: %s\n", mexFile);

    % -------------------------------------------------------------
    % Run the actual build
    % -------------------------------------------------------------
    %
    % mex() is MATLAB's C++ compiler driver.
    % It will call MSVC / clang / gcc underneath depending on the platform.
    %
    % If something goes wrong, the catch block will print a helpful hint
    % and then rethrow the real error so you still see full details.

    try
        mex(commonFlags{:}, includeFlags{:}, cppFiles{:});
    catch ME
        fprintf(2, "\nMEX build failed.\n");
        fprintf(2, "Most common fix: run `mex -setup C++` and choose a supported compiler.\n\n");
        rethrow(ME);
    end

    % If we get here, compilation and linking succeeded
    fprintf("Build succeeded: %s\n", mexFile);
end