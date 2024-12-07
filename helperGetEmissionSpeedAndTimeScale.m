function parameters = helperGetEmissionSpeedAndTimeScale()
% This is a helper function and may be removed or modified in a future
% release. 
%
% This function defines the emissions speed and units for time used in the
% 5G tracking example. Modify these values to use a different emission
% speed and time units. 

% Copyright 2023 The MathWorks, Inc. 

parameters.EmissionSpeed = 299792458.0;
parameters.TimeScale  = 1e9;
end