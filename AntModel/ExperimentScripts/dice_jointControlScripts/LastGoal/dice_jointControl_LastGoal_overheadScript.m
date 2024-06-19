%% Overhead script to run all experiments for last contacts goal generation with all joint controlled methods
% Fixed variance
% 3-30 contacts
% Scripts contained
%     Grasp Mode            Joint Control Mode
%     -----------           ---------------------
% 1.  Last                   Random Reach and Pull
% 2.  Last                   Contact Mean Centred




%Warning: current folder must be the one that contains the overhead script

%1
run('.\Last_RRaP.m')

%2
run('.\Last_CMC_fixvar.m')

cd('C:\Users\eroll\Documents\MATLAB\Model\AntMatlab\AntModel\ExperimentScripts')
