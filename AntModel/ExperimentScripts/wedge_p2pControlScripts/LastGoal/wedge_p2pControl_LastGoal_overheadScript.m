%% Overhead script to run all experiments for PCA goal generation with all p2p controlled methods
% Fixed variance
% 3-30 contacts
% Scripts contained
%     Grasp Mode            Position Control Mode
%     -----------           ---------------------
% 1.  Last                   Random Search Space
% 2.  Last                   Contact GMM Var Static
% 3.  Last                   Closing Search




%Warning: current folder must be the one that contains the overhead script


%1
run('.\Last_RSS.m')

%2
run('.\Last_CGMMC_fixvar.m')


%3
run('.\Last_Closing.m')



cd('C:\Users\eroll\Documents\MATLAB\Model\AntMatlab\AntModel\ExperimentScripts')
