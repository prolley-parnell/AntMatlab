%% Overhead script to run all experiments for PCA goal generation with all p2p controlled methods
% Fixed variance
% 3-30 contacts
% Scripts contained
%     Grasp Mode            Tip Control Mode
%     -----------           ---------------------
% 1.  PCA                   Random Search Space
% 2.  PCA                   Contact GMM Var Static
% 3.  PCA                   Closing




%Warning: current folder must be the one that contains the overhead script


%1
run('.\PCA_RSS.m')

%2
run('.\PCA_CGMMC_fixvar.m')

%3
run('.\PCA_Close.m')




cd('C:\Users\eroll\Documents\MATLAB\Model\AntMatlab\AntModel\ExperimentScripts')
