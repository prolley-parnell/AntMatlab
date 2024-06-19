%% Overhead script to run all experiments for PCA goal generation with all joint controlled methods
% Fixed variance
% 3-30 contacts
% Scripts contained
%     Grasp Mode            Joint Control Mode
%     -----------           ---------------------
% 1.  PCA                   Random Reach and Pull
% 2.  PCA                   Contact Mean Centred



%Warning: current folder must be the one that contains the overhead script

%1
run('.\PCA_RRaP.m')

%2
run('.\PCA_CMC_fixvar.m')


%3
%run('.\PCA_CMC_varinc.m')


%4
%run('.\PCA_CMC_vardec.m')

cd('C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentScripts')
