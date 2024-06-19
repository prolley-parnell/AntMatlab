%% Overhead script to run all experiments for direct contact point goal generation with all point to point control methods
% Fixed variance
% 2-40 contacts
% Scripts contained
%     Grasp Mode            Point Control Mode
%     -----------           ---------------------
% 1.  IPD                   Random Search Space
% 2.  IPD                   Contact GMM Centred
% 3.  IPD                   Closing
% 4.  Align                 Random Search Space
% 5.  Align                 Contact GMM Centred
% 6.  Align                 Closing
% 7.  IPD&Align             Random Search Space
% 8.  IPD&Align             Contact GMM Centred
% 9.  IPD&Align             Closing

%Warning: current folder must be the one that contains the overhead script

%1
run('.\IPD_RSS.m')

%2
run('.\IPD_CGMMC_fixvar.m')

%3
run('.\IPD_Close.m')

%4
run('.\Align_RSS.m')

%5
run('.\Align_CGMMC_fixvar.m')

%6
run('.\Align_Close.m')

%7
run('.\IPDAlign_RSS.m')

%8
run('.\IPDAlign_CGMMC_fixvar.m')

%9
run('.\IPDAlign_Close.m')

%%6.1
%%run('.\IPDAlign_CGMMC_vardec.m')

%%6.2
%%run('.\IPDAlign_CGMMC_varinc.m')

cd('C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentScripts')