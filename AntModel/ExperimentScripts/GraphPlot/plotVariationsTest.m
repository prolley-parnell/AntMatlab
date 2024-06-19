%% File to test different plotting styles for experiment results to see if there are clearer methods
% % Created 28/04/2023
% ASSUMES loading GPC_4shape_d_p_w_gs.mat

load('GPC_4shape_d_p_w_gs.mat')
%% For one experiment, plot the percent success against the number of contacts

experimentNameExample = "IPDAlign_CMC_fixvar";

GPC_grass_seed.plotProportionalKnee(experimentNameExample)


%% 
 
