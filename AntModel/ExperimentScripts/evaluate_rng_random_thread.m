%Close any figures and clear all variables
close all;
clear;

rng('shuffle')
r1 = rng

%Disable any printed warnings for the parallel pool
delete(gcp('nocreate'))
p = parpool();
parfevalOnAll(p,@warning, 0,'off');
opts = parforOptions(p);

samples = 5
r1_out = nan(samples,1);
r1_rng = cell(samples,1);
parfor (n = 1:samples, opts)
    r1_rng{n} = rng
    r1_out(n) = rand(1,1);
end

r1_out
%%
%Close any figures and clear all variables
close all;
clear;

rng('shuffle')
r2 = rng

%Disable any printed warnings for the parallel pool
delete(gcp('nocreate'))
p = parpool();
parfevalOnAll(p,@warning, 0,'off');
opts = parforOptions(p);

samples = 5
r2_out = nan(samples,1);
parfor (n = 1:samples, opts)
    r2_out(n) = rand(1,1);
end

r2_out

%% Adding randomness per stream

close all;
clear;


defaultStream = RandStream('mrg32k3a', 'Seed', 'shuffle');

%Disable any printed warnings for the parallel pool
delete(gcp('nocreate'))
p = parpool();
parfevalOnAll(p,@warning, 0,'off');
opts = parforOptions(p);

samples = 20
r3_out = nan(samples,5);
parfor (n = 1:samples, opts)
    set(defaultStream, 'Substream', n);
    RandStream.setGlobalStream(defaultStream)
    r3_out(n,:) = rand(1,5);
end

r3_out

defaultStream = RandStream('mrg32k3a', 'Seed', 'shuffle');
r4_out = nan(samples,5);
parfor (n = 1:samples, opts)
    set(defaultStream, 'Substream', n);
    RandStream.setGlobalStream(defaultStream)
    r4_out(n,:) = rand(1,5);
end

r4_out

%% Checking that randomness can be disabled

close all;
clear;


defaultStream = RandStream('mrg32k3a', 'Seed', 'shuffle');

%Disable any printed warnings for the parallel pool
delete(gcp('nocreate'))
p = parpool();
parfevalOnAll(p,@warning, 0,'off');
opts = parforOptions(p);

samples = 20
r3_out = nan(samples,5);
parfor (n = 1:samples, opts)
%     set(defaultStream, 'Substream', n);
%     RandStream.setGlobalStream(defaultStream)
    r3_rng(n) = rng
    r3_out(n,:) = rand(1,5);
end

r3_out

% %Disable any printed warnings for the parallel pool
% delete(gcp('nocreate'))
% p = parpool();
% parfevalOnAll(p,@warning, 0,'off');
% opts = parforOptions(p);

defaultStream = RandStream('mrg32k3a', 'Seed', 'shuffle');

r4_out = nan(samples,5);
parfor (n = 1:samples, opts)
    set(defaultStream, 'Substream', n);
    RandStream.setGlobalStream(defaultStream)
    r4_rng(n) = rng
    r4_out(n,:) = rand(1,5);
end

r4_out