function [exitFlag, fileHandler] = AntModelFunction(RUNTIME_ARGS)
%ANTMODELFUNCTION Runs a number of experiments defined by the parameters in
%RuntimeArgs

exitFlag = 0;
%Grasp complete if exit flag = 1

if or(RUNTIME_ARGS.RECORD.ENABLE, RUNTIME_ARGS.PRINTOUT.ENABLE)
    fileHandler = OutputData(RUNTIME_ARGS);
    fileHandlerIsEmpty = 0;
else
    fileHandlerIsEmpty = 1;
    fileHandler = OutputData.empty;
end



for n = 1:RUNTIME_ARGS.N_TRIALS

    close all
    if RUNTIME_ARGS.TERMPRINT
        disp("Starting Trial %d", n)
    end

    RUNTIME_ARGS.initiatePlots();

    %Reset the trial time
    trial_time = 0;
    tStart = tic;

    %Define Ant object
    ant = Ant(RUNTIME_ARGS);



    %Define the collision objects in the model environment
    env = CollisionObjects(RUNTIME_ARGS);


    while ~ant.grasp_complete

        stepStart = tic;

        [ant, sensedDataArray, goalObj, costStruct] = ant.update(env);
        if ant.grasp_complete == 1
            exitFlag = 1;
        end


        % TODO Add a pause point in case the user interacts with a graph
        if any(RUNTIME_ARGS.PLOT.ENABLE)
            endtime = toc(stepStart);
            diff = endtime - (RUNTIME_ARGS.RATE*2);
            pause(abs(min(diff, 0)));
        end

        if ~fileHandlerIsEmpty
            fileHandler = fileHandler.addTimeStep(trial_time, sensedDataArray, ant.q, ant.position, goalObj, costStruct);
        end

        trial_time = trial_time + RUNTIME_ARGS.RATE;

        if toc(tStart) > RUNTIME_ARGS.TIMEOUT
            break
        end

    end

    if ~fileHandlerIsEmpty
        fileHandler = fileHandler.saveTrial(n, ant.antTree, toc(tStart));
    end



end







end

