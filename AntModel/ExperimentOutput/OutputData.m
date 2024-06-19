classdef OutputData
    %OUTPUTDATA Operating class that is used to save and store measured
    %variables from the Ant Model for an experiment across multiple trials.
    %   Used to save results to .csv or .mat files with locations defined
    %   in the RUNTIME_ARGS.
    %   ChangeLog: 22/11/22 - Emily Rolley-Parnell - Added the ability to
    %   print a cost sheet

    properties

        script_folder

        printout_path
        mat_path

        printout_format

        RUNTIME_ARGS

        contact_data
        sense_goal_data
        replay_data

        experiment_seed
       

        costClass


    end

    methods
        function obj = OutputData(RUNTIME_ARGS)
            %OUTPUTDATA Creates an instance for a given experiment and
            %creates the folders to contain the printed data, based on the
            %file location provided in RUNTIME_ARGS
            day = datestr(now, 'dd-mm-yy_HH-MM');
            caller = dbstack;
            modelDir = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel';
            obj.script_folder = [modelDir, '\ExperimentOutput\', caller(end).name,'\', RUNTIME_ARGS.TRIAL_NAME,'_', day];

            if RUNTIME_ARGS.RECORD.ENABLE
                obj.mat_path = [obj.script_folder, '\',RUNTIME_ARGS.RECORD.FOLDER];
                mkdir(obj.mat_path)

            else
                obj.mat_path = [];
            end

            if RUNTIME_ARGS.PRINTOUT.ENABLE

                obj.printout_format = RUNTIME_ARGS.PRINTOUT.FORMAT;
                obj.printout_path = [obj.script_folder, '\', RUNTIME_ARGS.PRINTOUT.FOLDER];
                mkdir(obj.printout_path)
            else
                obj.printout_format = [];
                obj.printout_path = [];
            end


            obj.RUNTIME_ARGS = RUNTIME_ARGS;
            obj.contact_data = {};
            obj.sense_goal_data = {};
            obj.replay_data = {};
            obj.costClass = CostCalculator();


            obj.experiment_seed = RandStream.getGlobalStream;

        end

        function obj = saveTrial(obj, trial_number, antTree, tocTime)
            %saveTrialMAT Writes the data collected from the past trial to
            %a MATLAB MAT file
            % Inputs:
            % tocTime: Real world time taken from start to end of trial
            
            %Compile the sensory data
            if isempty(obj.contact_data)
                contactsTable = table.empty;
            else
                contactsTable = cell2table(vertcat(obj.contact_data{:}));
                contactsTable.Properties.VariableNames =  {'Time', 'Contact Location', 'Surface Normal', 'Contact Limb'};
            end

            %Compile the pose for the whole trial
            replayTable = cell2table(vertcat(obj.replay_data{:}));
            replayTable.Properties.VariableNames = {'Time', 'Pose', 'Position'};

            
            %Compile the different goals evaluated during the trial
            senseGoalTable = vertcat(obj.sense_goal_data{:});      

            obj.costClass = obj.costClass.calculateMotionCost(replayTable);

            %Make a copy of the cost summary table
            costSummaryTable = obj.costClass.convertToTable(tocTime);

            %Rename the runtime args for no particular reason
            setupRuntimeArgs = obj.RUNTIME_ARGS;

            %Save the random seed used in this experiment
            randomSeedCopy = obj.experiment_seed;

            %If you provided a destination then save a mat file for every trial with all the set up
            %details
            if ~isempty(obj.mat_path)
                save([obj.mat_path, '\trial_',int2str(trial_number), '.mat'], 'contactsTable', 'senseGoalTable', 'replayTable', 'costSummaryTable', 'antTree', 'setupRuntimeArgs', "randomSeedCopy")
            end

            % If you want the experiment saved in a written format
            if ~isempty(obj.printout_path)

                filename = [obj.printout_path, '\trial_',int2str(trial_number)];

                switch obj.printout_format
                    case "spreadsheet"
                        WriteMode = 'overwritesheet';
                    otherwise
                        WriteMode = 'overwrite';
                end

                writetable(contactsTable, filename, 'FileType', obj.printout_format, 'Sheet', 'Contact Points', 'WriteMode', WriteMode)
                writetable(senseGoalTable, filename, 'FileType', obj.printout_format, 'Sheet', 'Sensed Goal', 'WriteMode', WriteMode)
                writetable(replayTable, filename, 'FileType', obj.printout_format, 'Sheet', 'Model Pose and Positions', 'WriteMode', WriteMode)
                writetable(costSummaryTable, filename, 'FileType', obj.printout_format, 'Sheet', 'Trial Costs', 'WriteMode', WriteMode)
            end

            obj.sense_goal_data = {};
            obj.replay_data = {};
            obj.contact_data = {};
         
            obj.costClass = CostCalculator();

        end



        function obj = addTimeStep(obj, time, contactStructArray, pose, position, goalObj, costStruct)

            obj = obj.addPose(time, pose, position);
            obj = obj.addSensedData(time, contactStructArray);
            obj = obj.addGoal(time, goalObj);
            obj.costClass = obj.costClass.addCost(costStruct);
        end




        function obj = addSensedData(obj, time, contactStructArray)

            if ~isempty(contactStructArray)
                for i = 1:size(contactStructArray,1)
                    obj.contact_data{end+1} = {time, contactStructArray(i).point, contactStructArray(i).normal, contactStructArray(i).limb};
                end

            end
        end

        function obj = addPose(obj, time, pose, position)

            obj.replay_data{end+1} = {time, pose, position};

        end

        function obj = addGoal(obj, currentTime, goalObj)
            if ~goalObj.isempty()
                goalObj = goalObj.settime(currentTime);
                obj.sense_goal_data{end+1} = goalObj.convert2table();
            end
        end


    end
end


