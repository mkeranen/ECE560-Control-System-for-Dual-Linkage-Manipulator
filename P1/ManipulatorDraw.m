classdef ManipulatorDraw < handle
% ManipulatorDraw
%
% Created by S. Baek on 2014-10-07
% Bioinspiration and Intelligent Robotics Lab
% University of Michigan - Dearborn
%
% Last updated: 2018-10-17 by S. Baek
%   - Made a few changes for ECE560
%
% Copyright (c) University of Michigan.
% 
% DO NOT DISTRIBUTE THIS CODE WITHOUT PERMISSION OF THE UNIVERSITY OF MICHIGAN.
%
% THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, 
% EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED 
% WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

%
%           o m2  
%            \ 
%          l2 \  theta2  
%              \   /
%               \ / 
%                o m1
%               /  
%          l1  /  
%             /  
%            /  ) theta1
%      -----------------

    properties(GetAccess = 'public', SetAccess = 'private')
    % public read access, but private write access.
    
        % manipulator parameters
        m1 = 1;
        m2 = 1;
        l1 = 1;
        l2 = 1;
        g = 9.8;
        
    end

    properties(GetAccess = 'private', SetAccess = 'private')
    % private read and write access
    
        Paused = false;      % Is simulation pause? need to change the name to IsPause
        Terminated = false;
        
        PlotRecorder;
        
        SimulationRefreshSpeed;       
        % pause simulation
        PauseButton;    
        
        % To store video frames
        FrameQueue;
        
        FigureHandle;
        LineHandle;
        AnimationAxes;
        AxesRange = [-2.2, 2.2, -2.2, 2.2];
        
    end
            
    methods % public methods   
        
        function self = ManipulatorDraw(name, simulationRefreshSpeed)% , gcf)
            figure('name', name);
            self.SimulationRefreshSpeed = simulationRefreshSpeed;
            self.FigureHandle = gcf;
            self.AnimationAxes = self.DrawMissionMap(self.FigureHandle);
        end

        function plotaxes = GetAnimationAxes(self)
            plotaxes = self.AnimationAxes; 
        end
        
        function SetAxesRange(self, axes_range)
            self.AxesRange = axes_range; 
        end
        
        function EnablePlotRecoder(self)
            
            saveLocation = strcat('./save_', datestr(now,'yyyy-mm-dd-HH-MM-SS'));
            mkdir(saveLocation);
            
            videoFileName = strcat(saveLocation, '/VideoOutput.avi');
            self.PlotRecorder = VideoWriter(videoFileName); % Name it.
            self.PlotRecorder.FrameRate = self.SimulationRefreshSpeed; % How many frames per second.
            open(self.PlotRecorder); 
            self.FrameQueue = Queue();
            
        end
            
        function DisablePlotRecoder(self)
            self.PlotRecorder = []; 
            self.FrameQueue = Queue(); % empty out.
        end
        
        function RecordFrame(self)            
            assert(~isempty(self.PlotRecorder), 'Plot recoder has not been enabled.');
            frame = getframe(self.FigureHandle); 
            self.FrameQueue.Push(frame);
        end
        
        function Close(self)
            self.Terminate()
        end
           
        function bool = IsPaused(self)
            bool = self.Paused;            
        end

        function bool = IsRunning(self)
            bool = ~self.Terminated;            
        end                
        
        function Draw(self, theta1, theta2, time)
          
            simulationTimeStep = 1/self.SimulationRefreshSpeed;
            index = 1;
            
            while true
                
                tic;
                if (index > length(time))
                    return;
                end
                
                if (~self.IsRunning())
                    self.Close();                    
                    break;
                end
                
                if (self.IsPaused())
                    pause(0.5);
                    continue;
                end
                             
                self.DrawOneFrame(theta1(index), theta2(index), time(index));
                
                elapsedTime = toc;
                if (elapsedTime > simulationTimeStep)
                    fprintf('Warning: computational time %f is greater than simulation period of %f\n', ...
                            elapsedTime, simulationTimeStep);
                end
                pause(simulationTimeStep - elapsedTime);
                
                index = index + 1;
            end
         
        end % end of function
                
    end % end of public methods
        
    
    %% Private methods
    methods (Access = private)   
        
        function DrawOneFrame(self, theta1, theta2, time)
   
            delete(self.LineHandle); % remove lines from the privous frame

            x = [0 self.l1*cos(theta1)];
            y = [0 self.l1*sin(theta1)];
            self.LineHandle(1) = line(x,y,'Color','red','LineWidth',3);

            x = x + [self.l1*cos(theta1) self.l2*cos(theta1 + theta2)];
            y = y + [self.l1*sin(theta1) self.l2*sin(theta1 + theta2)];
            self.LineHandle(2) = line(x, y, 'Color', 'blue', 'LineWidth', 3);
            
            textposx = 0.3;
            textposy = 2.0;
            theta1 = mod(theta1*180/pi, 360);
            theta2 = mod(theta2*180/pi, 360);
            self.LineHandle(4) = text(textposx, textposy, ...
                sprintf('t: %2.1f s, %s: %3.1f deg, %s: %3.1f deg', time, '\theta_1', theta1, '\theta_2', theta2));
         
            drawnow;
            
            if ~isempty(self.PlotRecorder)
                self.RecordFrame();
            end
        end
        
        
        function PauseSimulation(self, varargin)        
            disp 'Simulation Paused. Press Resume to resume'
            
            if (self.Paused)
                set(self.PauseButton, 'String', 'Pause');
                self.Paused = false;
            else
                set(self.PauseButton, 'String', 'Resume');
                self.Paused = true;
            end
        end
                
        function StopSimulation(self, varargin)        
            self.Terminated = true;
        end
        
        function Terminate(self)
            
            disp 'Terminating Manipulator Simulation....'
            
            if isempty(self.PlotRecorder)
                close(self.FigureHandle);
                return;
            end
            
            % pause for 0.1 second to keep it from VideoWriter crashing. 
            % This is a workaround for the error, 'OBJ must be open before 
            % writing video.  Call open(obj) before calling writeVideo.'
            pause(0.1); 
            
            % TODO: waitbar is not working.
            % waitbarHandle = waitbar(0, '1', 'Name', 'Creating Video...', ...
            %                         'CreateCancelBtn',...
            %                         'setappdata(gcbf,''canceling'',1)');
            % setappdata(waitbarHandle,'canceling',0);

            % totalFrameSize = self.FrameQueue.Size;
            % currentFrameIndex = 1;
            % set(0, 'CurrentFigure', waitbarHandle); 
            while (self.FrameQueue.Size > 0)
                frame = self.FrameQueue.GetFirst();
                writeVideo(self.PlotRecorder, frame);
            %    if getappdata(waitbarHandle,'canceling')
            %        break
            %    end
            %    waitbar(currentFrameIndex / totalFrameSize); %, ...
                        %sprintf('%d frame out of %d frames have been exported', currentFrameIndex, totalFrameSize));
            %    currentFrameIndex = currentFrameIndex + 1;    
            end
            
            %pause(1);
            
            % close(waitbarHandle);
            % close(self.FigureHandle);
            close(self.PlotRecorder);
        end
                
        
        function missionMapAxes = DrawMissionMap(self, figureHandler)
        % DrawMissionPlan sets up a figure with plots for mission map. 
       
            % figure size
            xmin = 100;
            ymin = 100;
            width = 600;
            height = 600;
            
            figurePosition = [xmin, ymin, width, height];
            set(figureHandler, 'Position', figurePosition); 
                
            %% Set the plot area
            xmin = 0.1;
            ymin = 0.1;
            height = 0.8;
            width = 0.8;                

            plotPosition = [xmin, ymin, width, height];            
            missionMapAxes = subplot('Position', plotPosition);  % plot to draw uav mission
            set(gcf, 'currentaxes', missionMapAxes);            
            axis(self.AxesRange);
            grid on;
            
            %% Draw user interface
            uicStopSimulation = uicontrol('Style', 'pushbutton', 'String', 'Exit', ...
                                         'Position', [20 15 60 20], ...
                                        'Callback', @self.StopSimulation);
                                    
            self.PauseButton = uicontrol('Style', 'pushbutton', 'String', 'Pause', ...
                                         'Position', [100 15 60 20], ...
                                         'Callback', @self.PauseSimulation);                        

        end
                
    end
            
    methods (Static, Access = public)
        
        function RunSimpleTest()           
        
            %% Plot Preparation            
            simulationRefreshSpeed = 10;
            animation = ManipulatorDraw('Robotic Arm', simulationRefreshSpeed);
            animation.EnablePlotRecoder();            
            
            %% Run Test
            t = linspace(0, 4, 400);
            theta1 = 0.5*sin(2*pi*t);
            theta2 = sin(pi*t);
                             
            animation.Draw(theta1, theta2, t);
           
            % close...
            animation.Close();
        end
        
        
    end
end












        