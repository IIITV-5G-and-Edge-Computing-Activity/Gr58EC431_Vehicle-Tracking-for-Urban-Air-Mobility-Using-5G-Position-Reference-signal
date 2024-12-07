classdef Helper5GTrackingExampleDisplay < matlab.System
    % This is a helper class to visualize the scenario and results of the
    % example demonstrating tracking of UAVs using 5G Position Reference
    % Signals.
    %
    % It may be removed or modified in a future release.

    % Copyright 2023 The MathWorks, Inc.

    properties (Nontunable)
        ColorTheme {mustBeMember(ColorTheme,{'light','dark','default'})} = 'default';
        PlotTDOAHyperbola (1,1) logical = true;
    end

    properties (Access = protected)
        TopDisplay
        BottomDisplays
    end

    methods
        function obj = Helper5GTrackingExampleDisplay(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj, scene)
            f = figure('Units','normalized','Position',[0.05 0.05 0.85 0.85]);
            clrs = lines(7);
            if strcmpi(obj.ColorTheme,'dark') || clrs(1,1) ~= 0
                f.Color = [0.3 0.3 0.3];
            end

            p2 = uipanel(f,'Units','normalized','Position',[0 0.5 1 0.5]);
            obj.TopDisplay = Helper5GTrackingDisplayComponent('PlotTDOAHyperbola',obj.PlotTDOAHyperbola,...
                'ColorTheme',obj.ColorTheme,....
                'Parent',p2,...
                'ShowLegend',true,...
                'XLimits',[1400 2100],...
                'YLimits',[-50 250],...
                'ZLimits',[-10 350],...
                'ViewAngles',[0 90]);
            
            uavLogArray = strncmp([scene.Platforms.Name],'UAV',3);
            uavIndices = find(uavLogArray);
            numUAVs = sum(uavLogArray);
            bottomDisplays = cell(numUAVs,1);
            for i = 1:numUAVs
                 p = uipanel(f,'Units','normalized',...
                     'Position',[(i-1)/numUAVs 0 1/numUAVs 0.5],...
                     'TitlePosition','centertop','Title',scene.Platforms(uavIndices(i)).Name);
                 bottomDisplays{i} = Helper5GTrackingDisplayComponent('PlotTDOAHyperbola',false,...
                    'ColorTheme',obj.ColorTheme,....
                    'Parent',p,...
                    'ZoomOnPlatformIndex',i,...
                    'ShowLegend',false,...
                    'XLimits',[1600 2000],...
                    'YLimits',[-100 300],...
                    'ZLimits',[0 100],...
                    'ViewAngles',[45 45]);
            end
            obj.BottomDisplays = bottomDisplays;
        end

        function stepImpl(obj, varargin)
            obj.TopDisplay(varargin{:});
            for i = 1:numel(obj.BottomDisplays)
                obj.BottomDisplays{i}(varargin{:});
            end
            drawnow;
        end
    end

    methods
        function plotScenario(obj, scene)
            display = Helper5GTrackingDisplayComponent('PlotTDOAHyperbola',false,...
                'ColorTheme',obj.ColorTheme,....
                'ShowLegend',false,...
                'XLimits','auto',...
                'YLimits','auto',...
                'ZLimits',[0 250],...
                'ViewAngles',[0,90]);
            setup(display,scene,[],[],[]);
        end
    end
end