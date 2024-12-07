classdef Helper5GTrackingDisplayComponent < matlab.System
    % This is a helper class to visualize the scenario and results of the
    % example demonstrating tracking of UAVs using 5G Position Reference
    % Signals.
    %
    % It may be removed or modified in a future release.

    % Copyright 2023 The MathWorks, Inc.

    % Public properties
    properties
        Parent
        ColorTheme {mustBeMember(ColorTheme,{'light','dark','default'})} = 'default';
        PlotTDOAHyperbola (1,1) logical = true;
        ShowLegend (1,1) logical = false;
        XLimits = 'auto';
        YLimits = 'auto';
        ZLimits = 'auto';
        PlotTrackCovariance (1,1) logical = true;
        PlotDetectionCovariance (1,1) logical = true;
        ViewAngles = [0 0];
        ZoomOnPlatformIndex 
    end

    properties (Access = protected)
        Axes
        TrackPlotter
        PositionDetectionPlotter
        TDOADetectionPlotter
    end

    methods
        function obj = Helper5GTrackingDisplayComponent(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj, scene)
            % Create UAV scenario display
            if isempty(obj.Parent)
                obj.Parent = figure('Visible','on','Units','normalized','Position',[0.05 0.05 0.85 0.85]);
            end

            ax = axes(obj.Parent);

            % Color order
            switch obj.ColorTheme
                case {'light','default'}
                    clrs = lines(7);
                case 'dark'
                    ax.Color = [0 0 0];
                    clrs = darkColorOrder();
            end
            if clrs(1,1) == 0
                meshColor = 0.3*[1 1 1];
                parentColor = 0.94*[1 1 1];
            else
                meshColor = 0.7*[1 1 1];
                parentColor = 0.1*[1 1 1];
                ax.XAxis.Color = 1 - ax.XAxis.Color;
                ax.YAxis.Color = 1 - ax.YAxis.Color;
                ax.ZAxis.Color = 1 - ax.ZAxis.Color;
            end

            if isa(obj.Parent,'matlab.ui.Figure')
                obj.Parent.Color = parentColor;
            else
                obj.Parent.BackgroundColor = parentColor;
                obj.Parent.ForegroundColor = ax.XAxis.Color;
            end

            updateMeshColor(scene,meshColor);

            timestamps = 0:1/scene.UpdateRate:scene.StopTime;
            pos = cell(1,numel(scene.Platforms));
            numUAVs = 0;
            for i = 1:numel(scene.Platforms)
                traj = scene.Platforms(i).Trajectory;
                if isempty(traj)
                    motion = scene.Platforms(i).read();
                    pos{i} = motion(1:3);
                else
                    pos{i} = lookupPose(traj,timestamps);
                    numUAVs = numUAVs + 1;
                end
            end

            m = numUAVs + 3;
            clrs = clrs(rem(0:m-1,size(clrs,1))+1,:);

            % Create theater plot
            tp = theaterPlot('Parent',ax);

            % Create track plotter
            trkP = trackPlotter(tp,'DisplayName','Tracked Position','MarkerFaceColor',clrs(1,:),'MarkerEdgeColor',clrs(1,:),'ConnectHistory','on','ColorizeHistory','off','MarkerSize',6);

            % Create position detection plotter
            posDetP = detectionPlotter(tp,'DisplayName','Instantaneous Position','MarkerFaceColor',clrs(2,:),'MarkerEdgeColor',clrs(2,:),'MarkerSize',6);

            % Create trajectory plotter
            trajP = trajectoryPlotter(tp,'DisplayName','Ground truth','Color',meshColor,'LineWidth',1,'LineStyle','-.');

            trajP.plotTrajectory(pos);
            
            hold on;
            l = legend(ax);
            l.AutoUpdate = 'on';

            if obj.PlotTDOAHyperbola
                % TDOA detection plotter
                tdoaDetP = cell(numUAVs,1);
                for i = 1:numUAVs
                    tdoaDetP{i} = plot3(nan,nan,nan,'Color',clrs(3+i,:),'DisplayName','TDOA','LineWidth',0.1,'LineStyle','-');
                end
                obj.TDOADetectionPlotter = tdoaDetP;
            end

            l.AutoUpdate = 'off';
            l.TextColor = abs(1 - l.Color);

            show3D(scene,Parent=ax);

            if ~obj.ShowLegend
                delete(l);
            end

            xMax = -inf;
            xMin = inf;
            yMax = -inf;
            yMin = inf;
            zMax = -inf;
            zMin = inf;
            for i = 1:numel(scene.Platforms)
                uavMotion = read(scene.Platforms(i));
                pos = uavMotion(1:3);
                xMax = max(pos(1),xMax);
                xMin = min(pos(1),xMin);
                yMax = max(pos(2),yMax);
                yMin = min(pos(2),yMin);
                zMax = max(pos(3),zMax);
                zMin = min(pos(3),zMin);
            end

            if strcmpi(obj.XLimits,'auto')
                ax.XLim = [xMin - 0.1*(xMax - xMin), xMax + 0.1*(xMax - xMin)];
            else
                ax.XLim = obj.XLimits;
            end

            if strcmpi(obj.YLimits,'auto')
                ax.YLim = [yMin - 0.1*(yMax - yMin), yMax + 0.1*(yMax - yMin)];
            else
                ax.YLim = obj.YLimits;
            end

            if strcmpi(obj.ZLimits,'auto')
                ax.ZLim = [zMin - 0.1*(zMax - zMin), yMax + 0.1*(zMax - zMin)];
            else
                ax.ZLim = obj.ZLimits;
            end

            grid(ax,'on');
            view(ax,obj.ViewAngles(1),obj.ViewAngles(2));

            obj.Axes = ax;
            obj.TrackPlotter = trkP;
            obj.PositionDetectionPlotter = posDetP;
        end

        function stepImpl(display, scene, tdoaDetections, positionDetections, tracks)
            % Update scene
            show3D(scene,Parent=display.Axes,FastUpdate=true);

            % Update tracks
            [pos, posCov] = getTrackPositions(tracks,'constvel');
            vel = getTrackVelocities(tracks,'constvel');
            labels = string([tracks.TrackID]);
            if display.PlotTrackCovariance
                display.TrackPlotter.plotTrack(pos,posCov,vel,labels);
            else
                display.TrackPlotter.plotTrack(pos,vel,labels);
            end
            % Update position detections
            pos = zeros(numel(positionDetections),3);
            posCov = zeros(3,3,numel(positionDetections));
            for i = 1:numel(positionDetections)
                pos(i,:) = positionDetections{i}.Measurement;
                posCov(:,:,i) = positionDetections{i}.MeasurementNoise;
            end
            if display.PlotDetectionCovariance
                display.PositionDetectionPlotter.plotDetection(pos,posCov);
            else
                display.PositionDetectionPlotter.plotDetection(pos);
            end
            

            if display.PlotTDOAHyperbola
                % Update TDOA detections
                for i = 1:numel(tdoaDetections)
                    x = zeros(0,1);
                    y = zeros(0,1);
                    z = zeros(0,1);
                    for j = 1:numel(tdoaDetections{i})
                        [thisX, thisY, thisZ] = tdoaHyperbola(tdoaDetections{i}{j},'2D');
                        x = [x;nan;thisX(:)]; %#ok<AGROW>
                        y = [y;nan;thisY(:)]; %#ok<AGROW>
                        z = [z;nan;thisZ(:)]; %#ok<AGROW>
                    end
                    display.TDOADetectionPlotter{i}.XData = x;
                    display.TDOADetectionPlotter{i}.YData = y;
                    display.TDOADetectionPlotter{i}.ZData = 0*x;
                end
            end

            if ~isempty(display.ZoomOnPlatformIndex)
                platform = scene.Platforms(display.ZoomOnPlatformIndex);
                uavMotion = read(platform);
                display.Axes.XLim = uavMotion(1) + [-10 10];
                display.Axes.YLim = uavMotion(2) + [-10 10];
                display.Axes.ZLim = uavMotion(3) + [-10 10];
            end
        end
    end
end


function colorOrder = darkColorOrder
    colorOrder = [1.0000    1.0000    0.0667
        0.0745    0.6235    1.0000
        1.0000    0.4118    0.1608
        0.3922    0.8314    0.0745
        0.7176    0.2745    1.0000
        0.0588    1.0000    1.0000
        1.0000    0.0745    0.6510];
    
    colorOrder(8,:) = [1 1 1];
    colorOrder(9,:) = [0 0 0];
    colorOrder(10,:) = 0.7*[1 1 1];
end

function varargout = tdoaHyperbola(tdoaDetection,type)
    globalParams = helperGetEmissionSpeedAndTimeScale;
    emissionSpeed = globalParams.EmissionSpeed;
    timeScale = globalParams.TimeScale;
    
    % Location of receiver in global coordinates
    p1 = tdoaDetection.MeasurementParameters(1);
    r1Pos = p1.OriginPosition(:);
    
    % Location of receiver 2 in global coordinates
    p2 = tdoaDetection.MeasurementParameters(2);
    r2Pos = p2.OriginPosition(:);
    
    theta = linspace(-pi/2,pi/2,100);
    if strcmpi(type,'2D')
        phi = deg2rad(-10);
    else
        phi = linspace(-pi/2,pi/2,100);
    end
    
    [THETA, PHI] = meshgrid(theta,phi);
    
    % Compute hyperbola quantities
    D = norm(r2Pos - r1Pos)/2;
    c = tdoaDetection.Measurement*emissionSpeed/timeScale;
    
    % Hyperbola equation in a frame where sensors lie on x axis and are located
    % at -D and D on x axis
    xC = -c./cos(THETA)./2;
    yC = sqrt(4*D^2 - c^2).*tan(THETA).*cos(PHI)./2;
    zC = sqrt(4*D^2 - c^2).*tan(THETA).*sin(PHI)./2;
    
    % Translate and rotate to the scenario frame
    r0 = (r1Pos + r2Pos)/2;
    a = [1;0;0];
    b = (r1Pos - r2Pos);
    b = b/norm(b);
    v = cross(a,b);
    s = norm(v);
    c = dot(a,b);
    V = [0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
    if abs(s) > 0
        R = eye(3) + V + V^2*(1 - c)/s^2;
    else
        R = eye(3);
    end
    
    x = R(1,1).*xC + R(1,2).*yC + R(1,3).*zC;
    y = R(2,1).*xC + R(2,2).*yC + R(2,3).*zC;
    z = R(3,1).*xC + R(3,2).*yC + R(3,3).*zC;
    x = x + r0(1);
    y = y + r0(2);
    z = z + r0(3);
    
    if strcmpi(type,'2D')
        x = x(:);
        y = y(:);
        z = z(:);
        varargout{1} = x;
        varargout{2} = y;
        varargout{3} = z;
    else
        varargout{1} = surf2patch(x,y,z);
    end
end

function updateMeshColor(scene, color)
    for i = 1:numel(scene.Platforms)
        updateMesh(scene.Platforms(i),'custom',{scene.Platforms(i).Mesh.Vertices,scene.Platforms(i).Mesh.Faces},color,scene.Platforms(i).MeshTransform);
    end
end