clear all; close all; clc;
% ������������ ������ ���� ���������� 
% � ��������������������� �� ������ ���������������� � ����� 5G
c = physconst('LightSpeed');
f = 30e9;      % ������� � ��������� ���, ��
da = 0.5*c/f;  % ���������� ����� ���������� ��
snrThr = 10;   % ��������� ��������� ������/������ ��� ����������� �����
anim = 0;      % 1 - ���. �������� ������
stdCoords = 0; % ��� ������ ��������� x, y, z
useAntUE = 0;  % ������������ �������� ������� �� UE

% 1 - ������������ ���������� ��������� �������� ��
% (������� ��������� ������ ��� ��������� ��)
backLobe = 1;

% ������ ��������� ��������������������, �. 
% ���� ����� 0, �� �� ����������� �� ������ ���� �������
Ta = 0;

% ����� ���� �������� �������:
% 1 - ��������� ��
% 2 - �������� ��
% 3 - �������� ��
Nel = 4;   % ����� ��������� �� � ����� ���������
antElPos = createAnt(1, Nel, da); % ������������ ��

% ����� ��������:
% 1 - gNB ����������� �� ����� ������� ������������ ���������� �������� UE
% 2 - gNB ����������� �� ������ ������� ������������ ���������� �������� UE
[gNB, ueNode, d, T, v] = createScenarion(2);
Nd = length(d);

Nnb = length(gNB);    % ����� gNB
Nue = length(ueNode); % ����� UE
% ����� ����� ������� (����� ����� ��������� ���������� UE)
N = length(ueNode(1).Trajectory(:,1));
trajArray = [ueNode.Trajectory];       % ������ ��������� UE [N x 3*Nue]
gNBcoords = [gNB(:).Coords].';         % ������ ��������� gNB [Nnb x 3]

if (anim == 1)
    antPattScl = 0.8;   % �����. ��������������� ��� ����������� �� gNB
    antPattSclUe = 0.6; % �����. ��������������� ��� ����������� �� UE
    fg = figure(5);
    fg.WindowState = 'maximized';
    grid on; hold on;
    indStart = N/Nd*9;
    % ����������� �� gNB (��������� ��. � antPattPlot)
    gNBptrnPlot = gobjects(1, 2);
    for i=1:Nnb
        [x, y, z] = getplotAntPattern(antElPos,...
            f, gNB(i), 6, antPattScl, backLobe);
        gNBptrnPlot(i) = surf(x+gNB(i).Coords(1),...
                        y+gNB(i).Coords(2),...
                        z+gNB(i).Coords(3), 'FaceColor', '#4DBEEE');
    end
    
    % ����������� �� UE (��������� ��. � antPattPlot)
    if (useAntUE == 1)
        ueptrnPlot = gobjects(1, 2);
        for i=1:Nue
            [x, y, z] = antPattPlot(antElPos,...
                f, ueNode(i), 6, antPattSclUe, backLobe);
            ueptrnPlot(i) = surf(x+ueNode(i).Trajectory(indStart,1),...
                            y+ueNode(i).Trajectory(indStart,2),...
                            z+ueNode(i).Trajectory(indStart,3),...
                            'FaceColor', '#4DBEEE');
        end
    end

    % ����������� ��������� UE
    uePlot = gobjects(1, 2);
    ueText = gobjects(1, 2);
    for i=1:Nue
        uePlot(i) = plot3(ueNode(i).Trajectory(1,1),...    
                          ueNode(i).Trajectory(1,2),...
                          ueNode(i).Trajectory(1,3), '^', ...
                          'MarkerSize', 10);
        ueText(i) = text(ueNode(i).Trajectory(1,1), ...
                         ueNode(i).Trajectory(1,2), ...
                         ueNode(i).Trajectory(1,3), ...
            sprintf('UE_{%i}', i), 'FontSize', 14, 'Color', '#A2142F');
    end
    
    ueDirPlot  = gobjects(1, 2);
    ueDirPlot2 = gobjects(1, 2);
    indSnoi = [2,1];
    for i=1:Nnb
        % ����������� ������� ����������� �� gNB �� UE
        ueDirPlot(i) = plot3([gNB(i).Coords(1);ueNode(i).Trajectory(1,1)],...
                             [gNB(i).Coords(2);ueNode(i).Trajectory(1,2)],...
                             [gNB(i).Coords(3);ueNode(i).Trajectory(1,3)], ...
                            'Color', '#76AB2F');
        % ����������� ������� ����������� �� gNB �� �������� UE
        ueDirPlot2(i) = plot3([gNB(i).Coords(1);ueNode(indSnoi(i)).Trajectory(1,1)],...
                             [gNB(i).Coords(2);ueNode(indSnoi(i)).Trajectory(1,2)],...
                             [gNB(i).Coords(3);ueNode(indSnoi(i)).Trajectory(1,3)], ...
                            'Color', '#D95319');
    end
    ueDirPlot(2).LineStyle = '--';
    ueDirPlot2(2).LineStyle = '--';

    for i=1:Nnb
        text(gNB(i).Coords(1), gNB(i).Coords(2), gNB(i).Coords(3)+5, ...
            sprintf('gNB_{%i}', i), 'FontSize', 16, 'Color', '#A2142F');
    end
    
    xlabel('x, �'); ylabel('y, �'); axis equal;
    axis([-5, 155, -5, 65, 0, 30]);
    view([0, 90]);
else
    indStart = 1;
end

% ������������� �������� ��� �������� ����� �������� �� ������ gNB �
% ������ UE (����� ��� ���������� ������ ������ ��������������� ������� ���
% ��������� ������� ������)
azAng = zeros(Nue, Nnb);
elAng = zeros(Nue, Nnb);
azAngUE = zeros(Nue, Nnb);
elAngUE = zeros(Nue, Nnb);

% �������� ���� ���������
for i=indStart:N % ���� �� ����� ����� �������
    % ������ ��������� ���� UE ��� i-� ����� �������
    ueCoordsi = reshape(trajArray(i, :).', 3, Nue).';
    % �������� ������ � ������ ��������� UE �������� stdCoords
    ueCoordsiErr = ueCoordsi + stdCoords*randn(size(ueCoordsi));
    
    if (mod(i, round(Ta/T)) == 1 || Ta == 0)
        % ������������� �������� ��� �������� 
        % ����� �������� �� ������ gNB � ������� UE
        azAng = zeros(Nue, Nnb); % ���� �������� �� �������
        elAng = zeros(Nue, Nnb); % ���� �������� �� ���� �����
        % ������������� �������� ��� �������� ����� ������� 
        % ��� ������ UE �� ������ gNB (������������ ��� ������� �� �� UE)
        azAngUE = zeros(Nue, Nnb);
        elAngUE = zeros(Nue, Nnb);

        for j=1:Nue % ���� �� ����� UE
            % ������, �������� ����������� �� gNB � UE 
            % � ���������� ������� ��������� x,y,z
            diffCoord = ueCoordsiErr(j,:) - gNBcoords;        
            for n=1:Nnb % ���� �� ����� gNB
                % ������, �������� ����������� �� gNB � UE � �������
                % ��������� �� gNB (�.�. � ������ ��������� �� gNB)
                dirVect = gNB(n).AntOrient.'*diffCoord(n,:).';
                % ������ ����� �������� �� n-� gNB � j-� UE
                azAng(j, n) = rad2deg(atan2(dirVect(2), dirVect(1)));
                elAng(j, n) = rad2deg(atan2(dirVect(3), sqrt(sum(dirVect(1:2).^2))));        

                % ������ ������� ������������ �����. �� n-� gNB �������������
                % j-� UE (����� ������������� gNB ����������� � ���������
                % ServgNB ��� ������ UE)
                if (n == ueNode(j).ServgNB)
                    gNB(n).Steer = getAntPatternSteer(antElPos, f,...
                        azAng(j, n), elAng(j, n));
                end

                % ������ ����� �������� � ������� ������������ �����. ��� �� UE
                if ( useAntUE == 1)
                    % ������, �������� ����������� �� UE � gNB � �������
                    % ��������� �� UE (�.�. � ������ ��������� �� UE)
                    dirVect = -ueNode(j).AntOrient.'*diffCoord(n,:).';
                    % ������ ����� �������� �� j-� UE � n-� gNB
                    azAngUE(j, n) = rad2deg(atan2(dirVect(2), dirVect(1)));
                    elAngUE(j, n) = rad2deg(atan2(dirVect(3), sqrt(sum(dirVect(1:2).^2))));
                    % ������ ������� ������������ �����. 
                    % �� j-� gNB, ���������� � n-� gNB ???
                    if (n == ueNode(j).ServgNB)
                        ueNode(j).Steer = getAntPatternSteer(antElPos, ...
                            f, azAngUE(j,n), elAngUE(j,n));
                    end
                end
            end
        end
    end
    
    % ������ ��� ���������� �������� �������� 
    % ����������� �������� �� UE �� ������ gNB
    gNBpwr = zeros(Nnb, 1);

    % ������ ��������� ����������� �������� �� ������������� gNB 
    % � ��������, ����������� �� �������� gNB ��� ������ UE
    for j=1:Nue % ���� �� ����� UE
        for n=1:Nnb % ���� �� ����� gNB
            % ������ ����� �������� �������� ������� UE
            if ( useAntUE == 1 && j == 1)
                % �� ��� ������� �� �� UE, ��� ������� 
                % ��������� �� ������������� gNB
                gUE = getAntPatternG(antElPos, f, ...
                    azAngUE(n), elAngUE(n), ueNode(1).Steer, backLobe).^2;
            else
                % �� ��� �� �� UE
                gUE = 1;
            end
            % ������ �������� ����������� �� j-� UE �� n-� gNB � ������
            % �������������������� �� UE � gNB (��� ����� ���������)
            gNBpwr(n) = gUE*getAntPatternG(antElPos, f,...
                azAng(j, n), elAng(j, n), gNB(n).Steer, backLobe).^2;
        end
        % ������ ���������� �� j-� UE �� ������ gNB
        diffCoord = ueCoordsi(j,:) - gNBcoords;
        distSpace = sqrt(sum(diffCoord.^2,2));
        % ������ �������� ����������� �� j-� UE �� ������ gNB � ������
        % ��������� (������ �������������� �� ������ ��������� � ���������
        % ������������)
        gNBpwr = pow2db(gNBpwr) - fspl(distSpace,c/f);
        % ������ ��������� ����������� �������� �� ������������� gNB 
        % � �������� ����������� �� �������� gNB ��� j-� UE
        ueNode(j).SNR(i) = gNBpwr(ueNode(j).ServgNB) - ...
            sum(gNBpwr(1:end ~= ueNode(j).ServgNB));
    end
    
    % ���������� ����������� ��������� UE � �� �� gNB/UE 
    % ��� �������� ���� �������
    if (anim == 1)
        for ip=1:Nnb
            % ���������� �� gNB
            [x, y, z] = getplotAntPattern(antElPos, f,...
                gNB(ip), 6, antPattScl, backLobe);
            gNBptrnPlot(ip).XData = x + gNB(ip).Coords(1);
            gNBptrnPlot(ip).YData = y + gNB(ip).Coords(2);
            gNBptrnPlot(ip).ZData = z + gNB(ip).Coords(3);
            % ���������� �� UE
            if (useAntUE == 1)
                [x, y, z] = antPattPlot(antElPos, f, ...
                    ueNode(ip), 6, antPattSclUe, backLobe);
                ueptrnPlot(ip).XData = x + ueCoordsi(ip,1);
                ueptrnPlot(ip).YData = y + ueCoordsi(ip,2);
                ueptrnPlot(ip).ZData = z + ueCoordsi(ip,3);
            end
            % ���������� ������� ����������� �� gNB �� UE
            ueDirPlot(ip).XData = [gNB(ip).Coords(1); ueCoordsi(ip,1)];
            ueDirPlot(ip).YData = [gNB(ip).Coords(2); ueCoordsi(ip,2)];
            ueDirPlot(ip).ZData = [gNB(ip).Coords(3); ueCoordsi(ip,3)];
            % ���������� ������� ����������� �� gNB ��  �������� UE
            ueDirPlot2(ip).XData = [gNB(ip).Coords(1); ueCoordsi(indSnoi(ip),1)];
            ueDirPlot2(ip).YData = [gNB(ip).Coords(2); ueCoordsi(indSnoi(ip),2)];
            ueDirPlot2(ip).ZData = [gNB(ip).Coords(3); ueCoordsi(indSnoi(ip),3)];
            % ���������� ��������� UE
            uePlot(ip).XData = ueCoordsi(ip,1);
            uePlot(ip).YData = ueCoordsi(ip,2);
            uePlot(ip).ZData = ueCoordsi(ip,3);
            ueText(ip).Position = [ueCoordsi(ip,1)+2, ...
                                   ueCoordsi(ip,2), ...
                                   ueCoordsi(ip,3)+5];
        end      
        pause(0.001)
    end % if (anim == 1)
end % for i=indStart:N % ���� �� ����� ����� �������

% ���������� �������� ��������� � ������� �������� 
% ��������� ������/������ 1-� UE ��� �����������
X = reshape(ueNode(1).Trajectory(:,1), [], Nd).';
Y = reshape(ueNode(1).Trajectory(:,2), [], Nd).';
Z = reshape(ueNode(1).SNR, [], Nd).';

% ����� ��������� ������/������ � ������ ����� ��������� 1-� UE
figure(1); surf(X, Y, Z, 'FaceColor', 'interp', 'EdgeColor','none');
grid on; xlabel('x, �'); ylabel('y, �'); view([0, 90]);
c1 = colorbar; c1.Label.String = '��������� ������/������, ��';

% ����� ����� ��������� 1-� UE, � ������� ��������� 
% ������/������ ��������� �������� ����� snrThr
figure(2); surf(X, Y, double(Z>snrThr), 'EdgeColor','none');
grid on; xlabel('x, �'); ylabel('y, �'); view([0, 90]);
colormap(winter(2)); c3 = colorbar;
c3.Label.String = sprintf('��������� ������/������ > %.0f ��', snrThr);
c3.Ticks = [0, 1]; view([0, 90]);

% ����� ��������� ������/������ � ������������
% ��������� � ���������� �� gNB
figure(3); surf(X, Y, Z, 'FaceColor', 'interp', 'EdgeColor','none');
grid on; hold on; 
for n=1:Nnb
    absAntCoord = (gNB(n).AntOrient*[0, -1, 0; 0, 1, 0].'*3 + gNB(n).Coords).';
    plot3(absAntCoord(:,1), absAntCoord(:,2), absAntCoord(:,3), 'Color', '#ECB01F', 'LineWidth', 3)
    text(gNB(n).Coords(1), gNB(n).Coords(2)*1.06, gNB(n).Coords(3), sprintf('Ant gNB_{%i}', n));
end
xlabel('x, �'); ylabel('y, �'); axis equal; view([0, 90]);
c2 = colorbar; c2.Label.String = '��������� ������/������, ��';

%%
% ������� �������� �������� ������� (��������� gNB, ���������� �������� UE)
function [gNB, ueNode, d, T, v] = createScenarion(sceneN)
T = 0.01;  % ������ ���������
v = 10;    % �������� �������� ���������� UE, �/�
switch sceneN
    % gNB ����������� �� ����� ������� ������������ ����������
    % �������� UE. UE ��������� ����������� �� ���������� d
    case 1 
        % ���������� ����� ������������ �������� ���� UE.
        % � ������ �������� ���������� UE ��������� �� d �� Y
        d = 0:0.1:10;
        % �������� ������� �� ���� �������� gNB 
        % (��������� gNB ��. � creatgNB)
        gNB(1) = createNB([25, 50, 5], [-90, -1]);
        gNB(2) = createNB([125, 50, 5], [-90, -1]);
        % �������� ������� �� ���� �������� UE 
        % (��������� UE ��. � createUEnode)
        ueNode(1) = createUE([0; 150], [0; 0], 0, 1, v, T, [90, 0]);
        ueNode(2) = createUE([150; 0], [0; 0], 0, 2, v, T, [90, 0]);            
            
        % ���������� ������ ���������� �������� UE ��� ������ �������� d.
        % ���������� UE2 �� ��������, �� d ��������� ���������� UE1
        Nd = length(d); % ����� �������� d
        trajUE1 = zeros(length(ueNode(1).Trajectory(:,1)), 3, Nd);
        trajUE2 = zeros(length(ueNode(1).Trajectory(:,1)), 3, Nd);
        for i=1:Nd
            trajUE1(:,:,i) = ueNode(1).Trajectory + [0, d(i), 0];
            trajUE2(:,:,i) = ueNode(2).Trajectory;
        end
        % ���������� ������ ���������� � ���� 
        % ����� ������ ��������� ��� ������ UE
        trajUE1 = permute(trajUE1, [1 3 2]);
        trajUE1 = reshape(trajUE1, [],3 ,1);
        trajUE2 = permute(trajUE2, [1 3 2]);
        trajUE2 = reshape(trajUE2, [],3 ,1);
        % ���������� ���������� � ��������� UE
        ueNode(1).Trajectory = trajUE1;
        ueNode(2).Trajectory = trajUE2;
            
    % gNB ����������� �� ������ ������� ������������ ����������
    % �������� UE. UE ��������� ����������� �� ���������� d
    case 2
        % ���������� ����� ������������ �������� ���� UE.
        % � ������ �������� ���������� UE ��������� �� d �� Y
        d = 0:0.1:10;
        % �������� ������� �� ���� �������� gNB
        gNB(1) = createNB([25, 50, 5], [-90, -1]);
        gNB(2) = createNB([125, 0, 5], [90, -1]);
        % �������� ������� �� ���� �������� UE
        ueNode(1) = createUE([0; 150], [20; 20], 0, 1, v, T, [90, 0]);
        ueNode(2) = createUE([150; 0], [20; 20], 0, 2, v, T, [90, 0]);
        % ���������� ������ ���������� �������� UE ��� ������ �������� d. 
        % ���������� UE2 �� ��������, �� d ��������� ���������� UE1
        Nd = length(d); % ����� �������� d
        trajUE1 = zeros(length(ueNode(1).Trajectory(:,1)), 3, Nd);
        trajUE2 = zeros(length(ueNode(1).Trajectory(:,1)), 3, Nd);
        for i=1:Nd
            trajUE1(:,:,i) = ueNode(1).Trajectory + [0, d(i), 0];
            trajUE2(:,:,i) = ueNode(2).Trajectory;
        end
        % ���������� ������ ���������� � ���� 
        % ����� ������ ��������� ��� ������ UE
        trajUE1 = permute(trajUE1, [1 3 2]);
        trajUE1 = reshape(trajUE1, [],3 ,1);
        trajUE2 = permute(trajUE2, [1 3 2]);
        trajUE2 = reshape(trajUE2, [],3 ,1);
        % ���������� ���������� � ��������� UE
        ueNode(1).Trajectory = trajUE1;
        ueNode(2).Trajectory = trajUE2;
  
        % gNB ����������� �� ����� ������� ������������ ����������
        % �������� UE. UE ��������� ���� �� ������ �� ���������� d
        case 3
            % ���������� ����� ������������ �������� ���� UE.
            % � ������ �������� ���������� UE ��������� �� d �� X
            d = 0:0.1:50;
            % �������� ������� �� ���� �������� gNB
            gNB(1) = creatgNB([25, 50, 5], [-90, -1]);
            gNB(2) = creatgNB([125, 50, 5], [-90, -1]);
            % �������� ������� �� ���� �������� UE
            ueNode(1) = createUEnode([0; 150], [0; 0], 0, 1, v, T, [90, 0]);
            ueNode(2) = createUEnode([0; 150], [0; 0], 0, 2, v, T, [90, 0]);
            
            % ���������� ������ ���������� �������� UE ��� ������ ��������
            % d. ���������� UE2 �� ��������, �� d ��������� ���������� UE1
            Nd = length(d); % ����� �������� d
            trajUE1 = zeros(length(ueNode(1).Trajectory(:,1)), 3, Nd);
            trajUE2 = zeros(length(ueNode(1).Trajectory(:,1)), 3, Nd);
            for i=1:Nd
                trajUE1(:,:,i) = ueNode(1).Trajectory + [d(i), -(i-1)/100, 0];
                trajUE2(:,:,i) = ueNode(2).Trajectory;
            end
            % ���������� ������ ���������� � ���� 
            % ����� ������ ��������� ��� ������ UE
            trajUE1 = permute(trajUE1, [1 3 2]);
            trajUE1 = reshape(trajUE1, [],3 ,1);
            trajUE2 = permute(trajUE2, [1 3 2]);
            trajUE2 = reshape(trajUE2, [],3 ,1);
            % ���������� ���������� � ��������� UE
            ueNode(1).Trajectory = trajUE1;
            ueNode(2).Trajectory = trajUE2;
        otherwise
end
end