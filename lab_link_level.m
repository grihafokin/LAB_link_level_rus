clear all; close all; clc;
% Имитационная модель двух радиолиний 
% с диаграммообразованием на основе позиционирования в сетях 5G
c = physconst('LightSpeed');
f = 30e9;      % несущая в диапазоне ММВ, Гц
da = 0.5*c/f;  % расстояние между элементами АР
snrThr = 10;   % пороговое отношения сигнал/помеха для отображения карты
anim = 0;      % 1 - вкл. анимацию работы
stdCoords = 0; % СКО оценки координат x, y, z
useAntUE = 0;  % использовать антенную решетку на UE

% 1 - использовать подавление обратного лепестка ДН
% (логично применять только для планарной АР)
backLobe = 1;

% период процедуры диаграммообразования, с. 
% если равно 0, то ДН формируется на каждом шаге расчета
Ta = 0;

% выбор типа антенной решетки:
% 1 - планарная АР
% 2 - линейная АР
% 3 - круговая АР
Nel = 4;   % число элементов АР в одном измерении
antElPos = createAnt(1, Nel, da); % формирование АР

% выбор сценария:
% 1 - gNB расположены на одной стороне относительно траектории движения UE
% 2 - gNB расположены по разные стороны относительно траектории движения UE
[gNB, ueNode, d, T, v] = createScenarion(2);
Nd = length(d);

Nnb = length(gNB);    % число gNB
Nue = length(ueNode); % число UE
% число точек расчета (число точек координат траектории UE)
N = length(ueNode(1).Trajectory(:,1));
trajArray = [ueNode.Trajectory];       % массив координат UE [N x 3*Nue]
gNBcoords = [gNB(:).Coords].';         % массив координат gNB [Nnb x 3]

if (anim == 1)
    antPattScl = 0.8;   % коэфф. масштабирования для отображения ДН gNB
    antPattSclUe = 0.6; % коэфф. масштабирования для отображения ДН UE
    fg = figure(5);
    fg.WindowState = 'maximized';
    grid on; hold on;
    indStart = N/Nd*9;
    % отображение ДН gNB (параметры см. в antPattPlot)
    gNBptrnPlot = gobjects(1, 2);
    for i=1:Nnb
        [x, y, z] = getplotAntPattern(antElPos,...
            f, gNB(i), 6, antPattScl, backLobe);
        gNBptrnPlot(i) = surf(x+gNB(i).Coords(1),...
                        y+gNB(i).Coords(2),...
                        z+gNB(i).Coords(3), 'FaceColor', '#4DBEEE');
    end
    
    % отображение ДН UE (параметры см. в antPattPlot)
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

    % отображение положения UE
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
        % отображение вектора направления от gNB на UE
        ueDirPlot(i) = plot3([gNB(i).Coords(1);ueNode(i).Trajectory(1,1)],...
                             [gNB(i).Coords(2);ueNode(i).Trajectory(1,2)],...
                             [gNB(i).Coords(3);ueNode(i).Trajectory(1,3)], ...
                            'Color', '#76AB2F');
        % отображение вектора направления от gNB на соседней UE
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
    
    xlabel('x, м'); ylabel('y, м'); axis equal;
    axis([-5, 155, -5, 65, 0, 30]);
    view([0, 90]);
else
    indStart = 1;
end

% инициализация массивов для хранения углов отправки от каждой gNB к
% каждой UE (нужно для устранении ошибки чтения несуществующего массива при
% некоторых настрой модеои)
azAng = zeros(Nue, Nnb);
elAng = zeros(Nue, Nnb);
azAngUE = zeros(Nue, Nnb);
elAngUE = zeros(Nue, Nnb);

% основной цикл обработки
for i=indStart:N % цикл по числу точек расчета
    % массив координат всех UE для i-й точки расчета
    ueCoordsi = reshape(trajArray(i, :).', 3, Nue).';
    % внесение ошибки в оценку координат UE согласно stdCoords
    ueCoordsiErr = ueCoordsi + stdCoords*randn(size(ueCoordsi));
    
    if (mod(i, round(Ta/T)) == 1 || Ta == 0)
        % инициализация массивов для хранения 
        % углов отправки от каждой gNB к каждому UE
        azAng = zeros(Nue, Nnb); % угол отправки по азимуту
        elAng = zeros(Nue, Nnb); % угол отправки по углу места
        % инициализация массивов для хранения углов прихода 
        % для каждой UE от каждой gNB (используется при наличии АР на UE)
        azAngUE = zeros(Nue, Nnb);
        elAngUE = zeros(Nue, Nnb);

        for j=1:Nue % цикл по числу UE
            % вектор, задающий направление из gNB в UE 
            % в глобальной системе координат x,y,z
            diffCoord = ueCoordsiErr(j,:) - gNBcoords;        
            for n=1:Nnb % цикл по числу gNB
                % вектор, задающий направление из gNB в UE в системе
                % координат АР gNB (т.е. с учетом положения АР gNB)
                dirVect = gNB(n).AntOrient.'*diffCoord(n,:).';
                % расчет углов отправки от n-й gNB к j-й UE
                azAng(j, n) = rad2deg(atan2(dirVect(2), dirVect(1)));
                elAng(j, n) = rad2deg(atan2(dirVect(3), sqrt(sum(dirVect(1:2).^2))));        

                % расчет вектора направляющих коэфф. АР n-й gNB обслуживающей
                % j-ю UE (номер обслуживающей gNB указывается в параметре
                % ServgNB для каждой UE)
                if (n == ueNode(j).ServgNB)
                    gNB(n).Steer = getAntPatternSteer(antElPos, f,...
                        azAng(j, n), elAng(j, n));
                end

                % расчет углов отправки и вектора направляющих коэфф. для АР UE
                if ( useAntUE == 1)
                    % вектор, задающий направление из UE в gNB в системе
                    % координат АР UE (т.е. с учетом положения АР UE)
                    dirVect = -ueNode(j).AntOrient.'*diffCoord(n,:).';
                    % расчет углов отправки от j-й UE к n-й gNB
                    azAngUE(j, n) = rad2deg(atan2(dirVect(2), dirVect(1)));
                    elAngUE(j, n) = rad2deg(atan2(dirVect(3), sqrt(sum(dirVect(1:2).^2))));
                    % расчет вектора направляющих коэфф. 
                    % АР j-й gNB, работающей с n-й gNB ???
                    if (n == ueNode(j).ServgNB)
                        ueNode(j).Steer = getAntPatternSteer(antElPos, ...
                            f, azAngUE(j,n), elAngUE(j,n));
                    end
                end
            end
        end
    end
    
    % массив для временного хранения значений 
    % принимаемой мощности на UE от каждой gNB
    gNBpwr = zeros(Nnb, 1);

    % расчет отношения принимаемой мощности от обслуживающей gNB 
    % к мощности, принимаемой от соседней gNB для каждой UE
    for j=1:Nue % цикл по числу UE
        for n=1:Nnb % цикл по числу gNB
            % расчет коэфф усиления приемной антенны UE
            if ( useAntUE == 1 && j == 1)
                % КУ при наличии АР на UE, луч которой 
                % направлен на обслуживающую gNB
                gUE = getAntPatternG(antElPos, f, ...
                    azAngUE(n), elAngUE(n), ueNode(1).Steer, backLobe).^2;
            else
                % КУ без АР на UE
                gUE = 1;
            end
            % расчет мощности принимаемой на j-й UE от n-й gNB с учетом
            % диаграммообразования на UE и gNB (без учета дальности)
            gNBpwr(n) = gUE*getAntPatternG(antElPos, f,...
                azAng(j, n), elAng(j, n), gNB(n).Steer, backLobe).^2;
        end
        % расчет расстояния от j-й UE до каждой gNB
        diffCoord = ueCoordsi(j,:) - gNBcoords;
        distSpace = sqrt(sum(diffCoord.^2,2));
        % расчет мощности принимаемой на j-й UE от каждой gNB с учетом
        % дальности (потери рассчитываются по модели затухания в свободном
        % пространстве)
        gNBpwr = pow2db(gNBpwr) - fspl(distSpace,c/f);
        % расчет отношения принимаемой мощности от обслуживающей gNB 
        % к мощности принимаемой от соседней gNB для j-й UE
        ueNode(j).SNR(i) = gNBpwr(ueNode(j).ServgNB) - ...
            sum(gNBpwr(1:end ~= ueNode(j).ServgNB));
    end
    
    % обновление отображения положения UE и ДН АР gNB/UE 
    % для текущего шага расчета
    if (anim == 1)
        for ip=1:Nnb
            % обновление ДН gNB
            [x, y, z] = getplotAntPattern(antElPos, f,...
                gNB(ip), 6, antPattScl, backLobe);
            gNBptrnPlot(ip).XData = x + gNB(ip).Coords(1);
            gNBptrnPlot(ip).YData = y + gNB(ip).Coords(2);
            gNBptrnPlot(ip).ZData = z + gNB(ip).Coords(3);
            % обновление ДН UE
            if (useAntUE == 1)
                [x, y, z] = antPattPlot(antElPos, f, ...
                    ueNode(ip), 6, antPattSclUe, backLobe);
                ueptrnPlot(ip).XData = x + ueCoordsi(ip,1);
                ueptrnPlot(ip).YData = y + ueCoordsi(ip,2);
                ueptrnPlot(ip).ZData = z + ueCoordsi(ip,3);
            end
            % обновление вектора направления от gNB на UE
            ueDirPlot(ip).XData = [gNB(ip).Coords(1); ueCoordsi(ip,1)];
            ueDirPlot(ip).YData = [gNB(ip).Coords(2); ueCoordsi(ip,2)];
            ueDirPlot(ip).ZData = [gNB(ip).Coords(3); ueCoordsi(ip,3)];
            % обновление вектора направления от gNB на  соседнюю UE
            ueDirPlot2(ip).XData = [gNB(ip).Coords(1); ueCoordsi(indSnoi(ip),1)];
            ueDirPlot2(ip).YData = [gNB(ip).Coords(2); ueCoordsi(indSnoi(ip),2)];
            ueDirPlot2(ip).ZData = [gNB(ip).Coords(3); ueCoordsi(indSnoi(ip),3)];
            % обновление положения UE
            uePlot(ip).XData = ueCoordsi(ip,1);
            uePlot(ip).YData = ueCoordsi(ip,2);
            uePlot(ip).ZData = ueCoordsi(ip,3);
            ueText(ip).Position = [ueCoordsi(ip,1)+2, ...
                                   ueCoordsi(ip,2), ...
                                   ueCoordsi(ip,3)+5];
        end      
        pause(0.001)
    end % if (anim == 1)
end % for i=indStart:N % цикл по числу точек расчета

% подготовка массивов координат и массива значений 
% отношения сигнал/помеха 1-й UE для отображения
X = reshape(ueNode(1).Trajectory(:,1), [], Nd).';
Y = reshape(ueNode(1).Trajectory(:,2), [], Nd).';
Z = reshape(ueNode(1).SNR, [], Nd).';

% карта отношения сигнал/помеха в каждой точке положения 1-й UE
figure(1); surf(X, Y, Z, 'FaceColor', 'interp', 'EdgeColor','none');
grid on; xlabel('x, м'); ylabel('y, м'); view([0, 90]);
c1 = colorbar; c1.Label.String = 'Отношение сигнал/помеха, дБ';

% карта точек положения 1-й UE, в которых отношение 
% сигнал/помеха превышает заданный порог snrThr
figure(2); surf(X, Y, double(Z>snrThr), 'EdgeColor','none');
grid on; xlabel('x, м'); ylabel('y, м'); view([0, 90]);
colormap(winter(2)); c3 = colorbar;
c3.Label.String = sprintf('Отношение сигнал/помеха > %.0f дБ', snrThr);
c3.Ticks = [0, 1]; view([0, 90]);

% карта отношения сигнал/помеха с отображением
% положения и ориентации АР gNB
figure(3); surf(X, Y, Z, 'FaceColor', 'interp', 'EdgeColor','none');
grid on; hold on; 
for n=1:Nnb
    absAntCoord = (gNB(n).AntOrient*[0, -1, 0; 0, 1, 0].'*3 + gNB(n).Coords).';
    plot3(absAntCoord(:,1), absAntCoord(:,2), absAntCoord(:,3), 'Color', '#ECB01F', 'LineWidth', 3)
    text(gNB(n).Coords(1), gNB(n).Coords(2)*1.06, gNB(n).Coords(3), sprintf('Ant gNB_{%i}', n));
end
xlabel('x, м'); ylabel('y, м'); axis equal; view([0, 90]);
c2 = colorbar; c2.Label.String = 'Отношение сигнал/помеха, дБ';

%%
% функция создания сценария расчета (положение gNB, траектории движения UE)
function [gNB, ueNode, d, T, v] = createScenarion(sceneN)
T = 0.01;  % период измерений
v = 10;    % скорость движения устройства UE, м/с
switch sceneN
    % gNB расположены на одной стороне относительно траектории
    % движения UE. UE двигаются параллельно на расстоянии d
    case 1 
        % расстояние между траекториями движения двух UE.
        % В данном сценарии траектории UE разнесены на d по Y
        d = 0:0.1:10;
        % создание массива из двух структур gNB 
        % (параметры gNB см. в creatgNB)
        gNB(1) = createNB([25, 50, 5], [-90, -1]);
        gNB(2) = createNB([125, 50, 5], [-90, -1]);
        % создание массива из двух структур UE 
        % (параметры UE см. в createUEnode)
        ueNode(1) = createUE([0; 150], [0; 0], 0, 1, v, T, [90, 0]);
        ueNode(2) = createUE([150; 0], [0; 0], 0, 2, v, T, [90, 0]);            
            
        % построения набора траекторий движения UE для разных значений d.
        % Траектория UE2 не меняется, на d смещается траектория UE1
        Nd = length(d); % число значений d
        trajUE1 = zeros(length(ueNode(1).Trajectory(:,1)), 3, Nd);
        trajUE2 = zeros(length(ueNode(1).Trajectory(:,1)), 3, Nd);
        for i=1:Nd
            trajUE1(:,:,i) = ueNode(1).Trajectory + [0, d(i), 0];
            trajUE2(:,:,i) = ueNode(2).Trajectory;
        end
        % компоновка набора траекторий в один 
        % общий массив координат для каждой UE
        trajUE1 = permute(trajUE1, [1 3 2]);
        trajUE1 = reshape(trajUE1, [],3 ,1);
        trajUE2 = permute(trajUE2, [1 3 2]);
        trajUE2 = reshape(trajUE2, [],3 ,1);
        % сохранение траектории в структуре UE
        ueNode(1).Trajectory = trajUE1;
        ueNode(2).Trajectory = trajUE2;
            
    % gNB расположены по разные стороны относительно траектории
    % движения UE. UE двигаются параллельно на расстоянии d
    case 2
        % расстояние между траекториями движения двух UE.
        % В данном сценарии траектории UE разнесены на d по Y
        d = 0:0.1:10;
        % создание массива из двух структур gNB
        gNB(1) = createNB([25, 50, 5], [-90, -1]);
        gNB(2) = createNB([125, 0, 5], [90, -1]);
        % создание массива из двух структур UE
        ueNode(1) = createUE([0; 150], [20; 20], 0, 1, v, T, [90, 0]);
        ueNode(2) = createUE([150; 0], [20; 20], 0, 2, v, T, [90, 0]);
        % построения набора траекторий движения UE для разных значений d. 
        % Траектория UE2 не меняется, на d смещается траектория UE1
        Nd = length(d); % число значений d
        trajUE1 = zeros(length(ueNode(1).Trajectory(:,1)), 3, Nd);
        trajUE2 = zeros(length(ueNode(1).Trajectory(:,1)), 3, Nd);
        for i=1:Nd
            trajUE1(:,:,i) = ueNode(1).Trajectory + [0, d(i), 0];
            trajUE2(:,:,i) = ueNode(2).Trajectory;
        end
        % компоновка набора траекторий в один 
        % общий массив координат для каждой UE
        trajUE1 = permute(trajUE1, [1 3 2]);
        trajUE1 = reshape(trajUE1, [],3 ,1);
        trajUE2 = permute(trajUE2, [1 3 2]);
        trajUE2 = reshape(trajUE2, [],3 ,1);
        % сохранение траектории в структуре UE
        ueNode(1).Trajectory = trajUE1;
        ueNode(2).Trajectory = trajUE2;
  
        % gNB расположены на одной стороне относительно траектории
        % движения UE. UE двигаются друг за другом на расстоянии d
        case 3
            % расстояние между траекториями движения двух UE.
            % В данном сценарии траектории UE разнесены на d по X
            d = 0:0.1:50;
            % создание массива из двух структур gNB
            gNB(1) = creatgNB([25, 50, 5], [-90, -1]);
            gNB(2) = creatgNB([125, 50, 5], [-90, -1]);
            % создание массива из двух структур UE
            ueNode(1) = createUEnode([0; 150], [0; 0], 0, 1, v, T, [90, 0]);
            ueNode(2) = createUEnode([0; 150], [0; 0], 0, 2, v, T, [90, 0]);
            
            % построения набора траекторий движения UE для разных значений
            % d. Траектория UE2 не меняется, на d смещается траектория UE1
            Nd = length(d); % число значений d
            trajUE1 = zeros(length(ueNode(1).Trajectory(:,1)), 3, Nd);
            trajUE2 = zeros(length(ueNode(1).Trajectory(:,1)), 3, Nd);
            for i=1:Nd
                trajUE1(:,:,i) = ueNode(1).Trajectory + [d(i), -(i-1)/100, 0];
                trajUE2(:,:,i) = ueNode(2).Trajectory;
            end
            % компоновка набора траекторий в один 
            % общий массив координат для каждой UE
            trajUE1 = permute(trajUE1, [1 3 2]);
            trajUE1 = reshape(trajUE1, [],3 ,1);
            trajUE2 = permute(trajUE2, [1 3 2]);
            trajUE2 = reshape(trajUE2, [],3 ,1);
            % сохранение траектории в структуре UE
            ueNode(1).Trajectory = trajUE1;
            ueNode(2).Trajectory = trajUE2;
        otherwise
end
end