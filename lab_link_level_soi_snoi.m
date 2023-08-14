% clear all; close all; clc;
% Имитационная модель пространственной селекции полезного и мешающего 
% сигналов для сценария двух радиолиний с диаграммообразованием в составе 
% сверхплотной сети радиодоступа диапазона миллиметровых волн
c = physconst('LightSpeed');
f = 30e9;       % несущая частота диапазона ММВ, Гц
da = 0.5*c/f;   % расстояние между элементами АР, м
snrThr = 15;    % порог отношения сигнал/помеха для отображения карты
stdCoords = 0;  % СКО оценки координат x, y, z
useAntUE = 0;   % использовать антенную решетку на UE

% 1 - использовать подавление обратного лепестка ДН
% (в данной модели не имеет значения)
backLobe = 0;

% выбор типа антенной решетки:
% 1 - планарная АР
% 2 - линейная АР
% 3 - круговая АР
Nel = 8; % число элементов АР в одном измерении
antElPos = createAnt(1, Nel, da); % формирование АР

T = 0.5;          % период измерений
v = 1;            % скорость движения устройства UE, м/с
d = 0:0.2:100;    % дальность UE от BS
alpha = 0:0.2:60; % угловой разнос между UE, градусы

% создание структуры параметров eNB(параметры eNB см. в createNB)
eNB(1) = createNB([0, 0, 15], [90, -1]);
% создание массива структур параметров UE(параметры UE см. в createUEnode)
ueNode(1) = createUE([0; 15], [0; 0], 0, 1, v, T, [-90, 0]);
ueNode(2) = createUE([0; 15], [0; 0], 0, 1, v, T, [0 ,0]);

% создание массива координат точек расположения UE, где 
% для каждого значения в alpha соответствует набор значений d 
% (для каждого угла разноса расчет ведется для всех дальностей d)
Nd = length(d);     % число точек расчета по дальности
Na = length(alpha); % число точек расчета по угловому разносу
trajUE1t = zeros(Nd, 3, Na);
trajUE2t = zeros(Nd, 3, Na);
for i=1:Na
    trajUE1t(:,1:2,i) = [( sind(alpha(i)).*d).', (cosd(alpha(i))*d).'];
    trajUE2t(:,1:2,i) = [(-sind(alpha(i)).*d).', (cosd(alpha(i))*d).'];
end
% компоновка набора точек расположения UE 
% в один общий массив координат для каждой UE
trajUE1 = permute(trajUE1t, [1 3 2]); trajUE1 = reshape(trajUE1, [],3 ,1);
trajUE2 = permute(trajUE2t, [1 3 2]); trajUE2 = reshape(trajUE2, [],3 ,1);
% сохранение траектории в структуре UE
ueNode(1).Trajectory = trajUE1;
ueNode(2).Trajectory = trajUE2;

Nnb = length(eNB);      % число eNB
Nue = length(ueNode);   % число UE
% число точек расчета
N = length(ueNode(1).Trajectory(:,1));
trajArray = [ueNode.Trajectory];          % массив координат UE [N x 3*Nue]
eNBcoords = [eNB(:).Coords].';            % массив координат eNB [Nnb x 3]
% инициализация массива направляющий коэфф. eNB (для расчета SOI/SNOI 
% eNB имеет два вектора коэфф., для направления луча на UE1 и UE2)
eNB.Steer = zeros(size(antElPos,1), Nue);

% основной цикл обработки
for i=1:N % цикл по числу точек расчета
    % массив координат всех UE для i-й точки расчета
    ueCoordsi = reshape(trajArray(i, :).', 3, Nue).';
    % внесение ошибки в оценку координат UE согласно stdCoords
    ueCoordsiErr = ueCoordsi + stdCoords*randn(size(ueCoordsi));
    
    % инициализация массивов для хранения углов отправки от eNB к каждой UE
    azAng = zeros(Nue, 1); % угол отправки по азимуту
    elAng = zeros(Nue, 1); % угол отправки по углу места
    
    for j=1:Nue % цикл по числу UE
        % вектор, задающий направление из eNB в UE 
        % в глобальной системе координат x,y,z
        diffCoord = ueCoordsiErr(j,:) - eNBcoords;        
        % вектор, задающий направление из eNB в UE в системе
        % координат АР eNB (т.е. с учетом положения АР eNB)
        dirVect = eNB.AntOrient.'*diffCoord.';
        % расчет углов отправки от eNB к j-й UE
        azAng(j) = rad2deg(atan2(dirVect(2), dirVect(1)));
        elAng(j) = rad2deg(atan2(dirVect(3), sqrt(sum(dirVect(1:2).^2))));
        % расчет вектора направляющих коэфф. АР eNB для j-й UE
        eNB.Steer(:,j) = getAntPatternSteer(antElPos, f, azAng(j), elAng(j));
        % расчет углов отправки и вектора направляющих коэфф. для АР UE
        if ( useAntUE == 1 && j == 1)
            % вектор, задающий направление из UE в eNB 
            % в системе координат АР UE (т.е. с учетом положения АР UE)
            dirVect = -ueNode(j).AntOrient.'*diffCoord.';
            % расчет углов отправки от j-й UE к eNB
            azAngUE = rad2deg(atan2(dirVect(2), dirVect(1)));
            elAngUE = rad2deg(atan2(dirVect(3), sqrt(sum(dirVect(1:2).^2))));
            % расчет вектора направляющих коэфф. АР j-й UE работающей с eNB
            ueNode(j).Steer = getAntPatternSteer(antElPos, f, azAngUE, elAngUE);
        end
    end % for j=1:Nue % цикл по числу UE
    % массив для временного хранения значений 
    % принимаемой мощности на каждой UE eNB
    eNBpwr = zeros(1, Nue);
    % расчет коэфф усиления приемной антенны UE
    if (useAntUE == 1)
        % КУ при наличии АР на UE
        gUE = getAntPatternG(antElPos, f,...
            azAngUE, elAngUE, ueNode(1).Steer, backLobe).^2;
    else
        % КУ без АР на UE
        gUE = 1;
    end
    
    % расчет мощности принимаемой на j-й UE от eNB 
    % с учетом диаграммообразования на UE и eNB
    for j=1:Nue
        eNBpwr(j) = pow2db(gUE*getAntPatternG(antElPos, f,...
            azAng(1), elAng(1), eNB.Steer(:,j), backLobe).^2);
    end
    % расчет отношения SOI/SNOI
    ueNode(1).SNR(i) = eNBpwr(1) - eNBpwr(2);
end % for i=1:N % цикл по числу точек расчета

% подготовка массивов координат и массива значений 
% отношения SOI/SNOI 1-й UE для отображения
X = repmat(d, Na, 1);
Y = repmat(alpha, Nd, 1).';
Z = reshape(ueNode(1).SNR, Nd, Na).';
Z(Z>50) = 50; % ограничение макс. значения для лучшей визуализации

% карта отношения сигнал/помеха для всех значений d и alpha
figure(1); surf(X, Y, Z, 'FaceColor', 'interp', 'EdgeColor','none');
grid on; xlabel('d, м'); ylabel('\alpha, \circ');
view([0, 90]); c1 = colorbar; c1.Label.String = 'Отношение сигнал/помеха, дБ';

% Карта точек положения 1-й UE, в которых отношение 
% сигнал/помеха превышает заданный порог snrThr
figure(2); surf(X, Y, double(Z>snrThr), 'EdgeColor','none');
grid on; xlabel('d, м'); ylabel('\alpha, \circ'); 
view([0, 90]); colormap(winter(2)); c3 = colorbar;
c3.Label.String = sprintf('Отношение сигнал/помеха > %.0f дБ', snrThr);
c3.Ticks = [0, 1];

% срез отношения сигнал/помеха по 
% заданной величине d (ds) для всех значений alpha
figure(3)
ds = 60; % величина d, для которой строится срез
% нахождение ближайшего к ds значений доступных d
[~,ind] = min(abs(X(1,:) - ds)); ds = X(1,ind);
plot(Y(:, ind), Z(:, ind)); grid on;
ylabel('Отношение сигнал/помеха, дБ'); xlabel('\alpha, \circ');
title(sprintf('Отношение сигнал/помеха (дБ) при d=%0.0f м', ds));
hold on; % legend('N=2','N=4','N=6','N=8');

% отображение схемы сценария расчета
figure(4);
alphPlt = 20; % угловой разнос для отображения схемы сценария
[~,alphInd] = min(abs(alpha - alphPlt));

plot(trajUE1t([1,Nd],1,alphInd), trajUE1t([1,Nd],2,alphInd)); hold on;
plot(trajUE2t([1,Nd],1,alphInd), trajUE2t([1,Nd],2,alphInd));
plot([0;0], [0; d(end)], '--');

plot(0, 0, '^', 'MarkerSize', 10); text(3, 3, 'BS');
plot(trajUE1t(Nd,1,alphInd), trajUE1t(Nd,2,alphInd), 'o', 'MarkerSize', 10);
text(trajUE1t(Nd,1,alphInd)-10, trajUE1t(Nd,2,alphInd), 'UE_1');
plot(trajUE2t(Nd,1,alphInd), trajUE2t(Nd,2,alphInd), 'o', 'MarkerSize', 10)
text(trajUE2t(Nd,1,alphInd)+5, trajUE2t(Nd,2,alphInd), 'UE_2');
text(trajUE1t(Nd,1,alphInd)/1.3-5, trajUE1t(Nd,2,alphInd)/1.3, 'd', 'FontSize', 16);
text(trajUE2t(Nd,1,alphInd)/1.3+5, trajUE2t(Nd,2,alphInd)/1.3, 'd', 'FontSize', 16);
alphArc = 90-alpha(alphInd):90;
xArc = cosd(alphArc)*d(end-10);
yArc = sind(alphArc)*d(end-10);
plot(xArc, yArc);
text(mean(xArc), max(yArc), '\alpha', 'FontSize', 16);

% отображение двух лучей ДН eNB, направленных на UE
azAng = -90:90; % построение ДН по азимуту (только передняя полусфера)
gg1 = zeros(size(azAng));
gg2 = zeros(size(azAng));
gg = zeros(size(azAng));
% вектор направляющих коэфф. АР eNB для UE1
w1 = getAntPatternSteer(antElPos, f, -alpha(alphInd), 0);
% вектор направляющих коэфф. АР eNB для UE2 (UE2 располагается зеркально UE1)
w2 = getAntPatternSteer(antElPos, f,  alpha(alphInd), 0);
% расчет ДН eNB для UE1 (gg1) и UE2 (gg2) с учетом векторов направляющих
% коэфф. w1 и w2
for i=1:length(gg1)
    gg1(i) = getAntPatternG(antElPos, f, azAng(i), 0, w1, backLobe)/size(antElPos,1);
    gg2(i) = getAntPatternG(antElPos, f, azAng(i), 0, w2, backLobe)/size(antElPos,1);
end
% расчет координат кривых ДН для отображения
alphPatt = azAng + 90; % повернуто на 90 градусов для лучшей визуализации
xPatt1 = cosd(alphPatt).*gg1*mean(d);
yPatt1 = sind(alphPatt).*gg1*mean(d);
xPatt2 = cosd(alphPatt).*gg2*mean(d);
yPatt2 = sind(alphPatt).*gg2*mean(d);

set(gca,'ColorOrderIndex',1); plot(xPatt1, yPatt1);
plot(xPatt2, yPatt2); xlabel('x, м'); ylabel('y, м'); grid on; axis equal;