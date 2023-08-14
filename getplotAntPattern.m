% функция построения ДН АР
% antElPos - массив координат [x,y,z] АЭ, м
% f - рабочая частота, Гц
% nodeStruct - структура с параметрами узла (eNB или UE)
% angStep - шаг сетки углов по азимуту и углу места
% scl - коэфф. масштабирования ДН (для удобства визуализации)
% backLobe - использовать подавление обратного лепестка
% [x, y, z] - координаты поверхности для отображения ДН (безразмерная 
% величина, используется только для отображения)
function [x, y, z] = getplotAntPattern(antElPos, f, nodeStruct, angStep, scl, backLobe)
% сетка углов по азимуту и углу места для расчета ДН
azA = 0:angStep:360;
elA = -90:angStep:90;
% инициализация массивов для сохранения значений ДН
azN = length(azA);
elN = length(elA);
x = zeros(elN, azN);
y = zeros(elN, azN);
z = zeros(elN, azN);
for i=1:elN % цикл по массиву углов места
    for j=1:azN % цикл по массиву азимутов
        % расчет значения ДН АР для i-го угла места и j-го азимута с учетом
        % вектор направляющих коэфф. АР (nodeStruct.Steer)
        p = getAntPatternG(antElPos, f, azA(j), elA(i), ...
            nodeStruct.Steer, backLobe)*scl;
        % пересчет значений ДН АР из полярных координат в прямоугольные
        % с учетом ориентации АР (nodeStruct.AntOrient)
        r = p.*cosd(elA(i));
        xyz = nodeStruct.AntOrient*[r.*cosd(azA(j)); ...
            r.*sind(azA(j)); p.*sind(elA(i))];
        x(i,j) = xyz(1);
        y(i,j) = xyz(2);
        z(i,j) = xyz(3);
    end
end
end