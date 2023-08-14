% функция создания структуры eNB
function eNB = createNB(coords, antDir)
% coords - координаты [x,y,z] eNB, м
% antDir - ориентация АР eNB (направление оси симметрии) [азимут, наклон],
% градусы
eNB.Coords = coords.'; % координаты eNB
eNB.AntDir = antDir.'; % ориентация АР eNB
% матрица поворота согласно значениям antDir, используется для пересчета 
% векторов направлений из глобальных координат в систему координат АР eNB
eNB.AntOrient = rotz(antDir(1))*roty(-antDir(2));
eNB.Steer = 1;
end