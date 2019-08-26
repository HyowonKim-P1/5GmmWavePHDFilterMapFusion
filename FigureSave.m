function FigureSave(FolderN,FN)
    filename = sprintf('%s/%s',FolderN,FN);
    saveas(figure(1),filename,'fig');
    saveas(figure(1),filename,'jpg');
    close all;
end