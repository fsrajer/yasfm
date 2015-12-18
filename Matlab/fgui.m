% Filip's simple GUI for running and viewing experiments
% Filip Srajer, srajefil@fel.cvut.cz
% 30 Dec 2014

function fgui(fs)
% FGUI - simple GUI for running and viewing experiments
 
    fname = 'fgui';

    mainHandle = subfig(1,6,1);
    set(gcf,'closerequestfcn',[]);  % make GUI unclosable

    set(mainHandle,'menubar','none');
    set(mainHandle,'numbertitle','off');
    set(gcf,'name',fname);

    pos = get(mainHandle,'position');
    width = pos(3);
    height = pos(4);

    fns = fieldnames(fs);
    
    cellWidth = width/2;
    cellHeight = 20;
    space = 2;
    cellHeightMin = (height/size(fns,1));
    if cellHeightMin < (cellHeight+space)
        space = 0;
        cellHeight = cellHeightMin;
    end
        
    for i = 1:size(fns,1)
        fn = fns{i};
        val = getfield(fs,fn);        
        cls = class(val);
        
        x0 = 0;
        y = height-i*(cellHeight+space);
        switch cls
            case {'char', 'double', 'logical'}
                posFn = [x0, y, cellWidth, cellHeight];
                posElem = [x0+cellWidth, y, cellWidth, cellHeight]; 
                hFn = uicontrol('Style','text','String',fn,'position',posFn,...
                    'HorizontalAlignment','center');
                
                switch cls
                    case 'char'
                        txt = val;
                        hElem = uicontrol('Style','edit','string',txt,...
                            'position',posElem, 'Callback',@charCallback,...
                            'Tag',fn,'HorizontalAlignment','left'); 

                    case 'double'
                        txt = num2str(val);
                        hElem = uicontrol('Style','edit','string',txt,...
                            'position',posElem,'Callback',@doubleCallback,...
                            'Tag',fn,'HorizontalAlignment','left');  

                    case 'logical'
                        hElem = uicontrol('Style','radiobutton','position',posElem,...
                            'Callback',@logicalCallback,'Tag',fn,...
                            'HorizontalAlignment','left'); 
                end
                
            case 'function_handle'
                posBtn = [x0, y, 2*cellWidth, cellHeight];
                hElem = uicontrol('Style','pushbutton','String',fn,'position',posBtn,...
                    'Callback',@functionHandleCallback,'Tag',fn,...
                    'HorizontalAlignment','center');  
                
            otherwise
                posFn = [x0, y, cellWidth, cellHeight];
                posElem = [x0+cellWidth, y, cellWidth, cellHeight]; 
                hFn = uicontrol('Style','text','String',fn,'position',posFn,...
                    'HorizontalAlignment','center');
                txt = ['unsupported class: ' cls];
                hElem = uicontrol('Style','text','String',txt,'position',posElem,...
                    'HorizontalAlignment','left');
                errordlg(['Unsupported field class in fs: ' cls],'Bad Input','modal');
        end   
    end
    
    parcutscreen(mainHandle); % shrink the screen 
    
    function charCallback(source, eventdata) 
        fs = setfield(fs,get(source,'Tag'),get(source,'String'));
    end

    function doubleCallback(source, eventdata) 
        value = str2num(get(source,'String'));
        if isempty(value)
            set(source,'String',num2str(getfield(fs,get(source,'Tag'))));
            errordlg('Enter numerical value','Bad Input','modal');
        else
            fs = setfield(fs,get(source,'Tag'),value);            
        end
    end

    function logicalCallback(source, eventdata) 
        fs = setfield(fs,get(source,'Tag'),logical(get(source,'Value')));
    end

    function functionHandleCallback(source, eventdata) 
        func = getfield(fs,get(source,'Tag'));
        fs = func(fs, get(source,'Tag'));
    end
 
end 