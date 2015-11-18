function parcutscreen(f)

if nargin<1
    f = gcf;
end

if isempty(f)
    set(0,'userdata',[]);
    return
end
os = computer;
os = os(1:3);
sz = get(f,'position');
if isempty(get(0,'userdata'))
    p.screenposition = get(0, 'ScreenSize');
    switch os
        case 'MAC'
            p.screenposition = p.screenposition + [43  0 -43 -22];
        otherwise
            p.screenposition = p.screenposition + [ 0 23   0 -23];
    end
    p.screenposition(1) = sz(1)+sz(3);
    p.screenposition(3) = p.screenposition(3)-sz(3);
    set(0,'userdata',p);
end
