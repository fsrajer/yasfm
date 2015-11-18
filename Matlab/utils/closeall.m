% closeall - Closes all, even figures with hidden handles
%
% See also PARGUI

% (c) T.Pajdla, www.neovision.cz, Sep 18, 2005
set(0,'ShowHiddenHandles','on')
p = get(0,'userdata');
if isfield(p,'screenposition');
    set(0,'userdata',[])
end
delete(get(0,'Children'))
close all