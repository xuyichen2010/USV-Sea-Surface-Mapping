function pausebutton( fig_handle, fun_handle, varargin )
%% PAUSEBUTTON adds a pause toggle button to the figure's toolbar
%
%   PAUSEBUTTON( fig_handle, fun_handle )
%   PAUSEBUTTON( fig_handle, fun_handle, color )
%   PAUSEBUTTON( fig_handle, fun_handle, color, separator )
%
%   Arguments:
%   'fig_handle' is the handle to the figure
%   'fun_handle' is the handle to the pause button callback
%   'color' (optional) is a RGB color [R G B] with values [0,1]. If set to [],  
%       the default color (black) will be used  
%   'separator' (optional) adds a separator bar. Values are 'on' or 'off'
%
%   PAUSEBUTTON is a quick way to add a 'pause' toggle button to a toolbar for
%   quick (but limited) custom GUI interaction. This is used to stop in the 
%   middle of a loop (for example) so that you can take a detailed look at the 
%   data for a given frame.
%
% Copyright (C) 2017, by JRS @ LPR
%%

% Default color
butt = zeros(16,16,3);
% Custom color passed as RGB array
if nargin>=3
    color = varargin{1};
    if ~isempty(color)
        for i=1:3
            butt(:,:,i) = color(i);
        end
    end
end
butt(5:end-4,5:7,:) = 1;
butt(5:end-4,10:12,:) = 1;
% Add a separator (default is 'off')
separator = 'off';
if nargin==4
    separator = varargin{2};
end
% Get the toolbar handle
tb = findall(fig_handle,'type','uitoolbar');
% Associate the image and the function handle for the callback
uitoggletool(tb,'cdata',butt,'separator',separator,'handlevisibility','off', ...
    'clickedcallback',fun_handle,'tooltipstring','Pause the Application');
