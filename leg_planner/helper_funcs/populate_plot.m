function [] = populate_plot(Nplot,time,var,options)
%populate_plot(Nplot,time,var, <optional inputs>):
%This helper function populates a figure with the plotter quantities
%specifying the desired plotting format
%
%
%INPUT:
% 1. Nplot: number of subplots
% 2. time : time vector (n_samples x1)
% 3. var  : number of subplots (n_samples x Nplot)
%
%OPTIONAL INPUT - NAME PAIR ARGUMENTS:
% 1. linewidth: Defaults to 2
% 2. color: Defaults to 'k'
% 3. linestyle: Defaults to '-'
% 4. marker: Defaults to 'none'
% 5. display_names: cell array with LATEX style display names
% 6. hold_on: `on`/'off` -> defaults to 'on'
%
%OUTPUT:
% none. It affects the last declared figure.
%
%SEE ALSO:
%   initialize_plot, naming_automation

arguments
    Nplot {mustBeInteger,mustBeNonnegative}
    time 
    var   

    options.linewidth = 2
    options.color = 'k'
    options.linestyle = "-"
    options.marker = "none"
    options.hold_on = 'on';
    options.display_names = cell(Nplot,1)
end


for index = 1:Nplot
    subplot(Nplot,1,index)
    

    if isempty(options.display_names{index})
        plot(time,...
        var(:,index),...
        Color=options.color, ...
        LineWidth=options.linewidth, ...
        Marker= options.marker,...
        LineStyle=options.linestyle);
    else
        plot(time,...
        var(:,index),...
        Color=options.color, ...
        LineWidth=options.linewidth, ...
        Marker= options.marker,...
        LineStyle=options.linestyle, ...
        DisplayName=options.display_names{index});
        legend(Interpreter="latex")
    end

    %hold on/off
    hold(options.hold_on)
%     if options.hold_on  
%         hold on
%     else
%         hold off
%     end

end