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
% 6. reference: boolean if values are reference. tmpcDefaults to 'false'
% 7. hold_on: `on`/'off` -> defaults to 'on'
% 8. fontsize: Default =15
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
    options.fontsize = 15
    options.hold_on = 'on';
    options.reference = false
    options.stairs = false
    options.display_names = cell(Nplot,1)
end



for index = 1:Nplot
    subplot(Nplot,1,index)
    
    if options.reference 
        if isnan(var(:,index))
            continue
        end
        
        pp = yline(var(:,index),...
        Color=options.color, ...
        LineWidth=options.linewidth, ...
        LineStyle=options.linestyle);
    elseif options.stairs
        pp =  stairs(time, var(:,index),...
        Color=options.color, ...
        LineWidth=options.linewidth, ...
        Marker= options.marker,...
        LineStyle=options.linestyle);

    else
        pp = plot(time, var(:,index),...
        Color=options.color, ...
        LineWidth=options.linewidth, ...
        Marker= options.marker,...
        LineStyle=options.linestyle);
    end
    
    if isempty(options.display_names{index})
        pp.HandleVisibility='off';
    elseif     options.reference &&    isnan(var(:,index))
        pp.HandleVisibility='off';
    else 
        pp.DisplayName=options.display_names{index};
    end
    legend(Interpreter="latex",FontSize=options.fontsize)
    hold(options.hold_on)
end

end
