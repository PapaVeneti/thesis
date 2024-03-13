function [] = initialize_plot(Nplot, options)
%initialize_plot(Nplot, <optional inputs>):  
%This helper function initializes a figure that contains: 
%   - multiple subplots with 
%   - a title, 
%   - ylabels and xlabels,
%   - limits of the quantities and
%   - sets the axis fontsize 
%
%
%INPUT:
%   1. Nplot: number of subplots
%
%OPTIONAL INPUT - NAME PAIR ARGUMENTS:
%   1. title:  title string in LATEX format
%   2. ylabel: cell array with latex style name of the quantities
%   3. xlabel: string with the independedn variable in LATEX format. 
%   - Defaults to '$t\ [s]$'
%   4. fontsize:  Fontsize for title and labels
%   5. axis_size: Fontsize for axis ticks
%   6. grid: `on`/'off` -> enable grid
%   7. limits:   Array of size (Nplot,:) that contains the limits of each
%   quantity in the same row
%   8. limit_format:  Format for limit ylines
%
%OUTPUT:
%   none. It affects the last declared figure.
%
%SEE ALSO:
%   populate_plot, naming_automation

arguments
    Nplot {mustBeInteger,mustBeNonnegative}

    options.title     = '$$'
    options.subplot_titles = cell(Nplot,1)
    options.ylabel      = cell(Nplot,1)
    options.xlabel      = '$t\ [s]$'
    options.fontsize  = 25
    options.axis_size = 10
    options.grid      = 'on'  

    options.limits   
    options.limit_format = 'r--'
end

%% Handle optional arguments
add_limits=true;
if ~isfield(options, 'limits')
    add_limits = false;
elseif size(options.limits,1 ) ~= Nplot
    error('Column number of limits must be equal to number of subplots.')
end


%% Add plots

for index = 1:Nplot
    subplot(Nplot,1,index)
    title(options.subplot_titles{index},Interpreter='latex',FontSize=options.fontsize)
    set(gca,'Fontsize',options.axis_size)
    %title
%     if index == 1
%         title(options.title,Interpreter='latex',FontSize=options.fontsize)
%     end
    
    %limits
    if add_limits
        yline(options.limits(index,:) ,options.limit_format)
    end

    ylabel(options.ylabel{index},Interpreter='latex',FontSize=options.fontsize)
    hold on
    grid(options.grid)
end

sgtitle(options.title,Interpreter='latex',FontSize=options.fontsize) 
xlabel(options.xlabel,Interpreter='latex',FontSize=options.fontsize)
end
