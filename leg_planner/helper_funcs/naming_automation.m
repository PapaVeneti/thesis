function str_cell = naming_automation(quantity,subscripts,prefix,suffix)
%naming_automation: This function returns a cell array with strings in the
%form: 'prefix'+'quantity'+'subscript'+'suffix', where 
% 1. `quantity` is the plotted quantity 
% 2. `subscript` is the corresponding subscript
% 3. and prefix and suffixes are to modify the string. 
%
% A. Usefull for SUBPLOTS 
% B. Add `$` for latex
%
% Example usage: 
% ```
%   indices = {'motor1','motor2','motor3'};
%   naming_automation('\omega',indices,'$sim:\ ','$');
%```
%SEE ALSO:
%   populate_plot, initialize_plot

N = length(subscripts);
str_cell = cell(N,1);

for i =1:N
    str_cell{i} = [prefix,quantity,'_{',subscripts{i},'}',suffix];
end


