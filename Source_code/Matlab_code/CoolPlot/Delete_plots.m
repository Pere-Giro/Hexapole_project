function [ ] = Delete_plots( Array_plot )
%DELETE_PLOTS Summary of this function goes here
%   Detailed explanation goes here

for i=1:max(size(Array_plot))
    delete(Array_plot(i));
end

end

