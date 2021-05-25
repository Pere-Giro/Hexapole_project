function Eomatrixreturn = Eomatrix(x)
Eomatrixreturn = [1 0 0; 0 1 0; 0 0 1; 0 0 0; 0 0 0; 0 0 0; 0 -0.72209e0 * cos(x(7)) * cos(x(8)) -0.72209e0 * sin(x(7)) * cos(x(8)); 0.72209e0 * cos(x(8)) 0.72209e0 * sin(x(7)) * sin(x(8)) -0.72209e0 * cos(x(7)) * sin(x(8));];
end