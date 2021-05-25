function [ lsf,time ] = NewtonMeth2( ls0,p,rpy,m )
% NEWTONMETH the aim of this second newton meth i given a vector of p,rpy
% (in column) [summary solving IKP]
% and startitng ls0(cable lengths)(column to) uses Newton method to find  the correct
% value of cables that matches ls lenghts.  Flag is a
% string input that can be Analytic or DifFin.

%activateing profile to track execution time
%profile on
tic;

%precision you want to have in your solution

ep=1e-5; %400


%limit number of iterations to avoid infinite loop

t0=0;
tlim=1000;

%defining our x coordenates 
x=[p;rpy];

%creating the 2 possible options, Analytic or DifFin

        %Fxo will be the value of our function for xold
        Fxo=kinematiceq2(ls0,x(1:3),x(4:6),m.A);

        %If the value of Fxo is already 0 we can end here (warning the precision
        %here must be a bigger or equal numer than the one used in dx)

        if max(abs(Fxo))<ep
            lsf=ls0;
            disp('number of iter. = ')
            disp(t0)
            %DISPLAYING THE FINAL VALUE OF Fxo 
            disp(Fxo)
        else
            %Fx will be the Jacobian in xold point
            Fx=JacobianFinDif2(ls0,x(1:3),x(4:6),m.A);
            %inverting Jacobian
            iFx=pinv(Fx);
            %solving the linear system
            dx=-iFx*Fxo;
            %updating the value of xold
            ls0=ls0+dx;
            

            %iteration loop with 1e-9 tolerance error for instance
            while max(abs(Fxo))>ep
                t0=t0+1;
                Fx=JacobianFinDif2(ls0,x(1:3),x(4:6),m.A);
                Fxo=kinematiceq2(ls0,x(1:3),x(4:6),m.A);

                %If the value of Fxo is already 0 we can end here
                %inverting Jacobian
                iFx=pinv(Fx);
                %solving the linear system
                dx=-iFx*Fxo;
                %updating the value of xold
                ls0=ls0+dx;

                if t0>tlim
                    disp('exceeded limit time')
                    break
                end
                lsf=ls0;
            end
            %uncomment if you want dx Fxo and its modules to be displayed
            dx
            norm(dx)
            Fxo
            norm(Fxo)

            disp('number of iter. = ')
            disp(t0)
            %DISPLAYING THE FINAL VALUE OF Fxo 
            disp(Fxo)
        end
        
time=toc;
%profile viewer   % to stop the profiler and get the profile report
end

