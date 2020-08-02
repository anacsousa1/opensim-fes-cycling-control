function [u] = cycling_fuzzy( refVel, thisVel, lastError,  Ts )

    % compute error
    thisError = refVel - thisVel;
    thisErrorDerivade = (thisError-lastError)/Ts;

    % scalling factor
    g1 = 1.3;
    g2 = 0.022;
    
    % error*gain
    thisError = thisError*g1;
    thisErrorDerivade = thisErrorDerivade*g2;
    
    % fuzzy control
    if thisError > 1 || thisErrorDerivade > 1
        u = 1;
    elseif thisError < -1 || thisErrorDerivade < -1
        u = -1;
    else
        a = newfis('cycling_fuzzy'); % Build Mamdani Systems
        % input: error
        a.input(1).name = 'error'; a.input(1).range = [-1 1];

        a.input(1).mf(1).name = 'NB'; a.input(1).mf(1).type = 'gaussmf';
        a.input(1).mf(1).params = [0.2 -1];

        a.input(1).mf(2).name = 'NS'; a.input(1).mf(2).type = 'gaussmf';
        a.input(1).mf(2).params = [0.2 -.5];

        a.input(1).mf(3).name = 'Z'; a.input(1).mf(3).type = 'gaussmf';
        a.input(1).mf(3).params = [0.2 0];

        a.input(1).mf(4).name = 'PS'; a.input(1).mf(4).type = 'gaussmf';
        a.input(1).mf(4).params = [0.2 .5];

        a.input(1).mf(5).name = 'PB'; a.input(1).mf(5).type = 'gaussmf';
        a.input(1).mf(5).params = [0.2 1];
        a.input(1).name = 'error';

        % input: change of error
        a.input(2).name = 'change_error'; a.input(2).range = [-1 1];

        a.input(2).mf(1).name = 'NB'; a.input(2).mf(1).type = 'gaussmf';
        a.input(2).mf(1).params = [0.2 -1];

        a.input(2).mf(2).name = 'NS'; a.input(2).mf(2).type = 'gaussmf';
        a.input(2).mf(2).params = [0.2 -.5];

        a.input(2).mf(3).name = 'Z'; a.input(2).mf(3).type = 'gaussmf';
        a.input(2).mf(3).params = [0.2 0];

        a.input(2).mf(4).name = 'PS'; a.input(2).mf(4).type = 'gaussmf';
        a.input(2).mf(4).params = [0.2 .5];

        a.input(2).mf(5).name = 'PB'; a.input(2).mf(5).type = 'gaussmf';
        a.input(2).mf(5).params = [0.2 1];

        %% OUTPUT
        % output: stimulation
        a.output(1).name = 'stimulation'; a.output(1).range = [-1 1];

        a.output(1).mf(1).name = 'NB'; a.output(1).mf(1).type = 'gaussmf';
        a.output(1).mf(1).params = [0.2 -1];

        a.output(1).mf(2).name = 'NS'; a.output(1).mf(2).type = 'gaussmf';
        a.output(1).mf(2).params = [0.2 -.5];

        a.output(1).mf(3).name = 'Z'; a.output(1).mf(3).type = 'gaussmf';
        a.output(1).mf(3).params = [0.2 0];

        a.output(1).mf(4).name = 'PS'; a.output(1).mf(4).type = 'gaussmf';
        a.output(1).mf(4).params = [0.2 .5];

        a.output(1).mf(5).name = 'PB'; a.output(1).mf(5).type = 'gaussmf';
        a.output(1).mf(5).params = [0.2 1];

        %% RULES
        a.rule = [];
        ruleList = [ 1 1 1 1 1;
                     2 1 1 1 1;
                     3 1 1 1 1;
                     4 1 2 1 1;
                     5 1 3 1 1;
                     1 2 1 1 1;
                     2 2 1 1 1;
                     3 2 2 1 1;
                     4 2 3 1 1;
                     5 2 4 1 1;
                     1 3 1 1 1;
                     2 3 2 1 1;
                     3 3 3 1 1;
                     4 3 4 1 1;
                     5 3 5 1 1;
                     1 4 2 1 1;
                     2 4 3 1 1;
                     3 4 4 1 1;
                     4 4 5 1 1;
                     5 4 5 1 1;
                     1 5 3 1 1;
                     2 5 4 1 1;
                     3 5 5 1 1;
                     4 5 5 1 1;
                     5 5 5 1 1;];
        a = addrule(a,ruleList);

        u = evalfis([thisError thisErrorDerivade],a); %eval fuzzy
    end
end

