function progress_bar(start, stop, step, idx)
%PROGRESS_BAR
%   Prints a progress bar to the console.
%
%   Arguments
%   ---------
%    - start -> start index.
%    - stop  -> stop index.
%    - step  -> step to take.
%    - idx   -> current index.
%

last = floor( (stop - start) / step ) * step + start;
perc = floor( idx / (last - start) * 100 );

if idx ~= start
    fprintf('\b\b\b\b\b%3d%%\n', perc);
else
    fprintf('%3d%%\n', perc);
end

end

