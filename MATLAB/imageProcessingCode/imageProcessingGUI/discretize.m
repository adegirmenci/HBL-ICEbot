function [colors,counts] = discretize(idxXc,colors,counts)

colors(idxX) = colors(idxX) + c;
counts(idxX) = counts(idxX) + 1;

end