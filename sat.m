function sat_value=sat(x,beta)
if beta<0
    error('beta should be non-negative');
else
logicalvector = abs(x) > beta;
sat_value = ~logicalvector.*x + logicalvector.*sign(x)*beta;
end