function symbolic = ditch_small_terms(input,threshold)

symbolic  = mapSymType(input, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x));

end