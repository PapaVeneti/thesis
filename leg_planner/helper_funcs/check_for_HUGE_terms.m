function symbolic = check_for_HUGE_terms(input,threshold)
%NOT WORKING
symbolic  = mapSymType(input, 'vpareal', @(x) piecewise(abs(x)>=threshold, 0, x));

end