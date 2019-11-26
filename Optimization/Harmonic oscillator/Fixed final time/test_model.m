function err = test_model (l0)
    global xf x0 tf;
    
    si = [x0; l0];
    opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
    
    [~, s] = ode45 (@(t,s) test_state(t,s), [0 tf], si, opts);
    err = norm(xf- s(end,1:2)',2);
    
end