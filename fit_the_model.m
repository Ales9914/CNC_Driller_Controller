function [x] = fit_the_model(xdata,ydata,x0,A1,A2)
    fun = @(x)ydata-A1*sin(x(1)*xdata + x(2))-A2*sin(x(3)*xdata + x(4))-x(5);
    x = lsqnonlin(fun,x0);
end

