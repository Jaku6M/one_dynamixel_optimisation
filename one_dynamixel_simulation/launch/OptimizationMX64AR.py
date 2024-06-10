import matlab.engine
import threading


def run_matlab_optimization(eng):

    # Call the MATLAB function through the MATLAB engine
    eng.run('OptimizationMX64AR.m', nargout=0)
    
    # Stop MATLAB Engine (optional)
    # eng.quit()

if __name__ == '__main__':
    # Start the MATLAB engine
    eng = matlab.engine.start_matlab()
    run_matlab_optimization(eng)

    # #Start the optimization script in a separate thread (optional)
    # threading.Thread(target=run_matlab_optimization, args=(eng,)).start()
