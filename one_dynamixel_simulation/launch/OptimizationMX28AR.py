import matlab.engine
import threading


def run_matlab_optimization(eng):
    # Call MATLAB as a program in terminal (not used but maybe vauleable for someone)
    # subprocess.Popen(['gnome-terminal', '--','matlab', '-r', 'run("Optimization.m")'])
    # subprocess.Popen(['gnome-terminal', '--','matlab', '-nodisplay', '-nosplash', '-nodesktop', '-r', 'run("Optimization.m")'])

    # Call the MATLAB function through the MATLAB engine
    eng.run('Optimization.m', nargout=0)
    
    # Stop MATLAB Engine (optional)
    # eng.quit()

if __name__ == '__main__':
    # Start the MATLAB engine
    eng = matlab.engine.start_matlab()
    run_matlab_optimization(eng)

    # #Start the optimization script in a separate thread (optional)
    # threading.Thread(target=run_matlab_optimization, args=(eng,)).start()
