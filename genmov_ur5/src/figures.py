import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def figures(x_real,x_actual,x_error,x_deseado,y_real,y_actual,y_error,y_deseado,z_real,z_actual,z_error,z_deseado,q_plot):

        iteracion = np.linspace(0, len(x_actual)-1, len(x_actual))
        x_error = [abs(x) for x in x_error]
        y_error = [abs(x) for x in y_error]
        z_error = [abs(x) for x in z_error]



        fig, axs = plt.subplots(3, 1, figsize=(10, 15))
        # Subplot for X axis
        axs[0].plot(iteracion, x_actual, label='Current X')
        axs[0].plot(iteracion, x_deseado, label='Desired X')
        axs[0].plot(iteracion, x_error, label='Absolute error in X axis', color='red')
        axs[0].grid(True)
        axs[0].axhline(0, color='black', linewidth=0.7)
        axs[0].set_title('Current X vs Desired X')
        #axs[0].set_xlabel('Iteration')
        axs[0].set_ylabel('Position in X axis')
        axs[0].legend()

        # Subplot for Y axis
        axs[1].plot(iteracion, y_actual, label='Current Y')
        axs[1].plot(iteracion, y_deseado, label='Desired Y')
        axs[1].plot(iteracion, y_error, label='Absolute error in Y axis', color='red')
        axs[1].grid(True)
        axs[1].axhline(0, color='black', linewidth=0.7)
        axs[1].set_title('Current Y vs Desired Y')
        #axs[1].set_xlabel('Iteration')
        axs[1].set_ylabel('Position in Y axis')
        axs[1].legend()

        # Subplot for Z axis
        axs[2].plot(iteracion, z_actual, label='Current Z')
        axs[2].plot(iteracion, z_deseado, label='Desired Z')
        axs[2].plot(iteracion, z_error, label='Absolute error in Z axis', color='red')
        axs[2].grid(True)
        axs[2].axhline(0, color='black', linewidth=0.7)
        axs[2].set_title('Current Z vs Desired Z')
        axs[2].set_xlabel('Iteration')
        axs[2].set_ylabel('Position in Z axis')
        axs[2].legend()


        plt.figure(2)
        plt.plot(iteracion,q_plot, label=['q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6'])
        plt.title('q en el tiempo')
        plt.xlabel('Iteration')
        plt.ylabel('Ángulo (rad)')
        plt.legend()

        fig = plt.figure(3)
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x_actual, y_actual, z_actual, label='Performed robotic trajectory')
        ax.plot(x_deseado,y_deseado, z_deseado, label='Desired robotic trajectory')
        ax.plot(x_real,y_real, z_real, label='Human trajectory')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()


        
        # Display plots
        plt.show()
        plt.show()
        plt.show()