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
        axs[0].plot(iteracion, x_actual, label='X actual')
        axs[0].plot(iteracion, x_deseado, label='X deseado')
        axs[0].plot(iteracion, x_error, label='Error absoluto en el eje X', color='red')
        axs[0].grid(True)
        axs[0].axhline(0, color='black', linewidth=0.7)
        axs[0].set_title('X actual vs X deseado')
        #axs[0].set_xlabel('Iteration')
        axs[0].set_ylabel('Posición en X')
        axs[0].legend()

        # Subplot for Y axis
        axs[1].plot(iteracion, y_actual, label='Y actual')
        axs[1].plot(iteracion, y_deseado, label='Y deseado')
        axs[1].plot(iteracion, y_error, label='Error absoluto en Y', color='red')
        axs[1].grid(True)
        axs[1].axhline(0, color='black', linewidth=0.7)
        axs[1].set_title('Y actual vs Y deseado')
        #axs[1].set_xlabel('Iteration')
        axs[1].set_ylabel('Posición en el eje Y')
        axs[1].legend()

        # Subplot for Z axis
        axs[2].plot(iteracion, z_actual, label='Z actual')
        axs[2].plot(iteracion, z_deseado, label='Z deseado')
        axs[2].plot(iteracion, z_error, label='Error absoluto en Z', color='red')
        axs[2].grid(True)
        axs[2].axhline(0, color='black', linewidth=0.7)
        axs[2].set_title('Z actual vs Z deseado')
        axs[2].set_xlabel('Iteración')
        axs[2].set_ylabel('Posición en el eje Z')
        axs[2].legend()


        plt.figure(2)
        plt.plot(iteracion, q_plot)
        plt.title('Posiciones articulares "q" en el tiempo')
        plt.xlabel('Iteración')
        plt.ylabel('Ángulo (rad)')
        plt.legend(['q0', 'q1', 'q2', 'q3', 'q4'])


        fig = plt.figure(3)
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x_actual, y_actual, z_actual, label='Trayectoria realizada por el NAO')
        ax.plot(x_deseado,y_deseado, z_deseado, label='Trayectoria deseada')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()


        
        # Display plots
        plt.show()
        plt.show()
        plt.show()