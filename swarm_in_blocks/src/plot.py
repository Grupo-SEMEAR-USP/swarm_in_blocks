from tkinter import * 
#from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
  
# plot function is created for plotting the graph in tkinter window
def plot():
    pass

def plot_full_preview(self):
    # Main Tkinter window
    window = Tk()
    
    # Setting the title 
    window.title('Formation preview')
    
    # Dimensions of the main window
    window.geometry("500x500")

    # Setting the grid configuration
    window.columnconfigure(1, weight=2)
    window.columnconfigure(2, weight=2)
    window.rowconfigure(0, weight=6)

    # Define the plot size
    fig = plt.figure(figsize=(8, 8))

    # Recieves the plot
    coord = self.des_formation_coords
    plt.plot(coord[:,0],coord[:,1],'ro')

    # Set the axis and grid
    plt.axis([-1,11,-1,11])
    plt.xticks(np.linspace(0,10,11))
    plt.yticks(np.linspace(0,10,11))
    plt.grid(True)

    # creating the Tkinter canvas containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master = window)  
    canvas.draw()
  
    # placing the canvas on the Tkinter window
    canvas.get_tk_widget().grid(columnspan=4, row=0, column=0)
  
    # # placing the toolbar on the Tkinter window
    # toolbar = NavigationToolbar2Tk(canvas, window)
    # toolbar.update()
    # canvas.get_tk_widget().pack()
    
    # Creating buttons
    apply_button = Button(master = window, command = self.applyFormation, height = 2, width = 10, text = "Apply")
    cancel_button = Button(master = window, command = window.destroy, height = 2, width = 10, text = "Cancel")
    
    # Placing the buttons on grid
    apply_button.grid(row=1, column=1, sticky=E)
    cancel_button.grid(row=1, column=2, sticky=W)
    
    # Run the gui
    window.mainloop()

def plot_init(self):
    # Main Tkinter window
    window = Tk()
    
    # Setting the title 
    window.title('Formation preview')
    
    # Dimensions of the main window
    window.geometry("500x500")

    # Setting the grid configuration
    window.columnconfigure(1, weight=4)
    window.rowconfigure(0, weight=6)

    # Define the plot size
    fig = plt.figure(figsize=(8, 8))

    # Recieves the plot
    coord = self.init_formation_coords
    plt.plot(coord[:,0],coord[:,1],'ro')

    # Set the axis and grid
    plt.axis([-1,11,-1,11])
    plt.xticks(np.linspace(0,10,11))
    plt.yticks(np.linspace(0,10,11))
    plt.grid(True)

    # creating the Tkinter canvas containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master = window)  
    canvas.draw()
  
    # placing the canvas on the Tkinter window
    canvas.get_tk_widget().grid(columnspan=3, row=0, column=0)
    
    # Creating buttons
    add_command_button = Button(master = window, command = window.destroy, height = 2, width = 20, text = "Add next command")
    add_command_button.grid(row=1, column=1)
    
    # Run the gui
    window.mainloop()