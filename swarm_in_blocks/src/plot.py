from tkinter import * 
from tkinter import ttk
import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg)

#IDEIA:
#Planning
    #prev
    #next
    #save
    #resume
    #cancel


# plot function is created for plotting the graph in tkinter window
def save():
    pass

def plot_full_preview(self):
    # Main Tkinter window
    window = Tk()
    
    # Setting the title 
    window.title('Formation preview')
    
    # Dimensions of the main window
    window.geometry("500x500")

    #window.iconbitmap('./catkin_ws/src/swarm_in_blocks/swarm_in_blocks/Preto.png')

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
    max_point = int(np.amax(coord[:,0:2]))
    min_point = int(np.amin(coord[:,0:2]))
    if max_point <= 10: max_point=10
    if min_point >= 0: min_point=0
    plt.axis([(min_point-1),(max_point+1),(min_point-1),(max_point+1)])
    plt.xticks(np.linspace(min_point,(max_point-min_point),(max_point+1)))
    plt.yticks(np.linspace(min_point,(max_point-min_point),(max_point+1)))
    plt.grid(True)

    # creating the Tkinter canvas containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master = window)  
    canvas.draw()
  
    # placing the canvas on the Tkinter window
    canvas.get_tk_widget().grid(columnspan=4, row=0, column=0)
    
    # Creating buttons
    #apply_button = Button(master = window, command = self.applyFormation, height = 2, width = 10, text = "Apply")
    save_button = Button(master = window, command = save, height = 2, width = 10, text = "Save")
    resume_button = Button(master = window, command = lambda: [window.quit(), window.destroy()], height = 2, width = 10, text = "Resume")
    
    # Placing the buttons on grid
    #apply_button.grid(row=1, column=1, sticky=E)
    save_button.grid(row=1, column=1, sticky=E)
    resume_button.grid(row=1, column=2, sticky=W)
    
    window.protocol('WM_DELETE_WINDOW', lambda: [window.quit(), window.destroy()])
    # Run the gui
    window.mainloop()


def plot_init(self):
    # Main Tkinter window
    window = Tk()
    
    # Setting the title 
    window.title('Formation preview')
    
    # Dimensions of the main window
    window.geometry("500x500")

    path = os.getcwd() + '/images/Preto.png'
    img = PhotoImage(file=path)
    #window.tk.call('wm', 'iconphoto', window._w, img)
    window.iconphoto(True, img)

    # Setting the grid configuration
    window.columnconfigure(1, weight=4)
    window.rowconfigure(0, weight=6)

    # Define the plot size
    fig = plt.figure(figsize=(8, 8))

    # Recieves the plot
    coord = self.init_formation_coords
    plt.plot(coord[:,0],coord[:,1],'ro')

    # Set the axis and grid
    max_point = int(np.amax(coord[:,0:2]))
    min_point = int(np.amin(coord[:,0:2]))
    if max_point <= 10: max_point=10
    if min_point >= 0: min_point=0
    plt.axis([(min_point-1),(max_point+1),(min_point-1),(max_point+1)])
    plt.xticks(np.linspace(min_point,(max_point-min_point),(max_point+1)))
    plt.yticks(np.linspace(min_point,(max_point-min_point),(max_point+1)))
    plt.grid(True)

    # creating the Tkinter canvas containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master = window)  
    canvas.draw()
  
    # placing the canvas on the Tkinter window
    canvas.get_tk_widget().grid(columnspan=3, row=0, column=0)
    
    # Creating buttons
    add_command_button = Button(master = window, command = lambda: [window.quit(), window.destroy()], height = 2, width = 20, bg='white', activebackground='yellow', text = "Add next command")

    add_command_button.grid(row=1, column=1)
    
    window.protocol('WM_DELETE_WINDOW', lambda: [window.quit(), window.destroy()])
    # Run the gui
    window.mainloop()