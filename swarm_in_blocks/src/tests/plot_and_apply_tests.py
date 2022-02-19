from tkinter import * 
#from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
  
# plot function is created for plotting the graph in tkinter window
def plot():

    fig = plt.figure(figsize=(8, 8))
    #plt.subplots_adjust(bottom = 0.2)
    plt.plot([0,1,2,3,4,5],[0,1,2,3,4,5],'ro')
    plt.axis([-1,11,-1,11])
    plt.grid(True)
    plt.xticks(np.linspace(0,10,11))
    plt.yticks(np.linspace(0,10,11))
    #posit = plt.axes([0.4, 0.1, 0.2, 0.05])
    #plt.show(block=False)
  
    # the figure that will contain the plot
    #fig = Figure(figsize = (5, 5))
  
    # list of squares
    #y = [i**2 for i in range(101)]
  
    # adding the subplot
    #plot1 = fig.add_subplot(111)
  
    # plotting the graph
    #plot1.plot(y)
  
    # creating the Tkinter canvas containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master = window)  
    canvas.draw()
  
    # placing the canvas on the Tkinter window
    canvas.get_tk_widget().pack(anchor=W, fill='both', expand=True)
  
    # creating the Matplotlib toolbar
    # toolbar = NavigationToolbar2Tk(canvas, window)
    # toolbar.update()
  
    # # placing the toolbar on the Tkinter window
    # canvas.get_tk_widget().pack()
  
# the main Tkinter window
window = Tk()
  
# setting the title 
window.title('Formation preview')
  
# dimensions of the main window
window.geometry("500x500")
  
# Creating buttons
apply_button = Button(master = window, command = plot, height = 2, width = 10, text = "Apply")
cancel_button = Button(master = window, command = plot, height = 2, width = 10, text = "Cancel")
  
# place the button in main window
apply_button.pack(side='left')
cancel_button.pack(side='left')
  
# run the gui
window.mainloop()