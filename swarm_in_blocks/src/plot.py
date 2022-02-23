from re import I
from tkinter import * 
from tkinter import ttk
import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg)

'''
IDEIA:
Buttons:
    prev
    next
    save
    resume
    cancel (just simulation)

Colors:
'''
# Defining colors
background_color = '#5d5d9d'
grid_color = '#000000'
points_color = 'orange'

# plot function is created for plotting the graph in tkinter window
def save():
    pass

def nextel(self, preview_type):
    global i; i += 1
    coord = self.formation_list[i][2]
    create_swarm_preview(self, coord, preview_type, first_run=False)

def prev(self, preview_type):
    global i; i -= 1
    coord = self.formation_list[i][2]
    create_swarm_preview(self, coord, preview_type, first_run=False)

def plot_preview2d(self, coord):
    # Define the plot size
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(1, 1, 1)
    fig.patch.set_facecolor(background_color)
    ax.set_facecolor(background_color)

    # Recieves the plot
    plt.plot(coord[:,0], coord[:,1], color=points_color, marker='o', linestyle='')

    # Set the axis and grid
    max_point = int(np.amax(coord[:,0:2]))
    min_point = int(np.amin(coord[:,0:2]))
    if max_point <= 10: max_point=10
    if min_point >= 0: min_point=0
    plt.axis([(min_point-1),(max_point+1),(min_point-1),(max_point+1)])
    plt.xticks(np.linspace(min_point,(max_point-min_point),(max_point+1)))
    plt.yticks(np.linspace(min_point,(max_point-min_point),(max_point+1)))
    ax.tick_params(axis='both', colors=grid_color)    #setting up X-axis tick color to red
    for spine in ['top', 'right', 'left', 'bottom']:
        ax.spines[spine].set_color(grid_color)
        ax.spines[spine].set_linewidth(3)
    plt.grid(True, color=grid_color)
    return fig

def plot_preview3d(self, coord): 
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111,projection='3d')
    fig.patch.set_facecolor(background_color)
    ax.set_facecolor(background_color)

    ax.plot(coord[:,0],coord[:,1],coord[:,2],'ro')
    max_point = int(np.amax(coord[:,0:3]))
    min_point = int(np.amin(coord[:,0:3]))
    if max_point <= 10: max_point=10
    if min_point >= 0: min_point=0
    ax.set(xlim=((min_point-1),(max_point+1)), ylim=((min_point-1),(max_point+1)), zlim=((min_point-1),(max_point+1)))
    plt.xticks(np.linspace(min_point,(max_point-min_point),(max_point+1)))
    plt.yticks(np.linspace(min_point,(max_point-min_point),(max_point+1)))
    ax.tick_params(axis='both', colors=grid_color) 
    for spine in ['top', 'right', 'left', 'bottom']:
        ax.spines[spine].set_color(grid_color)
        ax.spines[spine].set_linewidth(3)
    plt.grid(True, color='black')
    return fig


def create_swarm_preview(self, coord,  preview_type='2D', first_run=True):
    # Main Tkinter window
    window = Tk()
    
    # Setting the title 
    window.title('Formation preview')
    
    # Dimensions of the main window
    window.minsize(300, 300)     
    window.maxsize(700, 700)                                                   # Set minimum dimension values
    positionRight = int(window.winfo_screenwidth()/2 - 500/2)
    positionDown = int(window.winfo_screenheight()/2 - 500/2)
    window.geometry("500x500+{}+{}".format(positionRight, positionDown))         # Size and distance from top-left

    # Setting the grid configuration
    window.columnconfigure(0, weight=2)
    window.columnconfigure(1, weight=2)
    window.columnconfigure(2, weight=2)
    window.rowconfigure(0, weight=6)

    if preview_type=='2D':
        fig = plot_preview2d(self, coord)
    elif preview_type=='3D':
        fig = plot_preview3d(self, coord)
    else: 
        pass #Retornar código de erro depois

    # creating the Tkinter canvas containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master = window)  
    canvas.draw()
  
    # placing the canvas on the Tkinter window
    canvas.get_tk_widget().grid(columnspan=4, row=0, column=0)
    
    if first_run==True:
        global i
        i = len(self.formation_list)-1
        first_run=False

    # Creating buttons
    # Sumir com next e prev nos valores máximos (ja funcional) (precisa melhorar o posicionamento)
    # Atchim se vc ler isso antes que eu acorde, ta com um problema que quando muda de tela ele nao ativa a aba pra mudar as formacoes

    path_1 = os.getcwd() + '/images/seta_direita.png'
    right_arrow = PhotoImage(file=path_1)

    path_2 = os.getcwd() + '/images/seta_esquerda.png'
    left_arrow = PhotoImage(file=path_2) 

    if i == len(self.formation_list)-1:
        prev_button = Button(master = window, command = lambda:[window.quit(), window.destroy(), prev(self,  preview_type)], height = 40, width = 100, image = left_arrow)
        resume_button = Button(master = window, command = lambda: [window.quit(), window.destroy()], height = 2, width = 10, text = "Resume")
        
        prev_button.grid(row=1, column=0)
        resume_button.grid(row=1, column=2)

    elif i == 0:
        resume_button = Button(master = window, command = lambda: [window.quit(), window.destroy()], height = 2, width = 10, text = "Resume")
        next_button = Button(master = window, command = lambda:[window.quit(), window.destroy(), nextel(self,  preview_type)], height = 40, width = 100, image = right_arrow)

        resume_button.grid(row=1, column=0)
        next_button.grid(row=1, column=2)

    else :
        
        #apply_button = Button(master = window, command = self.applyFormation, height = 2, width = 10, text = "Apply")
        prev_button = Button(master = window, command = lambda:[window.quit(), window.destroy(), prev(self,  preview_type)], height = 40, width = 100, image = left_arrow)
        resume_button = Button(master = window, command = lambda: [window.quit(), window.destroy()], height = 2, width = 10, text = "Resume")
        next_button = Button(master = window, command = lambda:[window.quit(), window.destroy(), nextel(self,  preview_type)], height = 40, width = 100, image = right_arrow)
    
        # Placing the buttons on grid
        #apply_button.grid(row=1, column=1, sticky=E)
        prev_button.grid(row=1, column=0)
        resume_button.grid(row=1, column=1)
        next_button.grid(row=1, column=2)

    
    window.protocol('WM_DELETE_WINDOW', lambda: [window.quit(), window.destroy()])
    # Run the gui
    window.mainloop()


def plot_init(self):
    # Main Tkinter window
    window = Tk()
    
    # Setting the title 
    window.title('Formation preview')
    
    # Dimensions of the main window
    window.minsize(300, 300)     
    window.maxsize(700, 700)                                                   # Set minimum dimension values
    positionRight = int(window.winfo_screenwidth()/2 - 500/2)
    positionDown = int(window.winfo_screenheight()/2 - 500/2)
    window.geometry("500x500+{}+{}".format(positionRight, positionDown))         # Size and distance from top-left
    #window.resizable(0,0)                                                       # Don't allow to resize the window
    #window.wm_attributes('-topmost', True)                                      # Window stays always on top

    path = os.getcwd() + '/images/Preto.png'
    img = PhotoImage(file=path)
    window.iconphoto(True, img)

    # Setting the grid configuration
    window.columnconfigure(1, weight=4)
    window.rowconfigure(0, weight=6)

    # Recieves the plot
    coord = self.init_formation_coords
    fig = plot_preview2d(self, coord)

    # creating the Tkinter canvas containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master = window)  
    canvas.draw()
  
    # placing the canvas on the Tkinter window
    canvas.get_tk_widget().grid(columnspan=3, row=0, column=0)
    
    # Creating buttons
    add_command_button = Button(master = window, command = lambda: [window.quit(), window.destroy()], height = 2, width = 20, bg='lightskyblue', activebackground='yellow', text = "Add next command")

    add_command_button.grid(row=1, column=1)
    
    window.protocol('WM_DELETE_WINDOW', lambda: [window.quit(), window.destroy()])
    
    # Run the gui
    window.mainloop()