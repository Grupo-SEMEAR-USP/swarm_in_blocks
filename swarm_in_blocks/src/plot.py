from re import I
from tkinter import * 
from tkinter import ttk
from turtle import color
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
'''
# Defining colors
background_color = '#9597bf' #dark
#background_color = 'white' #dark
#background_color = '#868686' #light
grid_color = 'black'
#points_color = '#ff00dd' #dark1
points_color = 'red' #dark2
#points_color = '#20c000' #light
button_color = 'black'
button_font_color = 'white'
active_button_color = 'gray'

# plot function is created for plotting the graph in tkinter window
def save():
    pass

def next(self, preview_type):
    global i; i += 1
    coord = self.formation_list['formation {}'.format(i)]['coord']
    create_swarm_preview(self, coord, preview_type, first_run=False)

def prev(self, preview_type):
    global i; i -= 1
    coord = self.formation_list['formation {}'.format(i)]['coord']
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
    plt.grid(True)
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

    # Setting the layout
    window.config(background=background_color)

    if preview_type=='2D':
        fig = plot_preview2d(self, coord)
    elif preview_type=='3D':
        fig = plot_preview3d(self, coord)
    else: 
        pass #Retornar código de erro depois

    # creating the Tkinter canvas containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master = window)  
    canvas.draw()
    canvas.get_tk_widget().grid(columnspan=4, row=0, column=0)
    
    if first_run==True:
        global i
        i = len(self.formation_list)-1
        first_run=False

    # Creating buttons
    path_1 = os.getcwd() + '/images/seta_direita.png'
    right_arrow = PhotoImage(file=path_1)

    path_2 = os.getcwd() + '/images/seta_esquerda.png'
    left_arrow = PhotoImage(file=path_2) 

    if i == len(self.formation_list)-1:
        prev_button = Button(master = window, 
                            command = lambda:[window.quit(), window.destroy(), 
                            prev(self,  preview_type)], 
                            height = 40, width = 100,
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            image = left_arrow)
    
        resume_button = Button(master = window, 
                            command = lambda: [window.quit(), window.destroy()], 
                            height = 2, width = 10, 
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            text = "Resume")
        
        prev_button.grid(row=1, column=0, pady=10)
        prev_button.config(highlightthickness=0)
        resume_button.grid(row=1, column=2, pady=10)

    elif i == 0:
        resume_button = Button(master = window, 
                            command = lambda: [window.quit(), window.destroy()], 
                            height = 2, width = 10, 
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            text = "Resume")
        
        next_button = Button(master = window, 
                            command = lambda:[window.quit(), window.destroy(), 
                            next(self,  preview_type)], 
                            height = 40, width = 100, 
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            image = right_arrow)

        next_button.grid(row=1, column=2, pady=10)
        next_button.config(highlightthickness=0)
        resume_button.grid(row=1, column=0, pady=10)

    else :  
        prev_button = Button(master = window, 
                            command = lambda:[window.quit(), window.destroy(), prev(self,  preview_type)], 
                            height = 40, width = 100, 
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            image = left_arrow)

        resume_button = Button(master = window, 
                            command = lambda: [window.quit(), window.destroy()], 
                            height = 2, width = 10, 
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            text = "Resume")

        next_button = Button(master = window, 
                            command = lambda:[window.quit(), window.destroy(), next(self,  preview_type)], 
                            height = 40, width = 100, 
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            image = right_arrow)
                                
        # Placing the buttons on grid
        #apply_button.grid(row=1, column=1, sticky=E)
        prev_button.grid(row=1, column=0, pady=10)
        prev_button.config(highlightthickness=0)
        next_button.grid(row=1, column=2, pady=10)
        next_button.config(highlightthickness=0)
        resume_button.grid(row=1, column=1, pady=10)

    resume_button.config(highlightthickness=0)

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

    # Setting the grid configuration
    window.columnconfigure(1, weight=4)
    window.rowconfigure(0, weight=6)

    # Setting the layout
    menubar = Menu(window)
    window.config(background=background_color, menu=menubar)

    # create the Options_menu
    options_menu = Menu(menubar, tearoff=0)
    options_menu.add_command(label='Save last formation', command=save)
    options_menu.add_command(label='Save all the formation history')
    options_menu.add_command(label='Close')
    options_menu.add_separator()
    options_menu.add_command(label='Exit', command=window.destroy)

    # create the Help menu
    help_menu = Menu(menubar, tearoff=0)
    help_menu.add_command(label='Welcome')
    help_menu.add_command(label='About...')

    # add the menus to the menubar
    menubar.add_cascade(label="Options", menu=options_menu)
    menubar.add_cascade(label="Help", menu=help_menu)

    # Recieves the plot
    coord = self.init_formation_coords
    fig = plot_preview2d(self, coord)

    # creating the Tkinter canvas containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master = window)  
    canvas.draw()
    canvas.get_tk_widget().grid(columnspan=3, row=0, column=0)
    
    # Creating buttons
    path = os.getcwd() + '/images/Preto.png'
    img = PhotoImage(file=path)
    window.iconphoto(True, img)

    resume_button = Button(master = window, 
                                command = lambda: [window.quit(), window.destroy()], 
                                height = 2, width = 20, 
                                bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                                text = "Resume")

    resume_button.grid(row=1, column=1, pady = 10)
    resume_button.config(highlightthickness=0)
    #window.config(highlightbackground = "red", highlightcolor= "red")
    
    window.protocol('WM_DELETE_WINDOW', lambda: [window.quit(), window.destroy()])
    # Run the gui
    window.mainloop()