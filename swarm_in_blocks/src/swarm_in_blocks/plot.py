import logging
from tkinter import * 
import tkinter.font as font
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg)
import numpy as np
import os
import sys
import json
from PIL import ImageTk, Image

# Defining colors
#background_color = '#9597bf' #roxo claro
background_color = '#d4d4d4' #cinza claro
#background_color = '#868686' #light
grid_color = 'black'
#points_color = '#700b8c' #roxo
points_color = 'red' #vermelho
#points_color = 'orange' #laranja
button_color = 'black'
button_font_color = 'white'
active_button_color = 'gray'

# Buttons functions
def save(self):
    name = self.formation_list['formation {}'.format(len(self.formation_list)-1)]['name']
    coord = self.formation_list['formation {}'.format(len(self.formation_list)-1)]['coord']
    logging.debug("Your last formation was saved. \nType of the formation: {}".format(name))
    # Create a file and save the last coordiante on it (Commented to not create the file everytime for now)
    # with open(os.path.dirname(os.path.abspath(__file__))+'/saved_files/last_formation.npy', 'wb') as f:
    #     np.save(f, coord)
    
    for i in self.formation_list:
        self.formation_list[i]['coord'] = self.formation_list[i]['coord'].tolist()
    with open(os.path.dirname(os.path.abspath(__file__))+'/saved_files/last_formation.json', 'w', encoding='utf-8') as f:
        print(type(self.formation_list))
        json.dump(self.formation_list, f, ensure_ascii=False, indent=4)

    # ESBOÇO PARA FUNÇÃO DE LOAD
    # with open(os.path.dirname(os.path.abspath(__file__))+'/last_formation.npy', 'rb') as f:
    #     loaded_formation = np.load(f)
    # print(loaded_formation)

def next(self, preview_type):
    global formation_count; formation_count += 1
    coord = self.formation_list['formation {}'.format(formation_count)]['coord']
    create_swarm_preview(self, coord, preview_type, first_run=False)

def prev(self, preview_type):
    global formation_count; formation_count -= 1
    coord = self.formation_list['formation {}'.format(formation_count)]['coord']
    create_swarm_preview(self, coord, preview_type, first_run=False)

# Plot functions
def plot_preview2d(self, coord):
    # Define the plot size and colors
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(1, 1, 1)
    fig.patch.set_facecolor(background_color)
    ax.set_facecolor(background_color)

    # Recieves the plot
    plt.plot(coord[:,0], coord[:,1], color=points_color, marker='o', linestyle='')

    # Set the axis, grid and colors
    max_point = int(np.amax(coord[:,0:2]))
    min_point = int(np.amin(coord[:,0:2]))
    if max_point <= 10: max_point=11
    if min_point >= 0: min_point=0
    plt.axis([(min_point-1),(max_point),(min_point-1),(max_point)])
    plt.xticks(np.linspace(min_point, max_point, (max_point+1-min_point)))
    plt.yticks(np.linspace(min_point, max_point, (max_point+1-min_point)))
    ax.tick_params(axis='both', colors=grid_color)
    for spine in ['top', 'right', 'left', 'bottom']:
        ax.spines[spine].set_color(grid_color)
        ax.spines[spine].set_linewidth(3)
    plt.grid(True, color=grid_color)
    return fig

def plot_preview3d(self, coord): 
    # Define the plot size and colors
    fig = plt.figure(figsize=(8, 8))
    #ax = fig.add_subplot(111,projection='3d')
    ax = plt.axes(projection="3d")
    fig.patch.set_facecolor(background_color)
    ax.set_facecolor(background_color)

    # Recieves the plot
    ax.plot(coord[:,0],coord[:,1],coord[:,2],'ro')

    # Set the axis, grid and colors
    max_point = int(np.amax(coord[:,0:3]))
    min_point = int(np.amin(coord[:,0:3]))
    if max_point <= 10: max_point=11
    if min_point >= 0: min_point=0
    ax.set(xlim = ((min_point-1), max_point), ylim = ((min_point-1), max_point), zlim = ((min_point-1), max_point))
    plt.xticks(np.linspace(min_point, max_point, (max_point+1-min_point)))
    plt.yticks(np.linspace(min_point, max_point, (max_point+1-min_point)))
    ax.tick_params(axis='both', colors=grid_color) 
    for spine in ['top', 'right', 'left', 'bottom']:
        ax.spines[spine].set_color(grid_color)
        ax.spines[spine].set_linewidth(3)
    plt.grid(True)
    return fig

# Creating the GUI functions
def create_swarm_preview(self, coord,  preview_type='2D', first_run=True):
    # Main Tkinter window
    window = Tk()
    
    # Setting the title 
    window.title('Formation preview')
    
    # Dimensions of the main window
    window.minsize(400, 300)                                                   # Set minimum dimension values
    window.maxsize(800, 700)                                                   # Set maximum dimension values
    positionRight = int(window.winfo_screenwidth()/2 - 600/2)
    positionDown = int(window.winfo_screenheight()/2 - 500/2)
    window.geometry("600x500+{}+{}".format(positionRight, positionDown))         # Size and distance from top-left

    # Setting the grid and layout
    window.columnconfigure(0, weight=3)
    window.columnconfigure(1, weight=3)
    window.rowconfigure(0, weight=1)
    window.rowconfigure(1, weight=1)
    window.rowconfigure(2, weight=1)
    window.rowconfigure(3, weight=1)
    window.rowconfigure(4, weight=2)
    window.config(background=background_color)

    if preview_type=='2D':
        fig = plot_preview2d(self, coord)
    elif preview_type=='3D':
        fig = plot_preview3d(self, coord)
    else: 
        pass #Retornar código de erro depois

    # Setting the window icon
    icon_path = os.path.dirname(os.path.abspath(__file__)) + "/images/icon.png"
    icon = PhotoImage(file=icon_path)
    window.iconphoto(True, icon)

    # Placing the logo
    logo_path = os.path.dirname(os.path.abspath(__file__)) + "/images/logo.png"
    logo = PhotoImage(file=logo_path)
    logo_label = Label(window, image=logo, background=background_color)
    #icon_label.place(relx=0.9,rely=0.0)
    logo_label.grid(row=0, column=2, padx=15)

    # Creating the Tkinter canvas containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master = window)  
    canvas.draw()
    canvas.get_tk_widget().grid(rowspan=5, columnspan=2, row=0, column=0)
    
    # Checking the first time ploting
    if first_run==True:
        global formation_count
        formation_count = len(self.formation_list)-1
        first_run=False

    # Creating buttons
    but_font = font.Font(family='Poppins', size=11, weight='bold')

    right_arrow_path = os.path.dirname(os.path.abspath(__file__)) + '/images/next.png'
    right_arrow_res = Image.open(right_arrow_path).resize((50, 50))
    right_arrow = ImageTk.PhotoImage(right_arrow_res)

    left_arrow_path = os.path.dirname(os.path.abspath(__file__)) + '/images/prev.png'
    left_arrow_res = Image.open(left_arrow_path).resize((50, 50))
    left_arrow = ImageTk.PhotoImage(left_arrow_res) 

    if len(self.formation_list)-1 == 0:
        pass

    elif formation_count == len(self.formation_list)-1:
        prev_button = Button(master = window, 
                            command = lambda:[window.quit(), window.destroy(), 
                            prev(self,  preview_type)], 
                            height = 50, width = 90,
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            image = left_arrow)
        
        prev_button.grid(row=5, column=0, padx=10, pady=10, sticky=E)
        prev_button.config(highlightthickness=0)

    elif formation_count == 0:        
        next_button = Button(master = window, 
                            command = lambda:[window.quit(), window.destroy(), 
                            next(self,  preview_type)], 
                            height = 50, width = 90, 
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            image = right_arrow)

        next_button.grid(row=5, column=1, padx=10, pady=10, sticky=W)
        next_button.config(highlightthickness=0)

    else :  
        prev_button = Button(master = window, 
                            command = lambda:[window.quit(), window.destroy(), prev(self,  preview_type)], 
                            height = 50, width = 90, 
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            image = left_arrow)

        next_button = Button(master = window, 
                            command = lambda:[window.quit(), window.destroy(), next(self,  preview_type)], 
                            height = 50, width = 90, 
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            image = right_arrow)
                                
        # Placing the buttons on grid
        prev_button.grid(row=5, column=0, padx=15, pady=10, sticky=E)
        prev_button.config(highlightthickness=0)
        next_button.grid(row=5, column=1, padx=15, pady=10, sticky=W)
        next_button.config(highlightthickness=0)

    resume_button = Button(master = window, 
                            command = lambda: [window.quit(), window.destroy()], 
                            height = 2, width = 8, 
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            text = "Resume", font=but_font)
    resume_button.grid(row=1, column=2, padx=15, pady=10, sticky=S)
    resume_button.config(highlightthickness=0)

    save_button = Button(master = window, 
                            command = lambda: [save(self)], 
                            height = 2, width = 8, 
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            text = "Save", font=but_font)
    save_button.grid(row=2, column=2, padx=15, pady=10, sticky=S)
    save_button.config(highlightthickness=0)

    cancel_button = Button(master = window, 
                            command = lambda: [sys.exit()], 
                            height = 2, width = 8, 
                            bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                            text = "Cancel", font=but_font)
    cancel_button.grid(row=3, column=2, padx=15, pady=10, sticky=S)
    cancel_button.config(highlightthickness=0)

    # Configuring the close window protocol
    window.protocol('WM_DELETE_WINDOW', lambda: [window.quit(), window.destroy()])
    window.mainloop()

def plot_init(self):
    # Main Tkinter window
    window = Tk()
    
    # Setting the title 
    window.title('Formation preview')
    
    # Dimensions of the main window
    window.minsize(300, 350)     
    window.maxsize(700, 750)                                                   # Set minimum dimension values
    positionRight = int(window.winfo_screenwidth()/2 - 500/2)
    positionDown = int(window.winfo_screenheight()/2 - 500/2)
    window.geometry("500x550+{}+{}".format(positionRight, positionDown))         # Size and distance from top-left

    # Setting the grid and layout
    window.columnconfigure(0, weight=6)
    window.rowconfigure(0, weight=6)
    window.config(background=background_color)

    # Setting the window icon
    icon_path = os.path.dirname(os.path.abspath(__file__)) + "/images/icon.png"
    icon = PhotoImage(file=icon_path)
    window.iconphoto(True, icon)

    # logo_path = os.path.dirname(os.path.abspath(__file__)) + "/images/logo.png"
    # logo = PhotoImage(file=logo_path)
    # icon_label = Label(window, image=logo)
    # #icon_label.place(relx=0.0,rely=0.0)
    # icon_label.grid(row=0, column=0)

    # Recieves the plot
    coord = self.init_formation_coords
    fig = plot_preview2d(self, coord)

    # creating the Tkinter canvas containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master = window)  
    canvas.draw()
    canvas.get_tk_widget().grid(row=0, column=0)
    
    # Creating buttons
    but_font = font.Font(family='Poppins', size=11, weight='bold')

    resume_button = Button(master = window, 
                                command = lambda: [window.quit(), window.destroy()], 
                                height = 2, width = 20, 
                                bg=button_color, fg=button_font_color, activebackground=active_button_color, 
                                text = "Resume", font=but_font)
    resume_button.grid(row=1, column=0, pady = 10)
    resume_button.config(highlightthickness=0)
    
    # Configuring the close window protocol
    window.protocol('WM_DELETE_WINDOW', lambda: [window.quit(), window.destroy()])
    window.mainloop()

def other_ideas():
    #window.resizable(0,0)                                                       # Don't allow to resize the window
    #window.wm_attributes('-topmost', True)                                      # Window stays always on top

    # # Setting the layout
    # menubar = Menu(window)
    # window.config(background=background_color, menu=menubar)

    # # create the Options_menu
    # options_menu = Menu(menubar, tearoff=0)
    # options_menu.add_command(label='Save last formation', command=save)
    # options_menu.add_command(label='Save all the formation history')
    # options_menu.add_command(label='Close')
    # options_menu.add_separator()
    # options_menu.add_command(label='Exit', command=window.destroy)

    # # create the Help menu
    # help_menu = Menu(menubar, tearoff=0)
    # help_menu.add_command(label='Welcome')
    # help_menu.add_command(label='About...')

    # # add the menus to the menubar
    # menubar.add_cascade(label="Options", menu=options_menu)
    # menubar.add_cascade(label="Help", menu=help_menu)

    # path = os.getcwd() + '/images/Preto.png'
    # logo_resized = Image.open(path).resize((500, 250))
    # logo = ImageTk.PhotoImage(logo_resized)
    # window.iconphoto(True, logo)
    # # Setting labels
    # llogo = Label(window, image=logo, anchor=NE, width=100, height=50)
    # llogo.grid(row=0, column=0)
    # llogo.image = logo

    # path = os.getcwd() + '/images/Preto.png'
    # logo = PhotoImage(file=path)
    # window.iconphoto(True, logo)
    # # Setting labels
    # llogo = Label(window, image=logo, anchor=NW, width=20, height=10)
    # llogo.grid(row=0, column=0, sticky=NW)
    # llogo.image = logo
    pass